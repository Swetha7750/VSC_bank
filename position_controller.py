#!/usr/bin/env python3

import asyncio
import logging
import math
import time
import os
import sys
import msvcrt  # For Windows keyboard input
from dataclasses import dataclass
from typing import Tuple, Optional
from pymavlink import mavutil
from qualisys_monitor import QualisysMonitor, DroneState

# Configure logging
logging.basicConfig(
    filename='position_controller.log',
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

@dataclass
class PIDController:
    kp: float  # Proportional gain
    ki: float  # Integral gain
    kd: float  # Derivative gain
    output_limits: Tuple[float, float]  # (min, max) output limits
    
    def __init__(self, kp: float, ki: float, kd: float, output_limits: Tuple[float, float]):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
        
    def compute(self, error: float) -> float:
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt > 0:
            # Calculate integral term
            self.integral += error * dt
            
            # Calculate derivative term
            derivative = (error - self.last_error) / dt
            
            # Compute PID output
            output = (self.kp * error + 
                     self.ki * self.integral + 
                     self.kd * derivative)
            
            # Apply output limits
            output = max(min(output, self.output_limits[1]), self.output_limits[0])
            
            # Update state
            self.last_error = error
            self.last_time = current_time
            
            return output
        return 0.0
    
    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

class PositionController:
    def __init__(self):
        """Initialize the position controller"""
        # Initialize PID controllers
        self.x_pid = PIDController(kp=0.01, ki=0.0, kd=0.05, output_limits=(-5, 5))
        self.y_pid = PIDController(kp=0.01, ki=0.0, kd=0.05, output_limits=(-5, 5))
        
        # Target position (in mm)
        self.target_position = (0.0, 0.0)
        
        # RC channel mapping - only pitch and roll, throttle is handled by remote
        self.rc_channels = {
            'roll': 2,    # Channel 1
            'pitch': 3,   # Channel 2
        }
        
        # RC override values (1500 is center/neutral) - only pitch and roll
        self.rc_override = {
            'roll': 1500,
            'pitch': 1500,
        }
        
        # Initialize components
        self.mavlink_connection = None
        self.qualisys = None
        self._update_count = 0
        self._latest_state = None  # Store the latest state
        self._last_state_time = 0  # Track when we last got a state update
        self._connection_attempts = 0
        self._keyboard_task = None
        logger.info("PositionController initialized - throttle control disabled")
        
    def _on_state_update(self, state: DroneState):
        """Callback for Qualisys state updates"""
        try:
            self._latest_state = state  # Store the latest state
            self._last_state_time = time.time()
            self._update_count += 1
            if self._update_count % 50 == 0:  # Log every 50th update
                logger.debug(f"Received {self._update_count} state updates, latest: tracked={state.is_tracked}, pos=({state.position[0]:.1f}, {state.position[1]:.1f}, {state.position[2]:.1f})")
        except Exception as e:
            logger.error(f"Error in state update callback: {e}")
            import traceback
            logger.error(traceback.format_exc())
        
    async def connect_to_drone(self):
        """Connect to the drone and Qualisys"""
        try:
            # Connect to the drone
            logger.info("Connecting to drone...")
            self.mavlink_connection = mavutil.mavlink_connection('tcp:localhost:14550')
            self.mavlink_connection.wait_heartbeat()
            logger.info("Connected to drone")
            
            # Create and start Qualisys monitor
            logger.info("Starting Qualisys monitor...")
            self.qualisys = QualisysMonitor(on_state_update=self._on_state_update)
            
            # Try to connect to Qualisys
            max_attempts = 3
            while self._connection_attempts < max_attempts:
                try:
                    self._connection_attempts += 1
                    logger.info(f"Attempting to connect to Qualisys (attempt {self._connection_attempts}/{max_attempts})...")
                    await self.qualisys.start()
                    logger.info("Qualisys monitor started successfully")
                    
                    # Wait a moment to ensure we're receiving data
                    await asyncio.sleep(1)
                    
                    # Check if we're receiving data
                    if self._latest_state is None:
                        logger.warning("No state data received after connection")
                        if self._connection_attempts < max_attempts:
                            logger.info("Retrying connection...")
                            continue
                        else:
                            logger.error("Failed to receive state data after multiple attempts")
                            return False
                    
                    logger.info("Successfully connected and receiving data")
                    return True
                    
                except Exception as e:
                    logger.error(f"Error connecting to Qualisys (attempt {self._connection_attempts}): {e}")
                    if self._connection_attempts < max_attempts:
                        logger.info("Retrying connection...")
                        await asyncio.sleep(1)  # Wait before retrying
                    else:
                        logger.error("Failed to connect to Qualisys after multiple attempts")
                        return False
            
            return False
            
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            import traceback
            logger.error(traceback.format_exc())
            if self.qualisys:
                try:
                    await self.qualisys.stop()
                except Exception as stop_error:
                    logger.error(f"Error stopping monitor: {stop_error}")
            return False
            
    def set_target_position(self, x: float, y: float):
        """Set the target position for the controller"""
        self.target_position = (x, y)
        self.x_pid.reset()
        self.y_pid.reset()
        
    def update_rc_override(self):
        """Update RC override values based on current position and target"""
        if not self.mavlink_connection:
            self._print_status("ERROR: No MAVLink connection")
            return
            
        if not self.qualisys:
            self._print_status("ERROR: No Qualisys connection")
            return
            
        # Check if we have recent state data
        if self._latest_state is None:
            self._print_status("ERROR: No state data available")
            logger.warning("No state data available - update count: %d, connection attempts: %d", 
                         self._update_count, self._connection_attempts)
            return
            
        # Check if state data is fresh (less than 0.1 seconds old)
        state_age = time.time() - self._last_state_time
        if state_age > 0.1:
            self._print_status(f"ERROR: State data is stale ({state_age:.2f}s old)")
            logger.warning("State data is stale - age: %.2fs, update count: %d", 
                         state_age, self._update_count)
            return
            
        if not self._latest_state.is_tracked:
            self._print_status("ERROR: Drone not being tracked")
            return
            
        # Calculate position errors using the stored state
        x_error = self.target_position[0] - self._latest_state.position[0]  # mm
        y_error = self.target_position[1] - self._latest_state.position[1]  # mm
        
        # Convert position errors to angle commands using PID controllers
        pitch_cmd = self.x_pid.compute(x_error)  # degrees
        roll_cmd = self.y_pid.compute(y_error)   # degrees
        
        # Convert angle commands to RC values
        pitch_rc = 1500 + int(pitch_cmd * (500/45))
        roll_rc = 1500 + int(roll_cmd * (500/45))
        
        # Update RC override values (only pitch and roll)
        self.rc_override['pitch'] = max(1000, min(2000, pitch_rc))
        self.rc_override['roll'] = max(1000, min(2000, roll_rc))
        
        # Print status to terminal using the stored state
        self._print_status(
            "OK",
            self._latest_state.position,
            self.target_position,
            (x_error, y_error),
            (pitch_cmd, roll_cmd),
            (self.rc_override['pitch'], self.rc_override['roll'])
        )
        
        # Send RC override commands - only for pitch and roll
        try:
            # Get current RC values for all channels
            current_channels = [0] * 8  # Initialize all channels to 0
            
            # Set only pitch and roll channels
            for channel, value in self.rc_override.items():
                chan = self.rc_channels[channel]
                current_channels[chan-1] = value  # -1 because channels are 1-based
                
            # Send RC override with only pitch and roll modified
            self.mavlink_connection.mav.rc_channels_override_send(
                self.mavlink_connection.target_system,
                self.mavlink_connection.target_component,
                *current_channels
            )
            logger.debug(f"Sent RC override: pitch={self.rc_override['pitch']}, roll={self.rc_override['roll']}")
        except Exception as e:
            logger.error(f"Error sending RC override: {e}")
            self._print_status(f"ERROR: Failed to send RC commands - {str(e)}")
            
    def _print_status(self, status, current_pos=None, target_pos=None, errors=None, 
                     angle_cmds=None, rc_values=None):
        """Print a formatted status display to the terminal"""
        # Clear screen and move cursor to top
        os.system('cls' if os.name == 'nt' else 'clear')
        
        print("Position Controller Status")
        print("=" * 50)
        print(f"Status: {status}")
        print(f"Time: {time.strftime('%H:%M:%S')}")
        print(f"Updates: {self._update_count}")
        print("-" * 50)
        
        if current_pos and target_pos:
            print("Position (mm):")
            print(f"  Current: X={current_pos[0]:8.1f} Y={current_pos[1]:8.1f} Z={current_pos[2]:8.1f}")
            print(f"  Target:  X={target_pos[0]:8.1f} Y={target_pos[1]:8.1f}")
            
        if errors:
            print("\nPosition Errors (mm):")
            print(f"  X: {errors[0]:8.1f}")
            print(f"  Y: {errors[1]:8.1f}")
            
        if angle_cmds:
            print("\nAngle Commands (degrees):")
            print(f"  Pitch: {angle_cmds[0]:8.1f}")
            print(f"  Roll:  {angle_cmds[1]:8.1f}")
            
        if rc_values:
            print("\nRC Values:")
            print(f"  Pitch: {rc_values[0]:4d} (1500 ± {abs(rc_values[0]-1500):3d})")
            print(f"  Roll:  {rc_values[1]:4d} (1500 ± {abs(rc_values[1]-1500):3d})")
            
        print("\nPress Ctrl+C to stop")
        print("=" * 50)

    async def _handle_keyboard(self):
        """Handle keyboard input in a separate task"""
        while True:
            try:
                if msvcrt.kbhit():  # Check if a key was pressed
                    key = msvcrt.getch()  # Get the key
                    if key == b' ':  # Space bar
                        # Reset PID controllers
                        self.x_pid.reset()
                        self.y_pid.reset()
                        logger.info("PID controllers reset by user (space bar)")
                        print("\nPID controllers reset!")  # Print to terminal
                await asyncio.sleep(0.1)  # Small delay to prevent high CPU usage
            except Exception as e:
                logger.error(f"Error in keyboard handler: {e}")
                await asyncio.sleep(1)  # Longer delay on error
                
    async def run(self):
        """Main control loop"""
        if not await self.connect_to_drone():
            logger.error("Failed to connect, exiting")
            return
            
        logger.info("Starting position control loop")
        try:
            # Start keyboard handler task
            self._keyboard_task = asyncio.create_task(self._handle_keyboard())
            
            while True:
                self.update_rc_override()
                await asyncio.sleep(0.02)  # 50Hz control loop
                
        except KeyboardInterrupt:
            logger.info("Position controller stopped by user")
        except Exception as e:
            logger.error(f"Error in control loop: {e}")
            import traceback
            logger.error(traceback.format_exc())
        finally:
            # Clean up
            if self._keyboard_task:
                self._keyboard_task.cancel()
                try:
                    await self._keyboard_task
                except asyncio.CancelledError:
                    pass
                    
            if self.mavlink_connection:
                try:
                    logger.info("Disabling RC override")
                    self.mavlink_connection.mav.rc_channels_override_send(
                        self.mavlink_connection.target_system,
                        self.mavlink_connection.target_component,
                        0, 0, 0, 0, 0, 0, 0, 0
                    )
                except Exception as e:
                    logger.error(f"Error disabling RC override: {e}")

if __name__ == "__main__":
    try:
        # Create and run the position controller
        logger.info("Starting position controller...")
        controller = PositionController()
        asyncio.run(controller.run())
    except KeyboardInterrupt:
        logger.info("Position controller stopped by user")
    except Exception as e:
        logger.error(f"Failed to start position controller: {e}")
        import traceback
        logger.error(traceback.format_exc())
        print(f"\nError: {e}")
        sys.exit(1) 