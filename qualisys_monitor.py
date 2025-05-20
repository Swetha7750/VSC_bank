#!/usr/bin/env python3

import asyncio
import qtm
import logging
import math
import time
import os
from dataclasses import dataclass
from typing import Optional, Tuple, Callable
import sys

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

@dataclass
class DroneState:
    timestamp: float
    position: Tuple[float, float, float]  # x, y, z in mm
    attitude: Tuple[float, float, float]  # roll, pitch, yaw in radians
    is_tracked: bool

def rotation_matrix_to_euler_xyz(rot):
    """Convert rotation matrix to Euler angles in XYZ sequence (roll, pitch, yaw)"""
    pitch = math.asin(-rot.matrix[2])  # -r13
    
    if abs(rot.matrix[2]) < 0.99999:  # Not at pitch = ±90°
        roll = math.atan2(rot.matrix[5], rot.matrix[8])  # r23/r33
        yaw = math.atan2(rot.matrix[1], rot.matrix[0])   # r12/r11
    else:
        # Gimbal lock case
        roll = 0  # assume roll = 0
        yaw = math.atan2(-rot.matrix[3], rot.matrix[4])  # -r21/r22
        
    return roll, pitch, yaw

class QualisysMonitor:
    def __init__(self, on_state_update: Optional[Callable[[DroneState], None]] = None, print_to_terminal: bool = False):
        """Initialize the Qualisys monitor"""
        self.on_state_update = on_state_update
        self._connection = None
        self._running = False
        self._current_state = None
        self._packet_count = 0
        self._stream_task = None
        self.print_to_terminal = print_to_terminal
        logger.info("QualisysMonitor initialized with callback: %s", "enabled" if on_state_update else "disabled")
        
    def get_latest_state(self) -> Optional[DroneState]:
        """Get the latest drone state"""
        return self._current_state
            
    def _update_state(self, state: DroneState):
        """Update the current state and notify callback if set"""
        self._current_state = state
        if self.on_state_update:
            try:
                self.on_state_update(state)
                if self._packet_count % 50 == 0:  # Log every 50th update
                    logger.debug(f"State update callback called with tracked={state.is_tracked}, pos=({state.position[0]:.1f}, {state.position[1]:.1f}, {state.position[2]:.1f})")
            except Exception as e:
                logger.error(f"Error in state update callback: {e}")
                import traceback
                logger.error(traceback.format_exc())
            
    def _on_packet(self, packet):
        """Handle incoming QTM packets"""
        try:
            self._packet_count += 1
            header, bodies = packet.get_6d()
            if header is None or bodies is None or len(bodies) < 6:
                logger.debug("No 6DOF data in packet")
                self._update_state(DroneState(
                    timestamp=packet.timestamp,
                    position=(0, 0, 0),
                    attitude=(0, 0, 0),
                    is_tracked=False
                ))
                return

            # Get the 6th body (index 5)
            body = bodies[5]
            if body is None:
                logger.debug("Body 6 not found in packet")
                self._update_state(DroneState(
                    timestamp=packet.timestamp,
                    position=(0, 0, 0),
                    attitude=(0, 0, 0),
                    is_tracked=False
                ))
                return
                
            pos = body[0]  # [x, y, z]
            rot = body[1]  # [rotation matrix]
            
            # Check if we have valid position and rotation data
            position_tracked = not (math.isnan(pos.x) or math.isnan(pos.y) or math.isnan(pos.z))
            rotation_tracked = not any(math.isnan(x) for x in rot.matrix)
            
            if position_tracked and rotation_tracked:
                # Get Euler angles in XYZ sequence
                roll, pitch, yaw = rotation_matrix_to_euler_xyz(rot)
                
                state = DroneState(
                    timestamp=packet.timestamp,
                    position=(pos.x, pos.y, pos.z),
                    attitude=(roll, pitch, yaw),
                    is_tracked=True
                )
                
                # Print to terminal if enabled
                if self.print_to_terminal:
                    print('\033[2J\033[H', end='')  # Clear screen
                    print(f"Time: {packet.timestamp} Frame: {packet.framenumber}\n")
                    print(f"Position (mm):")
                    print(f"  X: {pos.x:8.3f}")
                    print(f"  Y: {pos.y:8.3f}")
                    print(f"  Z: {pos.z:8.3f}")
                    print(f"\nRotation (degrees):")
                    print(f"  Roll (X) : {math.degrees(roll):8.3f}")
                    print(f"  Pitch (Y): {math.degrees(pitch):8.3f}")
                    print(f"  Yaw (Z)  : {math.degrees(yaw):8.3f}")
                
                if self._packet_count % 50 == 0:  # Log every 50th packet
                    logger.debug(f"Processing packet {self._packet_count}, position: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})")
                self._update_state(state)
            else:
                if self._packet_count % 50 == 0:  # Log every 50th packet
                    logger.debug(f"Invalid tracking data - position: {position_tracked}, rotation: {rotation_tracked}")
                self._update_state(DroneState(
                    timestamp=packet.timestamp,
                    position=(0, 0, 0),
                    attitude=(0, 0, 0),
                    is_tracked=False
                ))
                
        except Exception as e:
            logger.error(f"Error processing packet: {e}")
            import traceback
            logger.error(traceback.format_exc())
            
    async def _stream_frames(self):
        """Stream frames from QTM in a separate task"""
        try:
            while self._running:
                try:
                    await self._connection.stream_frames(
                        components=["6d"],
                        on_packet=self._on_packet
                    )
                except Exception as e:
                    if self._running:  # Only log if we're still supposed to be running
                        logger.error(f"Error in stream_frames: {e}")
                        await asyncio.sleep(1)  # Wait before retrying
        except asyncio.CancelledError:
            logger.info("Stream frames task cancelled")
            raise
        except Exception as e:
            logger.error(f"Stream frames task failed: {e}")
            raise
            
    async def start(self, qtm_ip: str = "192.168.252.1"):
        """Start the Qualisys monitor"""
        try:
            logger.info(f"Connecting to QTM at {qtm_ip}...")
            self._connection = await qtm.connect(qtm_ip.encode())
            if self._connection is None:
                raise ConnectionError("Failed to connect to QTM")
                
            logger.info("Connected to QTM")
            
            # Start streaming
            logger.info("Starting 6DOF streaming...")
            self._running = True
            
            # Start streaming in a separate task
            self._stream_task = asyncio.create_task(self._stream_frames())
            
        except Exception as e:
            logger.error(f"Error in Qualisys monitor: {e}")
            import traceback
            logger.error(traceback.format_exc())
            self._running = False
            raise
            
    async def stop(self):
        """Stop the Qualisys monitor"""
        self._running = False
        if self._stream_task:
            try:
                self._stream_task.cancel()
                await self._stream_task
            except asyncio.CancelledError:
                pass
            except Exception as e:
                logger.error(f"Error cancelling stream task: {e}")
            self._stream_task = None
            
        if self._connection:
            try:
                logger.info("Disconnecting from QTM...")
                self._connection.disconnect()
                logger.info("Disconnected from QTM")
            except Exception as e:
                logger.error(f"Error disconnecting from QTM: {e}")
            self._connection = None

async def main():
    """Main function when script is run directly"""
    monitor = QualisysMonitor(print_to_terminal=True)  # Enable terminal output when run directly
    try:
        await monitor.start()
    except KeyboardInterrupt:
        logger.info("Script stopped by user")
    except Exception as e:
        logger.error(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    asyncio.run(main()) 