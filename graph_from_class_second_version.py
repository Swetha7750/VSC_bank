import matplotlib.pyplot as plt
import numpy as np

def plot_data_y():
    data = np.loadtxt('trajectory_data_1748022677.csv', delimiter=',', skiprows=1)

    # Unpack columns based on CSV structure
    (
        ts_data,
        x_cmd, y_cmd,
        x_cmd_body, y_cmd_body,
        x_data, y_data,
        x_data_body, y_data_body,
        theta_cmd_rad, theta_cmd_deg,
        phi_cmd_rad, phi_cmd_deg,
        pwm_theta, pwm_phi,
        roll_rad, roll_deg,
        pitch_rad, pitch_deg,
        yaw_rad, yaw_deg
    ) = data.T

    plt.figure(figsize=(18, 20))

    # 1: X Global Frame
    plt.subplot(5, 2, 1)
    plt.plot(ts_data, x_cmd, label="x_cmd", color="orange")
    plt.plot(ts_data, x_data, label="x_data", color="purple")
    plt.xlabel("Time (s)")
    plt.ylabel("X (Global)")
    plt.title("x_cmd & x_data (Global) vs Time")
    plt.grid(True)
    plt.legend()

    # 2: X Body Frame
    plt.subplot(5, 2, 2)
    plt.plot(ts_data, x_cmd_body, label="x_cmd_body", color="blue")
    plt.plot(ts_data, x_data_body, label="x_data_body", color="green")
    plt.xlabel("Time (s)")
    plt.ylabel("X (Body)")
    plt.title("x_cmd_body & x_data_body vs Time")
    plt.grid(True)
    plt.legend()

    # 3: Y Global Frame
    plt.subplot(5, 2, 3)
    plt.plot(ts_data, y_cmd, label="y_cmd", color="darkorange")
    plt.plot(ts_data, y_data, label="y_data", color="darkblue")
    plt.xlabel("Time (s)")
    plt.ylabel("Y (Global)")
    plt.title("y_cmd & y_data (Global) vs Time")
    plt.grid(True)
    plt.legend()

    # 4: Y Body Frame
    plt.subplot(5, 2, 4)
    plt.plot(ts_data, y_cmd_body, label="y_cmd_body", color="blue")
    plt.plot(ts_data, y_data_body, label="y_data_body", color="green")
    plt.xlabel("Time (s)")
    plt.ylabel("Y (Body)")
    plt.title("y_cmd_body & y_data_body vs Time")
    plt.grid(True)
    plt.legend()

    # 5: Theta Command vs Actual (Radians)
    plt.subplot(5, 2, 5)
    plt.plot(ts_data, theta_cmd_rad, label="theta_cmd (rad)", color="red")
    plt.xlabel("Time (s)")
    plt.ylabel("Theta (rad)")
    plt.title("Theta Command vs Time")
    plt.grid(True)
    plt.legend()

    # 6: Phi Command vs Actual (Radians)
    plt.subplot(5, 2, 6)
    plt.plot(ts_data, phi_cmd_rad, label="phi_cmd (rad)", color="maroon")
    plt.xlabel("Time (s)")
    plt.ylabel("Phi (rad)")
    plt.title("Phi Command vs Time")
    plt.grid(True)
    plt.legend()

    # 7: PWM for Theta
    plt.subplot(5, 2, 7)
    plt.plot(ts_data, pwm_theta, label="PWM Theta", color="black")
    plt.xlabel("Time (s)")
    plt.ylabel("PWM")
    plt.title("PWM Signal (Theta) vs Time")
    plt.grid(True)
    plt.legend()

    # 8: PWM for Phi
    plt.subplot(5, 2, 8)
    plt.plot(ts_data, pwm_phi, label="PWM Phi", color="brown")
    plt.xlabel("Time (s)")
    plt.ylabel("PWM")
    plt.title("PWM Signal (Phi) vs Time")
    plt.grid(True)
    plt.legend()

    # 9: Roll, Pitch, Yaw (Degrees)
    plt.subplot(5, 2, 9)
    plt.plot(ts_data, roll_deg, label="Roll (deg)", color="teal")
    plt.plot(ts_data, pitch_deg, label="Pitch (deg)", color="olive")
    plt.plot(ts_data, yaw_deg, label="Yaw (deg)", color="gray")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title("Orientation Angles vs Time")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

# Call the function
plot_data_y()
