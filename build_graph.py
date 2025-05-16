
import matplotlib.pyplot as plt
import numpy as np 



def plot_data_y():
    data = np.loadtxt('trajectory_data.csv', delimiter=',', skiprows=1)
    ts_data = data[:, 0]
    x_cmd_data = data[:, 1]
    XX_data = data[:, 2]
    y_cmd_data = data[:, 3]
    yy_data = data[:, 4]
    theta_cmd_data = data[:, 5]
    phi_cmd_data =data[:,6]
    pwm_theta = data[:,7]
    pwm_phi = data[:,8]
    roll_angle =data[:,9]
    pitch_angle =data[:,11]

    plt.figure(figsize=(14, 12))
    print("Max timestamp in data:", ts_data.max())

    # Subplot 1: theta_cmd
    plt.subplot(3, 2, 3)
    plt.plot(ts_data, theta_cmd_data, label="theta_cmd", color="red", linewidth=2)
    plt.plot(ts_data, pitch_angle, label="theta", color="blue", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("theta_cmd")
    plt.title("theta over Time")
    plt.xlim(0, max(ts_data))
    plt.grid(True)
    plt.legend()

    # Subplot 2: y_cmd and yy
    plt.subplot(3, 2, 2)
    plt.plot(ts_data, y_cmd_data, label="y_cmd", color="green", linewidth=2)
    plt.plot(ts_data, yy_data, label="y", color="blue", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Y Values")
    plt.title("y_cmd and yy over Time")
    plt.xlim(0, max(ts_data))
    plt.grid(True)
    plt.legend()

    # Subplot 3: x_cmd and XX
    plt.subplot(3, 2, 1)
    plt.plot(ts_data, x_cmd_data, label="x_cmd", color="orange", linewidth=2)
    plt.plot(ts_data, XX_data, label="x", color="purple", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("X Values")
    plt.title("x_cmd and x over Time")
    plt.xlim(0, max(ts_data))
    plt.grid(True)
    plt.legend()

    # Subplot 4: pwm_theta
    plt.subplot(3, 2, 4)
    plt.plot(ts_data, pwm_theta, label="PWM_theta", color="black", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("PWM theta")
    plt.title("PWM_theta over Time")
    plt.xlim(0, max(ts_data))
    plt.grid(True)
    plt.legend()

    # Subplot 5: pwm_phi
    plt.subplot(3, 2, 5)
    plt.plot(ts_data, phi_cmd_data, label="phi_cmd", color="brown", linewidth=2)
    plt.plot(ts_data, roll_angle, label="phi", color="green", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("PWM phi")
    plt.title("phi over Time")
    plt.xlim(0, max(ts_data))
    plt.grid(True)
    plt.legend()

    # Subplot 5: phi_cmd
    plt.subplot(3, 2, 6)
    plt.plot(ts_data, pwm_phi, label="PWM_phi", color="brown", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel(" phi_cmd")
    plt.title("phi_cmd over Time")
    plt.xlim(0, max(ts_data))
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

#plot_data()
plot_data_y()