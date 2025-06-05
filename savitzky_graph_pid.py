import matplotlib.pyplot as plt
import numpy as np
import math
import matplotlib.ticker as ticker
from matplotlib.ticker import FormatStrFormatter

def plot_data_y():
    data = np.loadtxt('trajectory_data_nine.csv', delimiter=',', skiprows=1)

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
        yaw_rad, yaw_deg, 
        x_integral, y_integral,
        x_error, y_error,
        kp_x, kp_y,
        ki_x, ki_y, 
        kd_x, kd_y,
        not_filt
        

        

    ) = data.T

    plt.figure(figsize=(18, 20))

    # 1: X Global Frame
    plt.subplot(8,2, 1)
    plt.plot(ts_data, x_cmd_body, label="x_set", color="red")
    plt.plot(ts_data, x_data_body, label="x_data", color="green")
    #plt.ylim(-0.5, 0.5)
    plt.yticks([-0.4, -0.2, 0, 0.2, 0.4])

   
    #plt.plot(ts_data, x_error, label="x_error", color="red", linestyle="dashed")
    # plt.xlabel("Time (s)")
    # plt.ylabel("X (Global)")
    # plt.title("x_cmd & x_data (Global) vs Time")
    plt.grid(True)
    plt.legend()

    # 2: X Body Frame
    plt.subplot(8,2, 3)
    plt.plot(ts_data, x_error, label="x_error", color="black")
    #plt.plot(ts_data, x_data_body, label="x_data_body", color="green")
    plt.xlabel("Time (s)")
    #plt.ylabel("X (error)")
    #plt.ylim(-0.5, 0.5)
    plt.yticks([-0.6, -0.2, 0, 0.2, 0.6])
    #plt.title("X_error & x_data_body vs Time")
    plt.grid(True)
    plt.legend()

     #4: Y Body Frame
    plt.subplot(8,2, 5)
    plt.plot(ts_data, theta_cmd_deg, label="Theta_cmd", color="red")
    plt.plot(ts_data, pitch_deg, label="Theta", color="green")
    # plt.xlabel("Time (s)")
    # plt.ylabel("control_x")
    #plt.title("control_x vs Time")

    #lt.ylim(-7, 7)
    plt.yticks([-6, -4, -2, 0, 2, 4, 6])
    plt.grid(True)
    plt.legend()
 
    # using format string '{:.0f}' here but you can choose others
    plt.gca().yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

    plt.subplot(8,2, 7)
    plt.plot(ts_data, kp_x, label="p_x", color="midnightblue")
    #plt.plot(ts_data, x_data_body, label="x_data_body", color="green")
    plt.xlabel("Time (s)")
    ymin, ymax = plt.ylim()
    yrange = max(abs(ymin), abs(ymax))
    plt.ylim(-yrange, yrange)
    #plt.ylabel("X (error)")
    #plt.title("X_error & x_data_body vs Time")
    plt.grid(True)
    plt.legend()

    plt.subplot(8,2, 9)
    plt.plot(ts_data, ki_x, label="i_x", color="midnightblue")
    #plt.plot(ts_data, x_data_body, label="x_data_body", color="green")
    plt.xlabel("Time (s)")
    ymin, ymax = plt.ylim()
    yrange = max(abs(ymin), abs(ymax))
    plt.ylim(-yrange, yrange)
    #plt.title("X_error & x_data_body vs Time")
    plt.grid(True)
    plt.legend()

    plt.subplot(8,2, 11)
    plt.plot(ts_data, kd_x, label="d_x", color="red")
    plt.plot(ts_data, not_filt, label="not_filt", color="grey")
    #plt.plot(ts_data, x_data_body, label="x_data_body", color="green")
    plt.xlabel("Time (s)")
    # ymin, ymax = plt.ylim()
    # yrange = max(abs(ymin), abs(ymax))
    # plt.ylim(-yrange, yrange)
    # t = np.arange(0, max(ts_data), 0.04) 
    # plt.xticks(t)
    
    #plt.ylabel("X (error)")
    #plt.title("X_error & x_data_body vs Time")
    plt.grid(True)
    plt.legend()

     # 7: PWM for Theta
    plt.subplot(8,2, 15)
    # rc_pwm = np.round(1500 + (theta_cmd_rad / 0.523599) * 500).astype(int)
    plt.plot(ts_data, pwm_theta, label="PWM Theta", color="blue")
    # plt.plot(ts_data, rc_pwm, label="Commanded PWM", color="blue")  # Changed label and color to distinguish
    # plt.xlabel("Time (s)")
    # plt.ylabel("PWM")
    # plt.title("PWM Signal (Theta) vs Time")
    
    plt.grid(True)
    plt.legend()


    # 3: Y Global Frame
    plt.subplot(8,2, 2)
    plt.plot(ts_data, y_cmd, label="y_set", color="red")
    plt.plot(ts_data, y_data, label="y_data", color="green")
    plt.ylim(-0.5, 0.5)
    plt.yticks([-0.4, -0.2, 0, 0.2, 0.4])
    # plt.xlabel("Time (s)")
    # plt.ylabel("Y (Global)")
    # plt.title("y_cmd & y_data (Global) vs Time")
    plt.grid(True)
    plt.legend()

      # 2: X Body Frame
    plt.subplot(8,2, 4)
    plt.plot(ts_data, y_error, label="y_error", color="black")
    plt.ylim(-0.5, 0.5)
    plt.yticks([-0.6, -0.2, 0, 0.2, 0.6])
    #plt.plot(ts_data, x_data_body, label="x_data_body", color="green")fig = plt.figure(figsize=(18, 20))
    plt.xlabel("Time (s)")
   # plt.ylabel("Y (error)")
    #plt.title("X_error & x_data_body vs Time")
    plt.grid(True)
    plt.legend()

   
    plt.subplot(8, 2, 6)
    plt.plot(ts_data, phi_cmd_deg, label="Phi_cmd", color="red")
    plt.plot(ts_data, roll_deg, label="Phi", color="green")

    # Force y-axis to be centered around 0
    plt.ylim(-7, 7)
    plt.yticks([-6, -4, -2, 0, 2, 4, 6])

    # Clean, safe Y-tick formatting
    plt.gca().yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

    plt.grid(True)
    plt.legend()

    plt.subplot(8, 2, 8)
    plt.plot(ts_data, kp_y, label="p_y", color="teal")
    ymin, ymax = plt.ylim()
    yrange = max(abs(ymin), abs(ymax))
    plt.ylim(-yrange, yrange)
    #plt.plot(ts_data, x_data_body, label="x_data_body", color="green")
    # plt.xlabel("Time (s)")
    # plt.ylabel("X (error)")
    #plt.title("X_error & x_data_body vs Time")
    plt.grid(True)
    plt.legend()

    plt.subplot(8,2, 10)
    plt.plot(ts_data, ki_y, label="i_y", color="teal")
    ymin, ymax = plt.ylim()
    yrange = max(abs(ymin), abs(ymax))
    plt.ylim(-yrange, yrange)
    #plt.plot(ts_data, x_data_body, label="x_data_body", color="green")
    plt.xlabel("Time (s)")
    #plt.ylabel("X (error)")
    #plt.title("X_error & x_data_body vs Time")
    plt.grid(True)
    plt.legend()

    plt.subplot(8,2, 12)
    plt.plot(ts_data, kd_y, label="d_y", color="teal")
    ymin, ymax = plt.ylim()
    yrange = max(abs(ymin), abs(ymax))
    plt.ylim(-yrange, yrange)
    #plt.plot(ts_data, x_data_body, label="x_data_body", color="green")
    plt.xlabel("Time (s)")
    #plt.ylabel("X (error)")
    #plt.title("X_error & x_data_body vs Time")
    plt.grid(True)
    plt.legend()
    

    #  # 9: Roll, Pitch, Yaw (Degrees)
    # plt.subplot(8, 2, 14)
    # #plt.plot(ts_data, roll_deg, label="Roll (deg)", color="teal")
    # plt.plot(ts_data, yaw_rad, label="Yaw", color="black")
    # ymin, ymax = plt.ylim()
    # yrange = max(abs(ymin), abs(ymax))
    # plt.ylim(-yrange, yrange)
    # # plt.xlabel("Time (s)")
    # # plt.ylabel("Angle (rad)")
    # # plt.title("Orientation Angles vs Time")
    # plt.grid(True)
    # plt.legend()
    

  

   

    # 8: PWM for Phi
    plt.subplot(8, 2, 16)
    plt.plot(ts_data, pwm_phi, label="PWM Phi", color="blue")
    
    # plt.xlabel("Time (s)")
    # plt.ylabel("PWM")
    # plt.title("PWM Signal (Phi) vs Time")
    plt.grid(True) 
    plt.legend()

   


    # plt.subplot(6, 2, 10)
    # #plt.plot(ts_data, roll_deg, label="Roll (deg)", color="teal")
    # plt.plot(ts_data, x_integral, label="x_intergal", color="orange")
    # plt.xlabel("Time (s)")
    # plt.ylabel("x_integral ()")
    # plt.title("x_integral vs Time")
    # plt.grid(True)
    # plt.legend()

    # plt.subplot(6, 2, 11)
    # #plt.plot(ts_data, roll_deg, label="Roll (deg)", color="teal")
    # plt.plot(ts_data, y_integral, label="x_intergal", color="blue")
    # plt.xlabel("Time (s)")
    # plt.ylabel("y_integral ()")
    # plt.title("y_integral vs Time")
    # plt.grid(True)
    # plt.legend()

    # plt.subplot(6, 2, 10)
    # plt.plot(ts_data, x_error, label="x_error", color="teal")
    # plt.plot(ts_data, y_error, label="y_error", color="blue")
    # plt.xlabel("Time (s)")
    # plt.ylabel("error ()")
    # plt.title("error vs Time")
    # plt.grid(True)
    # plt.legend()
  # # 5: Theta Command vs Actual (Radians)
    # plt.subplot(6, 2, 5)
    # plt.plot(ts_data, theta_cmd_rad, label="theta_cmd (rad)", color="red")

    # #plt.plot(ts_data, pitch_rad, label="Pitch (rad)", color="olive")
    # # plt.xlabel("Time (s)")
    # # plt.ylabel("Theta (rad)")
    # # plt.title("Theta Command vs Time")
    # plt.grid(True)
    # plt.legend()
    # # after plotting the data, format the labels
    # current_values = plt.gca().get_yticks() 
    # # using format string '{:.0f}' here but you can choose others
    # plt.gca().set_yticklabels(['{:.6f}'.format(x) for x in current_values])

    # # 6: Phi Command vs Actual (Radians)
    # plt.subplot(6, 2, 6)
    # plt.plot(ts_data, phi_cmd_rad, label="phi_cmd (rad)", color="maroon")
    # # plt.plot(ts_data, roll_rad/math.pi*180, label="Roll (rad)", color="teal")
    # # plt.xlabel("Time (s)")
    # # plt.ylabel("Phi (rad)")
    # # plt.title("Phi Command vs Time")
    # plt.grid(True)
    # plt.legend()
    # # after plotting the data, format the labels
    # current_values = plt.gca().get_yticks() 
    # # using format string '{:.0f}' here but you can choose others
    # plt.gca().set_yticklabels(['{:.6f}'.format(x) for x in current_values])


    plt.show()



#theta_plot() 
plot_data_y()


