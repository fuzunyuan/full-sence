import numpy as np
import matplotlib.pyplot as plt


wheel_angles_new = []


with open('./detectLogs/2024-01-05 23:04:55.945265.txtangle.txt', 'r') as file:
    n = 0
    for line in file:
        s = line.strip()  # 使用 strip() 去除行尾的换行符
        s = s.split(" ")
        wheel_angles_new.append(float(s[-1]))



# Constants
velocity = 1  # speed of the car is constant at 1 unit per time
time_steps_new = len(wheel_angles_new)  # total number of time steps based on the provided angles

# Convert angles from degrees to radians for computation
wheel_angles_rad_new = np.radians(wheel_angles_new)

# Initialize arrays to store the x and y coordinates
# Assuming the initial position of the car is (0,0) and the initial orientation is 0 (aligned with the x-axis)
x_new = np.zeros(time_steps_new + 1)
y_new = np.zeros(time_steps_new + 1)
theta_new = 0  # Initial orientation of the car

# Simulation loop
for t in range(1, time_steps_new + 1):
    # Update the orientation of the car
    theta_new += velocity * np.tan(wheel_angles_rad_new[t - 1])

    # Update position
    x_new[t] = x_new[t - 1] + velocity * np.cos(theta_new)
    y_new[t] = y_new[t - 1] + velocity * np.sin(theta_new)

# Plot the trajectory
plt.figure(figsize=(10, 6))
plt.plot(x_new, y_new, marker='o')
plt.title("Car Trajectory with Constant Speed and Varying Steering Angles")
plt.xlabel("X position")
plt.ylabel("Y position")
plt.grid(True)
plt.show()

if __name__ == "__main__":
    print("123")