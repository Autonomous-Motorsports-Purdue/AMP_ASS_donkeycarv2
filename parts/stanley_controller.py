import math
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

class Stanley_Controller():
    def __init__(self, speed):
        self.wheelbase = 1.000506 
        self.speed = speed
        self.speed_fast = 10.0
        self.speed_prev = self.speed_fast
        self.k_e = 0.5 # Cross track error correction higher value causes more agressive adjustments

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def run(self, target_position, speed):
        target_x, target_y, target_z = target_position 
        target_x += 0.6096 #Back wheels to camera
        ##target_x *= 0.75 # Constant for urgency

        self.speed = speed
        # For simplified version assume that the current heading is 0 with the cart located a (0, 0)
        path_heading = math.atan2(target_y, target_x)
        heading_error = path_heading 
        heading_error = self.normalize_angle(heading_error)

        # Since travelling along x-axis cross track error is target_y
        cross_track_error = target_y
        #cross_track_error = y - target_y

        steering_angle = heading_error + math.atan2(self.k_e * cross_track_error, self.speed)

        # Ensure steering does not exceed 70 degrees
        max_steer = 1.22
        steering_angle = max(-max_steer, min(max_steer, steering_angle))
        steering_value = math.degrees(steering_angle) / 18.523
        angular_velocity = (self.speed_prev / self.wheelbase) * math.tan(steering_angle)

        ret_speed = self.speed_prev
        # Reduce speed for sharp turns
        if abs(math.degrees(steering_angle)) > 20 and self.speed_prev > self.speed_fast:
            ret_speed = self.speed_prev * 0.75
        
        # Increase speed for minimal turning
        if abs(math.degrees(steering_angle)) <= 20.0:
            ret_speed = self.speed_fast * 0.5 + 0.5 * self.speed_prev

        self.speed_prev = ret_speed
        
        print(ret_speed)
        return angular_velocity, ret_speed
    
import math

class KinematicBicycleModel:
    def __init__(self, x, y, heading, wheelbase, dt):
        self.x = x
        self.y = y
        self.heading = heading
        self.wheelbase = wheelbase
        self.dt = dt

    def update(self, speed, angular_velocity):
        self.x += speed * math.cos(self.heading) * self.dt
        self.y += speed * math.sin(self.heading) * self.dt
        self.heading += angular_velocity * self.dt
        self.heading = self.normalize_angle(self.heading)

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

def create_path():
    ##path = []
    ##for i in np.arange(0, 100, 0.1):
        ##path.append([i, 4.0])
    ##for i in np.arange(0, 200, 0.1):
      ##  x = i
       ## y = 5 * np.sin(x / 10) + 4
        ##path.append([x, y])
    ##return np.array(path)
    path = []
    # Generate points along a complex sine wave
    for i in np.arange(0, 200, 0.1):
        x = i
        y = 10 * np.sin(x / 20) + 7 * np.cos(x / 5)
        path.append([x, y])
    return np.array(path)


# Use to create a path from a CSV
def create_path_csv(filename):
    df = pd.read_csv(filename)
    path = df[['x', 'y']].to_numpy()
    return path


def find_closest_point(path, x, y):
    distances = np.sqrt((path[:,0]-x)**2 + (path[:,1]-y)**2)
    closest_idx = np.argmin(distances)
    # The Stanley controller uses the closest point, not a lookahead.
    return path[closest_idx]

def simulate_stanley(csv_file):
    dt = 0.01
    total_time = 50.0

    vehicle = KinematicBicycleModel(0.0, 0.0, 0.0, wheelbase=2.0, dt=dt)
    controller = Stanley_Controller(speed=0.45)
    #path = create_path()
    path = create_path_csv(csv_file)

    x_trajectory = [vehicle.x]
    y_trajectory = [vehicle.y]

    current_speed = 0.45

    for _ in np.arange(0, total_time, dt):
        if np.hypot(vehicle.x - path[-1,0], vehicle.y - path[-1,1]) < 0.1:
            print("Reached end of path")
            break
        
        # 1. Find the closest point in the global frame
        closest_point_global = find_closest_point(path, vehicle.x, vehicle.y)
        
        # 2. Transform the closest point to the vehicle's local frame
        # Rotation matrix for a 2D point
        rot_matrix = np.array([
            [math.cos(-vehicle.heading), -math.sin(-vehicle.heading)],
            [math.sin(-vehicle.heading), math.cos(-vehicle.heading)]
        ])
        
        # Translate the point to be relative to the vehicle's position
        translated_point = closest_point_global - np.array([vehicle.x, vehicle.y])
        
        # Rotate the translated point to the vehicle's heading
        closest_point_local = np.dot(rot_matrix, translated_point)
        
        # 3. Pass the local coordinates to the controller
        angular_velocity, speed = controller.run(
            (closest_point_local[0], closest_point_local[1], 0.0), current_speed
        )
        
        vehicle.update(speed, angular_velocity)
        current_speed = speed
        x_trajectory.append(vehicle.x)
        y_trajectory.append(vehicle.y)

    # Plot results
    plt.figure(figsize=(10,6))
    plt.plot(path[:,0], path[:,1], 'r--', label='Target Path')
    plt.plot(x_trajectory, y_trajectory, 'b-', label='Vehicle Trajectory')
    plt.scatter(x_trajectory[0], y_trajectory[0], color='green', marker='o', label='Start')
    plt.scatter(path[-1,0], path[-1,1], color='red', marker='x', label='End')
    plt.title('Stanley Controller Simulation')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()


