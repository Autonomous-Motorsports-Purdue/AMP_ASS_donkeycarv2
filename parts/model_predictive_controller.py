import math
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.optimize import minimize
import time

# ================= Kinematic Model =================
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
        self.heading = (self.heading + math.pi) % (2*math.pi) - math.pi

# ================= MPC Controller =================
class MPC_Controller:
    def __init__(self, horizon=20, dt_mpc=0.03, wheelbase=2.0, max_steer=np.radians(35)):
        self.horizon = horizon
        self.dt_mpc = dt_mpc        # MPC prediction timestep
        self.wheelbase = wheelbase
        self.max_steer = max_steer
        self.k_y = 1
        self.k_yaw = 0.75
        self.k_smooth = 0.25
        self.target_speed = 7.5

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2*np.pi) - np.pi

    def predict_trajectory(self, x0, y0, yaw0, steering_sequence):
        x_pred, y_pred, yaw_pred = x0, y0, yaw0
        traj = []
        for steer in steering_sequence:
            yaw_pred += (self.target_speed / self.wheelbase) * math.tan(steer) * self.dt_mpc
            x_pred += self.target_speed * math.cos(yaw_pred) * self.dt_mpc
            y_pred += self.target_speed * math.sin(yaw_pred) * self.dt_mpc
            traj.append((x_pred, y_pred, yaw_pred))
        return traj

    def cost_function(self, steering_sequence, vehicle, path, closest_idx):
        steering_sequence = np.clip(steering_sequence, -self.max_steer, self.max_steer)
        traj = self.predict_trajectory(vehicle.x, vehicle.y, vehicle.heading, steering_sequence)
        cost = 0.0
        idx = closest_idx
        prev_steer = 0.0
        for i, (x_pred, y_pred, yaw_pred) in enumerate(traj):
            idx = min(idx + 1, len(path)-1)
            x_ref, y_ref, yaw_ref = path[idx]
            dyaw = self.normalize_angle(yaw_ref - yaw_pred)
            cost += self.k_y * ((x_ref - x_pred)**2 + (y_ref - y_pred)**2) + self.k_yaw * dyaw**2
            cost += self.k_smooth * (steering_sequence[i] - prev_steer)**2
            prev_steer = steering_sequence[i]
        return cost

    def run(self, vehicle, path, closest_idx):
        x0 = np.zeros(self.horizon)
        bounds = [(-self.max_steer, self.max_steer)] * self.horizon
        res = minimize(self.cost_function, x0, args=(vehicle, path, closest_idx),
                       bounds=bounds, method='SLSQP', options={'ftol':1e-3, 'disp':False, 'maxiter':30})
        best_sequence = res.x
        angular_velocity = (self.target_speed / self.wheelbase) * math.tan(best_sequence[0])
        return angular_velocity, self.target_speed

# ================= Utilities =================
def create_path_csv(filename):
    df = pd.read_csv(filename, sep=';', skiprows=2)
    df.columns = df.columns.str.strip()
    df.rename(columns={'x_m':'x','y_m':'y'}, inplace=True)
    return df[['x','y','psi_rad']].to_numpy()

def find_closest_point(path, x, y):
    distances = np.sqrt((path[:,0]-x)**2 + (path[:,1]-y)**2)
    return np.argmin(distances)

def get_initial_state(path):
    n_segments = 5
    headings = [math.atan2(path[i+1,1]-path[i,1], path[i+1,0]-path[i,0]) for i in range(n_segments)]
    avg_heading = math.atan2(sum(math.sin(h) for h in headings),
                             sum(math.cos(h) for h in headings))
    cam_offset = 0.6096
    start_x = path[0,0] - cam_offset * math.cos(avg_heading)
    start_y = path[0,1] - cam_offset * math.sin(avg_heading)
    return start_x, start_y, avg_heading

# ================= Simulation =================
def simulate_mpc(csv_file):
    start = time.time()
    dt_sim = 0.01
    total_time = 200        # smaller for testing
    path = create_path_csv(csv_file)
    start_x, start_y, start_heading = get_initial_state(path)

    vehicle = KinematicBicycleModel(start_x, start_y, start_heading, 2.0, dt_sim)
    mpc_ctrl = MPC_Controller(horizon=8, dt_mpc=0.1)

    traj_x, traj_y = [vehicle.x], [vehicle.y]
    lap_done = False
    lap_time = None

    for t in np.arange(0, total_time, dt_sim):
        if lap_done:
            break
        idx = find_closest_point(path, vehicle.x, vehicle.y)
        ang_vel, speed = mpc_ctrl.run(vehicle, path, idx)
        vehicle.update(speed, ang_vel)
        traj_x.append(vehicle.x)
        traj_y.append(vehicle.y)

        if t > 5.0 and math.hypot(vehicle.x - path[0,0], vehicle.y - path[0,1]) < 1.0:
            lap_done = True
            lap_time = t

    end = time.time()
    print(end - start)
    plt.figure(figsize=(10,6))
    plt.plot(path[:,0], path[:,1], 'r--', label='Target Path')
    plt.plot(traj_x, traj_y, 'g-', label='MPC Trajectory')
    plt.scatter(traj_x[0], traj_y[0], color='green', marker='o', label='Start')
    plt.scatter(path[-1,0], path[-1,1], color='red', marker='x', label='End')
    if lap_done:
        plt.text(traj_x[-1], traj_y[-1], f"MPC Lap: {lap_time:.2f}s",
                 fontsize=12, color='green', ha='left', va='bottom',
                 bbox=dict(facecolor='white', edgecolor='green', boxstyle='round,pad=0.3'))
    else:
        plt.text(traj_x[-1], traj_y[-1], "MPC: Incomplete", fontsize=12)
    plt.title("MPC Controller with Coarse Prediction Step")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()

if __name__ == "__main__":
    simulate_mpc("tracks/traj_race_cl.csv")