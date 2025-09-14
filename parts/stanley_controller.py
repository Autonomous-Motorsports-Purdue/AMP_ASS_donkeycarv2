import math
class Stanley_Controller():
    def __init__(self, speed):
        self.wheelbase = 1.000506 
        self.speed = speed
        self.speed_fast = 0.45
        self.speed_prev = self.speed_fast
        self.k_e = 0.5 #Cross track error correction
        self.k_v = 1.0 #Velocity damping

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.py
        while angle < -math.pi:
            angle += 2 * math.py
        return angle
    
    def run(self, target_position):
        target_x, target_y, target_z = target_position 
        target_y = -target_y
        target_x += 0.6096 #Back wheels to camera
        target_x *= 0.75 # Constant for urgency
        #print(f"{target_x}, {target_y}")
        #targetx, targety = 3, -2.5 
        # we move forward in x 

        #if(velocity == 0):
         #   velocity = 1
        
        v = self.speed

        path_yaw = math.atan2(target_y, target_x)
        yaw_diff = self.normalize_angle(path_yaw)

        cross_track_error = math.sin(-path_yaw) * math.hypot(target_x, target_y)
        yaw_diff_crosstrack = math.atan2(self.k_e * cross_track_error, (self.k_v + v))

        steering_angle = yaw_diff + yaw_diff_crosstrack
        steering_angle = self.normalize_angle(steering_angle)

        max_steer = 1.22
        steering_angle = max(-max_steer, min(max_steer, steering_angle))

        steering_theta_deg = steering_angle * (180.0 / math.pi)
        steering_value = steering_theta_deg / 18.523

        
        # speed control!
        if abs(steering_theta_deg) < 3.0: # if absolute value of steering less than 1.0
            ret_speed = self.speed_fast * 0.5 + 0.5 * self.speed_prev
        else:
            ret_speed = self.speed

        self.speed_prev = ret_speed
        ret_speed = self.speed

        return steering_value, ret_speed

if __name__ == "__main__":
    controller = Stanley_Controller(0.35)
    print(controller.run((2.0, 1.0, 0.0)))


