class PIDController:
    def __init__(self, Kp, Ki, Kd, dt, windup_guard=30):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0
        self.previous_error = 0
        self.windup_guard = windup_guard

    def compute(self, setpoint, current_value):
        error = setpoint - current_value
        
        # Update integral term with anti-windup mechanism
        self.integral += error * self.dt
        if self.integral > self.windup_guard:
            self.integral = self.windup_guard
        elif self.integral < -self.windup_guard:
            self.integral = -self.windup_guard
        
        # Calculate derivative term
        derivative = (error - self.previous_error) / self.dt
        
        # Compute PID output
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # Update previous error for next iteration
        self.previous_error = error
        
        return output