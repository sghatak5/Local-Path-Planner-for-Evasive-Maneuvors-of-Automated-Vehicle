class PIDController:
    """
    PIDController implements a PID (Proportional-Integral-Derivative) control algorithm.

    Attributes:
        Kp (float): Proportional gain.
        Ki (float): Integral gain.
        Kd (float): Derivative gain.
        dt (float): Time step for simulation.
        integral (float): Accumulated integral error.
        previous_error (float): Error from the previous time step.
        windup_guard (float): Anti-windup guard for integral term.
    """
    def __init__(self, Kp, Ki, Kd, dt, windup_guard=30):
        """
        Initialize the PID controller with given gains and time step.

        Args:
            Kp (float): Proportional gain.
            Ki (float): Integral gain.
            Kd (float): Derivative gain.
            dt (float): Time step for simulation.
            windup_guard (float, optional): Anti-windup guard for integral term. Default is 30.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0
        self.previous_error = 0
        self.windup_guard = windup_guard

    def compute(self, setpoint, current_value):
        """
        Compute the PID control action.

        Args:
            setpoint (float): The desired target value.
            current_value (float): The current measured value.

        Returns:
            float: The control output.
        """
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