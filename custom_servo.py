import pigpio

class Servo:
    """
    A class for controlling a servo motor using PWM with the pigpio library.
    """

    # Constants for pulse width limits and PWM frequency
    MAX_PW = 1250  # Maximum pulse width for +90 degrees (in microseconds)
    MIN_PW = 250   # Minimum pulse width for -90 degrees (in microseconds)
    _freq = 50     # Frequency for PWM, set to 50Hz (20ms period)

    def _init_(self, pin, min_angle=-90, max_angle=90):
        """
        Initializes the Servo instance with a specified pin and angle range.

        Parameters:
        - pin (int): GPIO pin connected to the servo.
        - min_angle (int): Minimum angle the servo can be set to (default is -90).
        - max_angle (int): Maximum angle the servo can be set to (default is +90).
        """
        # Initialize pigpio and configure the PWM pin and range
        self.pi = pigpio.pi()
        self.pin = pin
        self.pi.set_PWM_frequency(self.pin, self._freq)
        self.pi.set_PWM_range(self.pin, 10000)  # Set PWM range for finer control
        self.angle = 0
        self.max_angle = max_angle
        self.min_angle = min_angle
        # Start with no PWM signal (duty cycle of 0)
        self.pi.set_PWM_dutycycle(self.pin, 0)

    def set_angle(self, angle):
        """
        Sets the servo to the specified angle within the min and max range.

        Parameters:
        - angle (int): Desired angle to set the servo to.
        """
        # Constrain angle to the specified min and max range
        if angle > self.max_angle:
            angle = self.max_angle
        elif angle < self.min_angle:
            angle = self.min_angle
        self.angle = angle
        # Map the angle to an appropriate PWM duty cycle
        duty = self.map(angle, -90, 90, self.MIN_PW, self.MAX_PW)
        self.pi.set_PWM_dutycycle(self.pin, duty)

    def get_angle(self):
        """
        Returns the current angle of the servo.

        Returns:
        - (int): Current angle of the servo.
        """
        return self.angle

    def map(self, x, in_min, in_max, out_min, out_max):
        """
        Maps a value from one range to another.

        Parameters:
        - x (float): The input value to map.
        - in_min (float): Minimum value of the input range.
        - in_max (float): Maximum value of the input range.
        - out_min (float): Minimum value of the output range.
        - out_max (float): Maximum value of the output range.

        Returns:
        - (float): Mapped value in the output range.
        """
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min