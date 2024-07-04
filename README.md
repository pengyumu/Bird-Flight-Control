# Bird-Flight-Control

This project aims to design a controller for an agent that manages the movement of a bird in a game similar to the popular "Flappy Bird". The goal is to create a controller that allows the bird to navigate through the pipes for as long as possible, maximizing the accumulation of points. This involves programming the agent to autonomously control the bird's vertical acceleration.

### PID Controller Setup
The PID controller aims to adjust the bird’s flight so that it can navigate through gaps between pipes effectively. The implementation of the PID controller is shown as:
## PID Controller Setup

The PID controller aims to adjust the bird’s flight so that it can navigate through gaps between pipes effectively. The implementation of the PID controller is shown as:

```python
class PIDController:
    kp: float = 0.5
    ki: float = 0.005
    kd: float = 200
    error_accumulator: float = 0
    prev_error: float = 0

    def calc_input(self, sp: float, pv: float, umin: float = 47, umax: float = 53) -> float:
        """Calculate the control signal.
        sp: Set point
        pv: Process variable
        """
        e = sp - pv
        P = 0.5 * e

        self.error_accumulator += e
        I = 0.005 * self.error_accumulator

        D = 200 * (e - self.prev_error)
        self.prev_error = e

        pid = P + I + D

        if pid < umin:
            u = umin
        elif pid > umax:
            u = umax
        else:
            u = pid

        return u
