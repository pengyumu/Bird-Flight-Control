

This project aims to design a controller for an agent that manages the movement of a bird in a game similar to the popular "Flappy Bird". The goal is to create a controller that allows the bird to navigate through the pipes for as long as possible, maximizing the accumulation of points. This involves programming the agent to autonomously control the bird's vertical acceleration.

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
```

### Parameters:

- **Kp**: Controls how large the control signal is for a given error.
- **Ki**: Accounts for past errors and integrates them over time.
- **Kd**: Accounts for future error trends based on its current rate of change.
- **Umin and Umax**: The minimum and maximum control signals that can be applied.
  
## PID Controller Input Calculation
After setting up the PID controller, the PID controller’s calc_input method is called with the set point and the process variable as arguments. This method will calculate the appropriate control action based on the difference between the set point and the process variable.


```python
def calculate_the_control_signal(bird: Bird, pipe: Pipe):
    """Calculate the control signal for the bird."""
    sp = pipe.h + pipe.gap / 2
    pv = bird.y + bird.h / 2
    u_jump = pid.calc_input(sp, pv)
    return u_jump
```


<div style="text-align: center;">
  <a href="https://github.com/pengyumu/Bird-Flight-Control/assets/174324735/d0df0b7a-0619-47f1-bda3-0c5a306692d8" target="_blank">
    <img src="path_to_your_thumbnail_image.png" alt="Video" style="width: 600px;">
  </a>
</div>
