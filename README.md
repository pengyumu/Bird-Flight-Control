

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

https://github.com/pengyumu/Bird-Flight-Control/assets/174324735/d0df0b7a-0619-47f1-bda3-0c5a306692d8


## Optimization-based controller 
### Model Predictive Control (MPC) Setup:
- Initialize a Pyomo model with a prediction horizon (N) and a discrete time step (dt).
- Define control variables (u) within specified bounds to represent the bird’s jump intensity over the prediction horizon. Given the gravity -50, here the bounds are set as (40,60), this can help to avoid instability.
- Set up a mutable parameter (T_target) to represent the target height (the midpoint of the gap between pipes) that the bird aims to navigate through.
### Dynamics and Constraints:
- Implement dynamic constraints for the bird’s vertical position (y) and velocity (vy) based on simple physics equations, incorporating the control action (u) and gravity.
- Use Pyomo variables and constraints to model these dynamics within the prediction horizon.
### Objective Function:
Define an objective function to minimize the sum of squared differences between the bird’s predicted positions and the target height, encouraging the bird to stay at the target height.

- #### Minimizing the Distance to Target:  `sum((model.y[k] - model.T_target)**2 for k in model.k)`
- This term represents the sum of squared differences between the bird’s predicted vertical positions (model.y[k]) over the prediction horizon (k in model.k) and the target height (model.T_target, typically the center of the gap between pipes). Squaring the difference serves two purposes: it ensures that the objective function penalizes deviations in both directions (above or below the target) and emphasizes larger errors more heavily than smaller ones, guiding the optimization solver to prioritize solutions that keep the bird closer to the target height.
- #### Ensuring Smooth Control: `0.1 * sum((model.u[k] - model.u[k-1])**2 for k in model.k if k > 1)`
- This term aims to smooth the control actions over the prediction horizon. It calculates the sum of squared differences between consecutive control actions (model.u[k] - model.u[k-1] for each k), penalizing large changes in control effort that could lead to erratic behavior. The coefficient 0.1 scales the influence of this term relative to the first term, balancing the importance of staying near the target versus ensuring smooth control actions. A smaller coefficient would make the controller less concerned about the smoothness of the control action, while a larger one would prioritize smoother control changes at the possible expense of staying close to the target.

Then include a term to minimize the sum of squared differences between consecutive control actions to encourage smooth control actions over time.

### Control Signal Calculation:
- Solve the optimization problem using the IPOPT solver, with options to suppress output for cleaner execution.
- Extract the first control action from the solution as the jump intensity to be applied in the current game frame.

### Adaptive Targeting: 
Dynamically update the target height (T_target) based on the position of the upcoming pipe gap, allowing the controller to adapt to changing game conditions.

```python
# Model parameters
N = 10  # Prediction horizon
dt = 1 / 30  # Time step
gravity = -50

# Create a Pyomo model for MPC
model = pyo.ConcreteModel()
model.k = pyo.RangeSet(1, N)
model.u = pyo.Var(model.k, within=pyo.Reals, bounds=(40, 60))
model.T_target = pyo.Param(initialize=300.0, mutable=True)

def calculate_the_control_signal(bird: Bird, pipe: Pipe) -> float:
    # Delete old model components if they exist
    for v in ['y', 'vy', 'dynamic_constraint_y', 'dynamic_constraint_vy', 'objective']:
        if hasattr(model, v):
            model.del_component(getattr(model, v))
    
    model.y = pyo.Var(model.k, initialize=bird.y)  # Bird position y
    model.vy = pyo.Var(model.k, initialize=bird.vy)  # Bird velocity y
    
    # Update the target position based on the pipe position
    model.T_target = pipe.h + pipe.gap / 2
    
    # Dynamics constraints for the bird's vertical position and velocity
    def dynamic_constraint_y(model, k):
        if k == 1:
            return model.y[k] == bird.y + bird.vy * dt
        else:
            return model.y[k] == model.y[k-1] + model.vy[k-1] * dt
    
    def dynamic_constraint_vy(model, k):
        if k == 1:
            return model.vy[k] == bird.vy + (model.u[k] - gravity) * dt
        else:
            return model.vy[k] == model.vy[k-1] + (model.u[k] - gravity) * dt
    
    model.dynamic_constraint_y = pyo.Constraint(model.k, rule=dynamic_constraint_y)
    model.dynamic_constraint_vy = pyo.Constraint(model.k, rule=dynamic_constraint_vy)
    
    # Objective function to minimize the distance to target and ensure smooth control
    model.objective = pyo.Objective(
        expr=sum((model.y[k] - model.T_target)**2 for k in model.k) + 0.1 * sum((model.u[k] - model.u[k-1])**2 for k in model.k if k > 1), sense=pyo.minimize)
    
    # Solve the optimization problem
    solver = pyo.SolverFactory('ipopt')
    solver.options['print_level'] = 0  # Suppress IPOPT output
    results = solver.solve(model, tee=False)
    
    # Extract the first control signal
    u_jump = pyo.value(model.u[1])
    return u_jump

```
