
# general imports
import numpy as np

# drake imports
from pydrake.all import MathematicalProgram, SnoptSolver, IpoptSolver, PiecewisePolynomial
from pydrake.solvers.mathematicalprogram import GetInfeasibleConstraints

# local imports
from settings import *
from utils import wrap_print
from dynamics import generate_dynamics


def get_initial_guess(start, target, duration, time_interval):
    '''
    Generate a starting trajectory guess by drawing a line from 
    start to target, and inferring velocities required to trace
    that line.
    '''
    np.random.seed(1)
    xs, ys, ths , xs_dot, ys_dot, ths_dot = start
    xg, yg, thg, xg_dot, yg_dot, thg_dot = target
    
    #-------------------------- STATE TRAJECTORY GUESS  --------------------------#
    
    # initial and final times/states
    time_limits = [0., duration * time_interval]
    position_limits = np.column_stack((start[:2], target[:2]))
    
    # constant heading guess
    shift_x = (xg - xs) # center coordinates
    shift_y = (yg - ys)
    if shift_x == 0 and shift_y == 0:
        heading_guess = 0.0
    else:
        heading_guess = np.arccos(shift_y/np.sqrt(shift_x**2 + shift_y**2)) 
        if shift_x < 0:
            heading_guess = -heading_guess
    
    position_limits = np.vstack((position_limits, [heading_guess, heading_guess])) # add heading guess
    state_limits = np.vstack((position_limits, np.zeros((3, 2)))) # add zeroed out velocity guesses
    
    # linear interpolation in state
    state = PiecewisePolynomial.FirstOrderHold(time_limits, state_limits)

    # stack position guesses
    traj_guess = np.vstack([state.value(t * time_interval).T for t in range(duration+1)])
    
    # add constant velocity guesses
    x_dot_guess = (traj_guess[-1, 0] - traj_guess[0, 0])/((duration+1)*time_interval)
    y_dot_guess = (traj_guess[-1, 1] - traj_guess[0, 1])/((duration+1)*time_interval)
    th_dot_guess = 0.0
    
    for t in range(duration+1):
        traj_guess[t][3] = x_dot_guess
        traj_guess[t][4] = y_dot_guess
        traj_guess[t][5] = th_dot_guess
    
    #-------------------------- TORQUE INPUT GUESS  --------------------------#
    
    # thrust angle (turn towards, stay straight, then turn to final angle)
    initial_thrust_angle = -(heading_guess - ths)
    final_thrust_angle = (heading_guess - thg)
    if np.abs(initial_thrust_angle) > thrust_lim[1]: # enforce thrust angle limit
        initial_thrust_angle = np.sign(initial_thrust_angle)*thrust_lim[1]
    if np.abs(final_thrust_angle) > thrust_lim[1]:
        final_thrust_angle = np.sign(final_thrust_angle)*thrust_lim[1]
    thrust_headings = np.zeros(duration)
    thrust_headings[:10] = initial_thrust_angle
    thrust_headings[-10:] = final_thrust_angle
    
    # thrust force
    thrust_forces = np.zeros(duration)

    # stack torque guesses
    thrust_guess = np.column_stack((thrust_forces, thrust_headings))

    return traj_guess, thrust_guess
    

def formulate_optimization(environment_name, obstacles):
    '''
    Generate and return an optimization problem. All decision variables,
    constraints, and costs are added here.
    '''
    
    def discrete_dynamics(state, state_next, thrust):
        '''Forward euler method to approximate discretized dynamics'''
        state_dot = generate_dynamics(state, thrust, env_name=environment_name)
        residuals = state_next - state - time_interval*state_dot
        return residuals
    
    def squared_distance_to_obstacle(boat_pos, obs_pos):
        '''Compute the squared distance from the boat to an obstacle'''
        vec = boat_pos - obs_pos
        return vec.dot(vec)
    
    # initialize optimization
    prog = MathematicalProgram()
    opt_state = prog.NewContinuousVariables(duration + 1, 6, 'optimal_state')
    opt_thrust = prog.NewContinuousVariables(duration, 2, 'optimal_thrust')

    if opt_switches[0]:
        # initial state constraint
        for i in range(len(start)):
            prog.AddConstraint(opt_state[0, i] - start[i], 0., 0.).evaluator().set_description("Initial State")

    if opt_switches[1]:
        # goal state constraint
        for i in range(len(start)):
            prog.AddConstraint(opt_state[-1, i] - target[i], -final_tolerances[i], final_tolerances[i]).evaluator().set_description(f"Final State {i}")

    if opt_switches[2]:
        # enforce dynamics
        for t in range(duration):
            residuals = discrete_dynamics(opt_state[t], opt_state[t+1], opt_thrust[t])
            for i, residual in enumerate(residuals):
                prog.AddConstraint(residual, -dynamics_tolerances[i], dynamics_tolerances[i])

    if opt_switches[3]:
        # thrust limits
        for t in range(duration):
            prog.AddConstraint(opt_thrust[t][0], -thrust_lim[0], thrust_lim[0])
            prog.AddConstraint(opt_thrust[t][1], -thrust_lim[1], thrust_lim[1])

    if opt_switches[4]:
        # avoid collisions
        for i, obstacle in enumerate(obstacles):
            for t in range(duration+1):
                d2 = squared_distance_to_obstacle(opt_state[t, 0:2], obstacle.traj[t])
                prog.AddConstraint(d2 >= obstacle.radius**2 + (w/2)**2)

    if opt_switches[5]:
        # fuel cost
        for t in range(duration):
            prog.AddCost(time_interval*opt_thrust[t, 0]**2)
    
    if opt_switches[6]:
        # distance cost
        for t in range(duration):
            prog.AddCost(time_interval*(opt_state[t, 3]**2+opt_state[t, 4]**2))

    return prog, opt_state, opt_thrust


def solve_optimization(solver, env_name, obstacles, state_guess=None, thrust_guess=None):
    '''
    Generate and solve a trajectory optimization problem.
    '''
    # generate optimization problem
    prog, opt_state, opt_thrust = formulate_optimization(env_name, obstacles)
    
    # initial guess
    if state_guess is None and thrust_guess is None:
        traj_guess, thrust_guess = get_initial_guess(start, target, duration, time_interval)
        prog.SetInitialGuess(opt_state, traj_guess)
        prog.SetInitialGuess(opt_thrust, thrust_guess)
    else:
        if state_guess is not None:
            prog.SetInitialGuess(opt_state, state_guess)
        if thrust_guess is not None:
            prog.SetInitialGuess(opt_thrust, thrust_guess)
    
    # solve
    traj_result = solver.Solve(prog)
    if not traj_result.is_success():
        infeasible = GetInfeasibleConstraints(prog, traj_result)
        wrap_print("INFEASIBLE: {}".format(env_name))
        for constraint in infeasible:
            print(constraint)
        optimal_states = None
        optimal_thrust = None
    else:
        wrap_print("SUCCESS: {}".format(env_name))
        optimal_states = traj_result.GetSolution(opt_state)
        optimal_thrust = traj_result.GetSolution(opt_thrust)
     
    return optimal_states, optimal_thrust





