
# general imports
import os
import numpy as np

# drake imports
from pydrake.all import DiagramBuilder, LogOutput, Variable, Simulator, PiecewisePolynomial

# local imports
from settings import *
from utils import *
from dynamics import *
from optimizer import *
from plot import *

class Boat:
    def __init__(self, current_scale=1):
        """
        """
        self.symbolic_rep = SymbolicVectorSystem( state=state, input=torques, output=state, dynamics=generate_dynamics(state, torques, current_scale, map_name))
        self.graphic_rep = np.array([[0, l-(w/2)], # center of circular tip
                                     [-w/2, -l], # rectangular hull - bottom left
                                     [w/2, -l], # rectangular hull - bottom right
                                     [w/2, l-(w/2)], # rectangular hull - top right
                                     [-w/2, l-(w/2)]]) # rectangular hull - top left


class Obstacle:
    def __init__(self, start, end, radius, time_steps, time_interval):
        """
            Obstacle Moves from Start to End in simulation
            start: 2x1 (x, y)
            end: 2x1 (x,y)
            
            Example: Obstacle([0,0],[1,1], 5, duration, time_interval)
        """
        self.start = np.array(start).reshape(-1, 1)
        self.end = np.array(end).reshape(-1, 1)
        self.radius = radius
        self.traj = self.add_trajectory(time_steps, time_interval)
        
    def add_trajectory(self, time_steps, time_interval):
        '''
        Generate a starting trajectory guess by drawing a line from 
        start to target.
        '''
        # initial and final time and state
        time_limits = [0., time_steps * time_interval]
        position_limits = np.column_stack((self.start, self.end))

        # linear interpolation in state
        state = PiecewisePolynomial.FirstOrderHold(time_limits, position_limits)

        # sample state on the time grid
        return np.vstack([state.value(t * time_interval).T for t in range(time_steps+1)])


def build_block_diagram(boat, controller):

    builder = DiagramBuilder()
    boat = builder.AddSystem(boat)
    boat.set_name('boat')

    # logger to track trajectories
    logger = LogOutput(boat.get_output_port(0), builder)
    logger.set_name('logger')

    # expose boats input as an input port
    builder.ExportInput(boat.get_input_port(0))

    # build diagram
    diagram = builder.Build()
    diagram.set_name('diagram')

    return diagram

def simulate(initial_state, const_input, boat, controller, duration):

    diagram = build_block_diagram(boat, controller)

    # set up a simulator
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()

    # set the initial conditions
    boat = diagram.GetSubsystemByName('boat')
    boat_context = diagram.GetMutableSubsystemContext(boat, simulator_context)
    boat_context.get_mutable_continuous_state_vector().SetFromVector(initial_state)

    # set inputs to boat
    simulator_context.FixInputPort(0, const_input)

    # simulate from time zero to duration
    simulator.AdvanceTo(duration)

    return diagram.GetSubsystemByName('logger')


if __name__=='__main__':

    boat = Boat()
    obstacles = [Obstacle(x,y,r,duration,time_interval) for x,y,r in obstacle_params]

    # optimize trajectory
    if run_optimization:
        
        if solver_type == "Ipopt":
            solver = IpoptSolver()
        elif solver_type == "Snopt":
            solver = SnoptSolver()
        
        if two_phase_optimization:
            optimal_states_nocurrent, optimal_thrust_nocurrent = solve_optimization(solver, "no_current", obstacles)
            optimal_states, optimal_thrust = solve_optimization(solver, map_name, obstacles,
                                                                state_guess=optimal_states_nocurrent, 
                                                                thrust_guess=optimal_thrust_nocurrent)
        else:
            optimal_states, optimal_thrust = solve_optimization(solver, map_name, obstacles)
        
        if optimal_states is not None or optimal_thrust is not None:
            x_opt, y_opt, th_opt, x_dot_opt, y_dot_opt, th_dot_opt = optimal_states.T
            metric_times = np.linspace(0, len(x_opt)*time_interval, len(x_opt))
            animate_env(x_opt, y_opt, th_opt, x_dot_opt, y_dot_opt, th_dot_opt, metric_times, boat.graphic_rep, obstacles, frame_skip=5)
            plt.figure(2)
            plot_boat_state_and_thrust(optimal_states, # plot optimization metrics
                                       optimal_thrust,
                                       metric_times,
                                       plot=metric_switches,
                                       normalize=True) 

    # simulate trajectory
    if run_simulation:
        logger = simulate(start, [0, 0], boat.symbolic_rep, None, duration)
        x_sim, y_sim, th_sim, x_dot_sim, y_dot_sim, th_dot_sim = logger.data()[:6]
        metric_times = logger.sample_times()
        animate_env(x_sim, y_sim, th_sim, x_dot_sim, y_dot_sim, th_dot_sim, metric_times, boat.graphic_rep, obstacles, frame_skip=20)






