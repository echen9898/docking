
# general imports
import numpy as np

# plotting imports
import matplotlib.pyplot as plt
from matplotlib import rcParams
import matplotlib.animation as animation 
from matplotlib.patches import Rectangle, Circle, Arrow, Polygon
from matplotlib.transforms import Affine2D

# local imports
from settings import *
from utils import get_normalized_vec, rotate
from dynamics import get_current


def plot_boat_state_and_thrust(optimal_state, optimal_thrust, metric_times, plot=(0, 0, 0, 0, 0, 0, 0, 0), normalize=True):
    '''
    Function that plots the norm of the thrust force and thrust force angle.
    '''
    fig2 = plt.figure(2)
    ax2 = fig2.axes[0]
    thrust_time_steps = optimal_thrust.shape[0]
    
    states = [r'$x(t)$', r'$y(t)$', r'$\theta_h(t)$', r'$\dot x(t)$', r'$\dot y(t)$', r'$\dot \theta_h(t)$']
    thrust = [r'$F(t)$', r'$\theta_T(t)$']
    for i, state in enumerate(states):
        if plot[i]:
            vals = get_normalized_vec(optimal_state.T[i]) if normalize else optimal_state.T[i]
            ax2.plot(metric_times, vals, label=state)
    
    for i, thrust in enumerate(thrust):
        if plot[i+len(states)]:
            vals = get_normalized_vec(optimal_thrust.T[i]) if normalize else optimal_thrust.T[i]
            ax2.plot(metric_times[:-1], vals, label=thrust)
    
    # misc settings
    ax2.set_xlabel(r'Time')
    ax2.grid(True)
    ax2.legend()
    
    if save_plots: fig2.savefig('{}/metrics.png'.format(logdir))

def style_plot(ax, x_label, y_label, x, y, plot_limits, mode=None):
    '''
    Apply stylistic attributes to a plot
    '''
    ax.set_xlabel(x_label, labelpad=15)
    ax.set_ylabel(y_label, rotation=0, labelpad=20)
    
    if mode == 'simulation': # as opposed to a metrics plot
        xlim_low, xlim_up, ylim_low, ylim_up = plot_limits
        ax.set_xlim([xlim_low, xlim_up])
        ax.set_ylim([ylim_low, ylim_up])

def plot_boat(boat, x, y, heading, ax=None, **kwargs):
    '''
    Plot the boat as a rectangular patch with an arrow to represent the heading
    '''

    kwargs['edgecolor'] = 'black'
    kwargs['facecolor'] = 'black'
    
    if ax is None:
        ax = plt.gca()
    

    # draw boat hull
    rotated_hull = [rotate(heading, point) + [x, y] for point in boat[1:]] # rotate
    rotated_circ = rotate(heading, boat[0]) + [x, y]

    hull_rectangle = Polygon(rotated_hull, closed=True, **kwargs)
    hull_circle = Circle(rotated_circ, radius=w/2, **kwargs)

    # bottom_left = [x - w/2, y - l]
    # top_right = [x + w/2, y + l]
    # boat_shape = Rectangle(bottom_left, w, 2*l, 0.0, **kwargs)
    
    # # draw boat heading
    # hdx = 2*l*np.cos(np.pi/2 - heading)
    # hdy = 2*l*np.sin(np.pi/2 - heading)
    # boat_heading = Arrow(x, y, hdx, hdy, width=6, **kwargs)

    ax.add_patch(hull_circle)
    ax.add_patch(hull_rectangle)
    
    return [hull_rectangle, hull_circle]

def plot_obstacles(objects, n, ax=None, **kwargs):
    '''
    Plot moving obstacles as solid circles
    '''
    kwargs['edgecolor'] = 'black'
    kwargs['facecolor'] = 'orange'
    
    if ax is None:
        ax = plt.gca()
        
    shapes = list()
    for obstacle in objects:
        x, y = obstacle.traj[n]
        obs_shape = Circle((x, y), obstacle.radius)
        added = ax.add_patch(obs_shape)
        shapes.append(obs_shape)
        
    return shapes

def static_plot(x, y, th, x_dot, y_dot, th_dot, metric_times, boat, objects):
    '''
    Render a static plot of the boat, objects, current, along with trajectories traced
    for each of them. Used to initialize the animation, or just quickly plot simulation
    results without taking the time to animate frames.
    '''
    fig1, ax1 = plt.subplots() # plot environment
    fig2, ax2 = plt.subplots() # plot metrics
    
    # dynamically zoom in
    max_x = np.max(x) # largest x coord in trajectory
    min_x = np.min(x) # smallest x coord in trajectory
    max_y = np.max(y) # largest y coord in trajectory
    min_y = np.min(y) # smallest y coord in trajectory
    traj_diff_x = max_x - min_x
    traj_diff_y = max_y - min_y
    padding_x = traj_diff_x*pad_factor
    padding_y = traj_diff_y*pad_factor
    if padding_x == 0: padding_x = 10*w # if the boat doesn't move (scaling is arbitrary)
    if padding_y == 0: padding_y = 10*l
    xlim_low = min_x-padding_x
    xlim_up = max_x+padding_x
    ylim_low = min_y-padding_y
    ylim_up = max_y+padding_y
    plot_limits = [xlim_low, xlim_up, ylim_low, ylim_up]
    
    # plot current field
    try:
        x_curr = np.arange(xlim_low, xlim_up, int(xlim_up - xlim_low)/num_curr_x + 1)
        y_curr = np.arange(ylim_low, ylim_up, int(ylim_up - ylim_low)/num_curr_y + 1)
        u, v = get_current(x_curr, y_curr, map_name, vectorize=True)
        ax1.quiver(x_curr, y_curr, u, v, color='teal', width=0.0045)
    except:
        x_curr = np.arange(xlim_low, xlim_up, (xlim_up - xlim_low)/num_curr_x)
        y_curr = np.arange(ylim_low, ylim_up, (ylim_up - ylim_low)/num_curr_y)
        u, v = get_current(x_curr, y_curr, map_name, vectorize=True)
        ax1.quiver(x_curr, y_curr, u, v, color='teal', width=0.0045)

    # plot goal
    if run_simulation == False and run_optimization == True:
        ax1.plot(target[0], target[1], 'Xg')
        
    # highlight start and end of trajectory
    ax1.plot(x[0], y[0], 'og') # start
    ax1.plot(x[-1], y[-1], 'Xr') # end
    
    # plot initial boat position
    boat_graphic = plot_boat(boat, x[0], y[0], th[0], ax1)
    
    # plot initial object positions
    obstacle_graphics = plot_obstacles(objects, n=0, ax=ax1)
    
    # plot position trajectory
    ax1.plot(x, y, label='Optimal Trajectory')

    # labels and stylistic attributes min_x - padding_x, max_x + padding_x
    style_plot(ax1, r'$x(t)$', r'$y(t)$', x, y, plot_limits, 'simulation')
    style_plot(ax2, r'$Time$', r'$Normalized$', x, y, plot_limits, 'metrics')

    return fig1, ax1, fig2, ax2, [boat_graphic, obstacle_graphics]

def animate_env(x, y, th, x_dot, y_dot, th_dot, metric_times, boat, objects, path_rrt, frame_skip=50):
    '''
    Render a frame by frame animation, and save all figures if specified
    in global parameters.
    '''
    fig1, ax1, fig2, ax2, graphics = static_plot(x, y, th, x_dot, y_dot, th_dot, metric_times, boat, objects)

    # Add RRT path
    if path_rrt is not None:
        ax1.plot(path_rrt[0], path_rrt[1], '-r', label='RRT Guess')
        ax1.legend()
    
    if save_plots: fig1.savefig('{}/trajectory.png'.format(logdir))
        
    def animate(step):

        boat_graphic, object_graphics = graphics
        
        # get boat position
        x_current = x[step]
        y_current = y[step]
        heading = th[step]

        # update boat hull
        new_points = [rotate(heading, point) + [x_current, y_current] for point in boat[1:]]
        new_center = rotate(heading, boat[0]) + [x_current, y_current]
        boat_graphic[0].set_xy(new_points)
        boat_graphic[1].set_center(new_center)

        # update Object Positions
        logger_steps = len(x)
        for i, obstacle in enumerate(objects):
            # Adjust step as Drake Simulation has more steps than the trajectory planning
            obs_x, obs_y = obstacle.traj[step*obstacle.traj.shape[0]//logger_steps]
            object_graphics[i].set_center((obs_x, obs_y))
        
        return [boat_graphic[0], boat_graphic[1]] + object_graphics # list of patches to redraw
    
    video = animation.FuncAnimation(fig1, animate, frames=np.arange(0, len(x), frame_skip), interval=200, blit=True)
    if save_plots: video.save('{}/trajectory_video.mp4'.format(logdir)) # save the video

