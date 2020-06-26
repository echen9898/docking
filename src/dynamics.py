
# general imports
import numpy as np

# drake imports
from pydrake.all import SymbolicVectorSystem, Variable, AutoDiffXd
import pydrake.symbolic as sym

# local imports
from settings import *


def drag_coeff_from_angle(angle, C_side, C_front):
    '''
    Takes a weighted average of the side drag coefficient and front/back drag
    coefficient. The weighting corresponds to angle = boat_heading for the
    y-direction coefficients, with the x-direction just being a 90 deg shift

    Currently assumes boat is symmetrical about both axes 
    '''
    method = np if isinstance(angle, AutoDiffXd) else sym # Check type for autodiff
    # X is the same as Y shifted by 90 degrees
    return C_front*method.cos(angle-np.pi/2)**2 + C_side*method.sin(angle-np.pi/2)**2, C_front*method.cos(angle)**2 + C_side*method.sin(angle)**2

def drag_area_from_angle(angle, A_side, A_front):
    '''
    Takes weighted average of boat areas across front and side using same
    convention as drag_coeff_from_angle
    '''
    method = np if isinstance(angle, AutoDiffXd) else sym # Check type for autodiff
    return A_front*method.cos(angle-np.pi/2)**2 + A_side*method.sin(angle-np.pi/2)**2, A_front*method.cos(angle)**2 + A_side*method.sin(angle)**2

def get_drag_params(th_h):
    '''
    From paper
    eps = 0.000001 (separate from optimization smoothing parameter)
    C_front = 0.02058*(l**2*w*h/(sqrt(x_dot**2 + y_dot**2)+eps))**(-1/8)
    '''
    A_side = 2*l*h
    A_front = w*h
    scale = 1.0 # current scaling
    C_x, C_y = drag_coeff_from_angle(th_h, C_side, C_front)
    A_x, A_y = drag_area_from_angle(th_h, A_side, A_front)

    return C_x*scale, C_y*scale, A_x, A_y

def get_current(x, y, name, vectorize=False, scale=1):
    '''
    Retrieve x and y current components at a specific coordinate. If vectorize is True,
    then assume x, y inputs are arrays and return a grid of currents along these coordinate
    ranges. Used for plotting the vector field.
    ''' 
    method = np if isinstance(x, AutoDiffXd) else sym # Check type for autodiff

    currents = {
        "downstream" : 
            [lambda x, y: 0, 
             lambda x, y: -1.0],
        "upstream" : 
            [lambda x, y: 0, 
             lambda x, y: 1.0],
        "leftside_stream" :
            [lambda x, y: 1.0,
             lambda x, y: 0],
        "rightside_stream" :
            [lambda x, y: -1.0,
             lambda x, y: 0],
        "diagonal_downstream" : 
            [lambda x, y: -5.0, 
             lambda x, y: -1.0],
        "diagonal_upstream" : 
            [lambda x, y: 1.0, 
             lambda x, y: 1.0],
        "no_current" :
            [lambda x, y: 0,
             lambda x, y: 0],
        "vertical_trench" :
            [lambda x, y: 0,
             lambda x, y: x],
        "horizontal_trench" :
            [lambda x, y: y,
             lambda x, y: 0],
        "circular_orbit" :
            [lambda x, y: x/((x**2 + y**2)**0.5+1),
             lambda x, y: -y/((x**2 + y**2)**0.5+1)],
        "hyperbola" :
            [lambda x, y: -y/((x**2 + y**2)**0.5+1),
             lambda x, y: x/((x**2 + y**2)**0.5+1)],
        "radial_outwards" :
            [lambda x, y: y/((x**2 + y**2)**0.5+1),
             lambda x, y: x/((x**2 + y**2)**0.5+1)],
        "radial_inwards" :
            [lambda x, y: -y/((x**2 + y**2)**0.5+1),
             lambda x, y: -x/((x**2 + y**2)**0.5+1)],
        "WHIRLPOOL!" :
            [lambda x, y: -y/((x**2 + y**2)**0.5+1),
             lambda x, y: (-x-y)/((x**2 + y**2)**0.5+1)],}

    if vectorize:
        x_curr = np.zeros((len(x), len(y)))
        y_curr = np.zeros((len(x), len(y)))
        for i in range(len(x)):
            for j in range(len(y)):
                xi = x[i]
                yj = y[j]
                x_curr[i][j] = currents[name][0](xi, yj)
                y_curr[i][j] = currents[name][1](xi, yj)
    else:
        x_curr = currents[name][0](x, y)
        y_curr = currents[name][1](x, y)
    
    return x_curr*scale, y_curr*scale
  
def generate_dynamics(state, torques, scale=1, env_name="no_current"):
    '''
    Generate continuous time dynamics: x_dot = f(x, u, t)
    '''
    method = np if isinstance(state[0], AutoDiffXd) else sym # Check type for autodiff
    
    F_t, th_t = torques
    x, y, th_h, x_dot, y_dot, th_dot = state
    x_dot_c, y_dot_c = get_current(x, y, env_name, scale=scale)

    # Drag coefficient is a function of heading
    C_drag_x, C_drag_y, A_drag_x, A_drag_y = get_drag_params(th_h)
    
    # F = coefficients*signed v^2
    F_drag_x = 0.5*rho_w*C_drag_x*A_drag_x*(x_dot_c - x_dot)*method.sqrt((x_dot_c - x_dot)*(x_dot_c - x_dot) + eps)
    F_drag_y = 0.5*rho_w*C_drag_y*A_drag_y*(y_dot_c - y_dot)*method.sqrt((y_dot_c - y_dot)*(y_dot_c - y_dot) + eps)

    # Drag from boat spinning (to stop from spinning infinitely fast)
    T_drag_omega = -0.25*rho_w*C_side*h*(l**4)*th_dot*method.sqrt(th_dot * th_dot + eps)/4
    I = m*((2*l)**2 + w**2) / 12
    
    x_ddot = (F_t*method.sin(th_h + th_t) + F_drag_x) / m
    y_ddot = (F_t*method.cos(th_h + th_t) + F_drag_y) / m
    th_ddot = (-F_t*l*method.sin(th_t) + (F_drag_x*method.cos(th_h) - F_drag_y*method.sin(th_h))*d + T_drag_omega)/I

    return np.array([x_dot, y_dot, th_dot, x_ddot, y_ddot, th_ddot])

# current variable
x_dot_c = Variable("x_dot_c")
y_dot_c = Variable("y_dot_c")

# state of the robot (standard cartesian)
x = Variable("x") # horizontal position
y = Variable("y") # vertical position
theta = Variable("theta") # angular position
x_dot = Variable("x_dot") # horizontal velocity
y_dot = Variable("y_dot") # vertical velocity
theta_dot = Variable("theta_dot") # angular velocity
state = np.array([x, y, theta, x_dot, y_dot, theta_dot])

# control torques
thrust = Variable("thrust") # thrust force
thrust_angle = Variable("thrust_angle") # thrust angle
torques = np.array([thrust, thrust_angle])