
import numpy as np


MAPS = [
    'no_current',            # 0   
    'downstream',            # 1
    'upstream',              # 2
    'diagonal_downstream',   # 3
    'diagonal_upstream',     # 4
    'leftside_stream',       # 5
    'rightside_stream',      # 6
    'WHIRLPOOL!',            # 7
    'hyperbola',             # 8
    'radial_outwards',       # 9
    'vertical_trench',       # 10
    'horizontal_trench',     # 11
    'circular_orbit'         # 12
]


# ------------------------------------------------ SIM PARAMS ------------------------------------------------ #

map_name = MAPS[0]                 # map setting
duration = 150                     # timesteps (must be bigger than 30 - make this long enough to feasibily traverse your desired trajectory)
time_interval = 0.5                # time between steps [sec]
run_optimization = True            # trajectory optimization toggle
run_simulation = False             # simulated dynamics toggle

start = [5,                        # x
         5,                        # y
         np.pi/4,                  # th (always use np.pi never -np.pi)
         0,                        # x_dot
         0,                        # y_dot
         0]                        # th_dot

target =[-30,                      # x
         -30,                      # y
         0,                        # th (always use np.pi never -np.pi)
         0,                        # x_dot
         0,                        # y_dot
         0]                        # th_Dot


# ------------------------------------------------ DYNAMICS PARAMS ------------------------------------------------ #

m = 10000                          # boat mass [kg]
l = 8.0                            # boat half length [m]
w = 5.0                            # boat width [m]
h = 1.0                            # boat submerged height [m]
rho_w = 1000.0                     # H2O density [kg/m^3]
d = 7                              # drag lever arm length [m]
C_front = 0.05                     # forward drag coeff of boat [unitless]
C_side = 0.4                       # sideways drag coeff of boat [unitless]

boat_points = np.array([[], [], ])
obstacle_params = np.array([       # define obstacles: [[x_start, y_start], [x_end, y_end], radius]
                            [[-10, -15], [-10, -15], 10],
                            [[0, -60], [0, -60], 10]
                           ], dtype=object)


# ------------------------------------------------ PLOT PARAMS ------------------------------------------------ #

save_plots = True                  # toggle to save all videos/plots to logdir
logdir = 'results/'                # results folder (all plots and videos)
pad_factor = 1.0                   # affects how much we pad plot margins around trajectories (ensures we're always centered/zoomed into trajectory)
num_curr_x = 10                    # number of current vectors plotted along x
num_curr_y = 10                    # number of current vectors plotted along y

# plotting toggles
metric_switches = [0,              # plot x(t)
                   0,              # plot y(t)
                   0,              # plot theta(t)
                   0,              # plot x_dot(t)
                   0,              # plot y_dot(t)
                   0,              # plot th_dot(t)
                   1,              # plot F_t (torque force)
                   1]              # plot thF_t (torque force heading)


# ------------------------------------------------ OPTIMIZATION PARAMS ------------------------------------------------ #

solver_type = "Ipopt"              # which solver to use: Snope of Ipopt
two_phase_optimization = False     # True: solve optimization without current first, then solve with current. False: solve directly with current
use_rrt = True                     # True: Use RRT to get initial path guess, False: Uses straightline path
rrt_iters = 500                  # More samples is a more optimal path, but takes longer
thrust_lim = [10000, np.pi/2.75]   # torque limits on force and rudder angle [newtons, radians]
eps = 0.1                          # smoothing factor, allows differentiability: abs(x) ~ sqrt(x*x + eps)

final_tolerances = [0.0,           # final x tolerance
                    0.0,           # final y tolerance
                    0.1,           # final th tolerance
                    0.1,           # final x_dot tolerance
                    0.1,           # final y_dot tolerance
                    0.1]           # final th_dot tolerance

dynamics_tolerances = [0.0,        # x_dot dynamics tolerance
                       0.0,        # y_dot dynamics tolerance
                       0.0,        # th_dot dynamics tolerance
                       0.1,        # x_ddot dynamics tolerance
                       0.1,        # y_ddot dynamics tolerance
                       0.1]        # th_ddot dynamics tolerance

opt_switches = [1,                 # toggle initial state constraint
                1,                 # toggle goal state constraint
                1,                 # toggle dynamics constraints
                1,                 # toggle thrust limit constraints
                1,                 # toggle obstacle constraints
                0,                 # toggle fuel cost (optional)
                0]                 # toggle distance cost (optional)



