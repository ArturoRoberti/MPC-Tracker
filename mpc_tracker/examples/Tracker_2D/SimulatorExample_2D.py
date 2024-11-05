from mpc_tracker.Tracker_2D.MPC_2DTracker import MPC_2DTracker
from mpc_tracker.Tracker_2D.MPC_2DSimulator import MPC_2DSimulator
from mpc_tracker.custom_helpers.helper_classes import Circle, Polygon

import numpy as np

if __name__ == '__main__':
    # Define static goal position (only necessary if "add_satellite_goal" is not used)
    x_goal = 10
    y_goal = 10

    # Define weights for the cost function
    position_weight = 10
    position_end_weight = 100
    angle_weight = 1
    angle_end_weight = 10
    omega_weight = 0
    omega_end_weight = 0
    velocity_weight = 0
    velocity_bound = 1
    force_weight = 0.1
    ds_weight = 100

    # Define control bounds
    force_bound = 1.5

    # Define goal satellite orbit parameters
    goal_satellite_orbitradius = 9
    goal_satellite_radius = 3
    omega_goal_satellite_orbit = 0.04           # Try reducing this (e.g. to 0.01) and see what happens
    psi0_goal_satellite_orbit = 0

    # Define necessary inputs (r_robot, ts, parameters)
    robot_radius = 0.1
    sample_time = 0.1
    params = {'m': 1}



    # Define the model. For the order of the function calls (if necessary), see the documentation of the MPC_2DTracker class
    ## Initialize the model
    model = MPC_2DTracker(r_robot=robot_radius, ts=sample_time, parameters=params)

    ## Define the states
    model.add_state('x', 'vx', reference=x_goal, weight_N=position_end_weight)
    model.add_state('y', 'vy', reference=y_goal, weight_N=position_end_weight)
    model.add_state('psi', 'dpsi', weight=angle_weight, weight_N=angle_end_weight)
    model.add_state('dpsi', 'Mpsi', weight=omega_weight, weight_N=omega_end_weight)
    model.add_state('vx', 'Fx/m', lower_bound=-velocity_bound, upper_bound=velocity_bound, weight=velocity_weight)
    model.add_state('vy', 'Fy/m', lower_bound=-velocity_bound, upper_bound=velocity_bound, weight=velocity_weight)

    ## Define the controls
    model.add_control('Mpsi', lower_bound=-force_bound, upper_bound=force_bound, weight=force_weight)
    model.add_control('Fx', lower_bound=-force_bound, upper_bound=force_bound, weight=force_weight)
    model.add_control('Fy', lower_bound=-force_bound, upper_bound=force_bound, weight=force_weight)

    ## Define all static obstacles
    model.add_static_obstacle(Circle(np.array([9, 9]), 1))
    model.add_static_obstacle(Circle(np.array([4, 3.6]), 1))
    model.add_static_obstacle(Circle(np.array([1.5, 3]), 1))
    model.add_static_obstacle(Circle(np.array([4, 2.1]), 1))
    model.add_static_obstacle(Polygon([[6 + 1.5*np.cos(theta), 10 + 1.5*np.sin(theta)] for theta in np.linspace(0, 2 * np.pi, 7)[:-1]]))
    model.add_static_obstacle(Polygon([[0, 1], [1, 0], [2, 1], [1, 2]]))

    ## Define all satellite obstacles
    model.add_satellite_obstacle(1, 1, 1, 2, 0.1, 0)
    model.add_satellite_obstacle(0, 0, goal_satellite_radius, goal_satellite_orbitradius, omega_goal_satellite_orbit, theta0_orbit=psi0_goal_satellite_orbit)

    ## Define a moving goal (overwrites the static goal)
    model.add_satellite_goal(0, 0, goal_satellite_orbitradius + goal_satellite_radius + 4*robot_radius*1.5, omega_goal_satellite_orbit, psi0_orbit=psi0_goal_satellite_orbit, w_pos=position_weight, wN_pos=position_end_weight, w_vel=velocity_weight, wN_vel=velocity_weight, w_angle=angle_weight, wN_angle=angle_end_weight)

    ## Compile the model (final step)
    model.set_horizon(horizon_length=5)     # Might get stuck at local optima for short horizon (e.g. increase horizon to 20 for static goal)



    # Run the simulation
    ## Define the simulator
    simulator = MPC_2DSimulator(model, max_steps=500, simulation_time_factor=10, maximize_plotwindows=True, output_timespecs=True)

    ## Run the simulation
    data = simulator.run_mpc()

    ## Plot the results
    simulator.plot_map(block=True)
    simulator.plot_data(block=True)