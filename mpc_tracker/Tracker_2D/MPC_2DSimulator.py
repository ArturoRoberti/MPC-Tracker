from dataclasses import dataclass, field
from typing import List, Tuple, Dict
from termcolor import colored

import do_mpc
import numpy as np
import time

import matplotlib.pyplot as plt
import tkinter as tk

from mpc_tracker.Tracker_2D.MPC_2DTracker import MPC_2DTracker
from mpc_tracker.custom_helpers.helper_classes import Circle, Polygon

@dataclass
class MPC_2DSimulator:
    '''
    This class is used to simulate the 2D motion of the system using the MPC controller.
    Here, many options such as visualization, simulation speed, data logging, etc. can be set.
    The Simulator can only be used with a completed MPC_2DTracker object (i.e. after 'set_horizon' is called). # TODO: add check for this

    Inputs:
        - tracker (MPC_2DTracker):  MPC controller object
        - max_steps (int):          (OPTIONAL - default: 1000) Maximum number of MPC steps to simulate
        - plot_data (bool):         (OPTIONAL - default: False) If True, the data of each state and input will be plotted separately
        - plot_map (bool):          (OPTIONAL - default: True) If True, the robot position (x, y) as well as all obstacles and the goal will be animated over time
        - noise (tuple):            (OPTIONAL - default: (0, 0, 0)) Tuple of three floats (mean, std, max) for the gaussian noise added to the x and y position
                                    # TODO: add noise to controls as well # TODO in general
        - xlim (tuple):             (OPTIONAL - default: None) Tuple of two floats (xmin, xmax) for the x-axis limits of the map plot
        - ylim (tuple):             (OPTIONAL - default: None) Tuple of two floats (ymin, ymax) for the y-axis limits of the map plot
        - simulation_time_factor (float):      (OPTIONAL - default: 1) Factor by which the time between each MPC step is multiplied for the animation
        - max_plotwindows (bool):          (OPTIONAL - default: False) If True, the map plot will be maximized
        - mpc_output (bool):        (OPTIONAL - default: False) If True, the output of the MPC will be printed in detail in each time step
    '''

    # TODO: Do separate functions 1) Run_MPC (returns dict of states, inputs and time), 2) Plot_Data, 3) Plot_Map
    # TODO: Update docstring

    # Public attributes
    ## Necessary inputs
    tracker:                MPC_2DTracker   = field()

    ## Optional inputs
    max_steps:              float                   = field(default=1000)
    noise:                  Tuple                   = field(default=(0, 0, 0)) # TODO (also, implement as vectors/matrices)
    xlim:                   Tuple                   = field(default=None)
    ylim:                   Tuple                   = field(default=None)
    simulation_time_factor: float                   = field(default=1)
    maximize_plotwindows:   bool                    = field(default=False)
    suppress_dompc_output:  bool                    = field(default=True)
    output_timespecs:       bool                    = field(default=False)
    goal_pos_tol:           float                   = field(default=0.01)
    goal_vel_tol:           float                   = field(default=0.01)
    goal_angle_tol:         float                   = field(default=0.01)
    data:                   Dict[str, np.ndarray]   = field(init=False)         # Contains the states, inputs and time of the simulation (can be accessed after calling 'run_mpc()')

    # Private attributes
    _mpc_ran:               bool                        = field(init=False, default=False)
    _deltatimevec:          List                        = field(init=False, default_factory=list)
    _sim_graphics:          do_mpc.graphics.Graphics    = field(init=False, default=None)

    def run_mpc(self) -> np.ndarray:
        '''
        This function runs the MPC simulation for the given number of steps.

        Inputs:
            - None

        Outputs:
            - data (np.ndarray):    Dictionary containing the states, inputs and time of the simulation. This can also be accessed via the 'data' attribute after calling this function.
        '''
        if self.max_steps == 0:
            print("WARNING in 'simulate_mpc()': 'max_steps' is 0 - MPC will not be simulated and 'None' will be returned")
            return None
        
        if self._mpc_ran:
            print(colored("WARNING in 'simulate_mpc()': MPC has already been simulated - running it again will overwrite the previous simulation", 'red'))
        else:
            self._mpc_ran = True

        for i in range(self.max_steps):
            x0_dict = {'x': self.tracker._curr_x[self.tracker._indices['x']], 'y': self.tracker._curr_x[self.tracker._indices['y']], 'vx': self.tracker._curr_x[self.tracker._indices['vx']], 'vy': self.tracker._curr_x[self.tracker._indices['vy']]}

            if self.output_timespecs:
                t0 = time.time()

            x0u0 = self.tracker.next_step(x0_dict, pos_tol = self.goal_pos_tol, vel_tol = self.goal_vel_tol, angle_tol = self.goal_angle_tol, suppress_output = self.suppress_dompc_output)
            
            if self.output_timespecs:
                t1 = time.time()
                self._deltatimevec.append(t1 - t0)
            
            if x0u0 is None:
                break

        if i == 0:
            print("The robot is already at the goal position")
        else:
            print("Necessary time steps: " + str(i+1))

        if self.output_timespecs:
            print("Average time per MPC step: \t\t\t" + str(np.mean(self._deltatimevec)))
            print("Standard deviation of time per MPC step: \t" + str(np.std(self._deltatimevec)))
            print("Maximal time per MPC step: \t\t\t" + str(np.max(self._deltatimevec)))
            print("Minimal time per MPC step: \t\t\t" + str(np.min(self._deltatimevec)))

        result_statedict = {}
        for key in self.tracker._indices.keys():
            result_statedict[key] = self.tracker._simulator.data['_x'][:, self.tracker._indices[key]]

        result_controldict = {}
        for ind, control in enumerate([i.name() for i in self.tracker._controls]):
            result_controldict[control] = self.tracker._simulator.data['_u'][:, ind]

        result_timedict = {'t': self.tracker._simulator.data['_time'].reshape((len(self.tracker._simulator.data['_time']), ))}

        self.data = result_statedict | result_controldict | result_timedict
        return self.data # TODO: Add possibility to also return distances to obstacles
        
    def plot_data(self, block: bool=False) -> None:
        '''
        This function plots the data of the simulation (states and inputs).

        Inputs:
            - block (bool): (OPTIONAL - default: False) If True, the plot will block the rest of the code until closed. Note that if 'block' is False, the plot will not be shown unless 'plt.show()' is called sometime after this function.

        Outputs:
            - None
        '''

        if not self._mpc_ran:
            print(colored("ERROR", "red") + " in 'plot_data()': MPC has not been simulated yet - call 'run_mpc()' first")
            return
        
        self._sim_graphics = do_mpc.graphics.Graphics(self.tracker._simulator.data)

        xplot = [self.tracker._model.x.keys()[i] for i in range(len(self.tracker._model.x.keys())) if i not in self.tracker._do_not_plot_ind]
        if 'default' in self.tracker._model.u.keys():
            names = np.concatenate([xplot, [i for i in self.tracker._model.u.keys() if i != 'default']], axis=-1)
        else:
            names = np.concatenate([xplot, self.tracker._model.u.keys()], axis=-1)
        xlen = len(self.tracker._model.x.keys()) - len(self.tracker._do_not_plot_ind)

        fig_data, ax_data = plt.subplots(len(names), sharex=True)
        for name_ind, name in enumerate(names):
            ax_data[name_ind].set_ylabel(name)
            if name_ind < xlen:
                self._sim_graphics.add_line(var_type='_x', var_name=name, axis=ax_data[name_ind])
            else:
                self._sim_graphics.add_line(var_type='_u', var_name=name, axis=ax_data[name_ind])

        if self.maximize_plotwindows:
            # Get the current figure manager
            figManager = plt.get_current_fig_manager()
            
            # Retrieve screen width and height using tkinter
            root = tk.Tk()
            root.withdraw()
            screen_width = root.winfo_screenwidth()
            screen_height = root.winfo_screenheight()

            # Resize the window to the screen size
            figManager.window.geometry(f"{screen_width}x{screen_height}+0+0")  # Set window size to screen size

            # Set window title
            fig_data.canvas.manager.set_window_title("Simulation Data")

        self._sim_graphics.plot_results()
        self._sim_graphics.reset_axes()
        
        if block:
            plt.show()
        else:
            plt.draw()
            plt.show(block=False)
            fig_data.canvas.flush_events()

    def plot_map(self, block: bool=False) -> None:
        '''
        This function plots an animation of the simulation on a 2D map.

        Inputs:
            - block (bool): (OPTIONAL - default: False) If True, the plot will block the rest of the code until closed. Note that if 'block' is False, the plot will not be shown unless 'plt.show()' is called sometime after this function.

        Outputs:
            - None
        '''

        if not self._mpc_ran:
            print(colored("ERROR", "red") + " in 'plot_map()': MPC has not been simulated yet - call 'run_mpc()' first")
            return
        
        fig_map, ax_map = plt.subplots()
        ax_map.set_xlabel('x-Position [m]')
        ax_map.set_ylabel('y-Position [m]')
        if self.xlim is not None:
            ax_map.set_xlim(self.xlim)
        if self.ylim is not None:
            ax_map.set_ylim(self.ylim)
        ax_map.set_aspect('equal')
        fig_map.canvas.manager.set_window_title("Simulation Map")

        if self.maximize_plotwindows:
            # Get the current figure manager
            figManager = plt.get_current_fig_manager()
            
            # Retrieve screen width and height using tkinter
            root = tk.Tk()
            root.withdraw()
            screen_width = root.winfo_screenwidth()
            screen_height = root.winfo_screenheight()

            # Resize the window to the screen size
            figManager.window.geometry(f"{screen_width}x{screen_height}+0+0")  # Set window size to screen size

        # Goal position
        if self.tracker._indices['goal_x'] is not None and self.tracker._indices['goal_y'] is not None:
            add_plots = 2
        elif self.tracker._indices['goal_x'] is not None or self.tracker._indices['goal_y'] is not None:
            raise AssertionError("ERROR in 'simulate_mpc()': Either both or none of the moving reference states must be set - should not happen")
        else:
            ax_map.plot(self.tracker._reference[self.tracker._indices['x']], self.tracker._reference[self.tracker._indices['y']], 'go')
            add_plots = 1

        # Static obstacles
        theta = np.linspace(0, 2*np.pi, 100)
        for obs in self.tracker._static_obstacles:
            # TODO: Differentiate between circles and poylgons
            if isinstance(obs, Circle):
                x_obs_plot = obs.center[0] + obs.radius*np.cos(theta)
                y_obs_plot = obs.center[1] + obs.radius*np.sin(theta)
                ax_map.plot(x_obs_plot, y_obs_plot, 'k')
            elif isinstance(obs, Polygon):
                x_obs_plot = [point[0] for point in (obs.points + obs.points[0:1])]
                y_obs_plot = [point[1] for point in (obs.points + obs.points[0:1])]
                # ax_map.fill(x_obs_plot, y_obs_plot, 'k')
                ax_map.plot(x_obs_plot, y_obs_plot, 'k')

        moving_plot = [ax_map.plot([], [], markersize=5)[0] for _ in range(len(self.tracker._sat_radii)+add_plots)]
        
        # Animate map: robot position over time, satellite (as circle) positions over time # TODO: Modify when adding goal posiiton over time
        timevec = np.arange(0, len(self.tracker._simulator.data['_x', 'x']), 1)
        for i in range(len(timevec)):
            ax_map.plot(self.tracker._simulator.data['_x', 'x'][i], self.tracker._simulator.data['_x', 'y'][i],  'b+', markersize=5)
            x_robot = self.tracker._simulator.data['_x', 'x'][i] + self.tracker.r_robot*np.cos(theta)
            y_robot = self.tracker._simulator.data['_x', 'y'][i] + self.tracker.r_robot*np.sin(theta)
            moving_plot[0].set_data(x_robot, y_robot)

            if add_plots == 2:
                x_goal = self.tracker._simulator.data['_x', 'goal_x'][i] + self.tracker.r_robot*np.cos(theta)
                y_goal = self.tracker._simulator.data['_x', 'goal_y'][i] + self.tracker.r_robot*np.sin(theta)
                moving_plot[1].set_data(x_goal, y_goal)

            for sat_ind, sat_rad in enumerate(self.tracker._sat_radii):
                x_obs_plot = self.tracker._simulator.data['_x', f'sat_x{sat_ind+1}'][i] + sat_rad*np.cos(theta)
                y_obs_plot = self.tracker._simulator.data['_x', f'sat_y{sat_ind+1}'][i] + sat_rad*np.sin(theta)
                moving_plot[sat_ind + add_plots].set_data(x_obs_plot, y_obs_plot)

            plt.draw()              # Draw the updated line
            plt.show(block=False)   # Show the plot
            fig_map.canvas.flush_events()
            time.sleep(self.tracker.ts/self.simulation_time_factor)
            if not plt.fignum_exists(fig_map.number):
                break

        if block:
            plt.show()
        else:
            plt.draw()
            plt.show(block=False)
            fig_map.canvas.flush_events()

if __name__ == '__main__':
    x_goal = 10
    y_goal = 10
    position_weight = 10
    position_end_weight = 100
    angle_weight = 1
    angle_end_weight = 10
    omega_weight = 0
    omega_end_weight = 0
    velocity_weight = 0
    velocity_bound = 1
    force_weight = 0.1
    # position_weight = 0
    # position_end_weight = 0
    # angle_weight = 0
    # angle_end_weight = 0
    # velocity_weight = 0
    # velocity_bound = 0
    # force_weight = 0.1
    ds_weight = 100
    force_bound = 1.5
    r_robot = 0.1
    goal_satellite_orbitradius = 9
    goal_satellite_radius = 3
    omega_goal_satellite_orbit = 0.04
    psi0_goal_satellite_orbit = 0

    params = {'m': 1}

    model = MPC_2DTracker(r_robot=r_robot, ts=0.1, parameters=params)
    model.add_state('x', 'vx', reference=x_goal, weight_N=position_end_weight)# , lower_bound=-2, upper_bound=12)
    model.add_state('y', 'vy', reference=y_goal, weight_N=position_end_weight)#, lower_bound=-2, upper_bound=12)
    model.add_state('psi', 'dpsi', weight=angle_weight, weight_N=angle_end_weight)
    model.add_state('dpsi', 'Mpsi', weight=omega_weight, weight_N=omega_end_weight)
    model.add_control('Mpsi', lower_bound=-force_bound, upper_bound=force_bound, weight=force_weight)
    # m = 1
    model.add_state('vx', 'Fx/m', lower_bound=-velocity_bound, upper_bound=velocity_bound, weight=velocity_weight)
    model.add_state('vy', 'Fy/m', lower_bound=-velocity_bound, upper_bound=velocity_bound, weight=velocity_weight)
    # model.add_state('der_ds', f'(x-{x_goal})*Fx/m + vx**2 + vy**2 + (y-{y_goal})*Fy/m', upper_bound=0.1, weight=ds_weight)
    # model.add_state('der_ds', f'2*((x - {x_goal})*vx + (y - {y_goal})*vy)', x0 = x_goal, upper_bound=x_goal + 0.01, weight=ds_weight)
    model.add_control('Fx', lower_bound=-force_bound, upper_bound=force_bound, weight=force_weight)
    model.add_control('Fy', lower_bound=-force_bound, upper_bound=force_bound, weight=force_weight)
    model.add_static_obstacle(Circle(np.array([9, 9]), 1))
    model.add_static_obstacle(Circle(np.array([4, 3.6]), 1))
    model.add_static_obstacle(Circle(np.array([1.5, 3]), 1))
    model.add_static_obstacle(Circle(np.array([4, 2.1]), 1))
    # model.add_static_obstacle(Polygon([[7, 10], [6, 11], [5, 10], [6, 9]]))
    # model.add_static_obstacle(Polygon([[7, 10], [6, 11], [5, 10], [6, 9]]))
    model.add_static_obstacle(Polygon([[6 + 1.5*np.cos(theta), 10 + 1.5*np.sin(theta)] for theta in np.linspace(0, 2 * np.pi, 7)[:-1]]))
    model.add_static_obstacle(Polygon([[0, 1], [1, 0], [2, 1], [1, 2]]))
    model.add_satellite_obstacle(1, 1, 1, 2, 0.1, 0)
    model.add_satellite_obstacle(0, 0, goal_satellite_radius, goal_satellite_orbitradius, omega_goal_satellite_orbit, theta0_orbit=psi0_goal_satellite_orbit)
    # model.add_satellite_goal(0, 0, goal_satellite_orbitradius + goal_satellite_radius + 4*r_robot*1.5, omega_goal_satellite_orbit, psi0_orbit=psi0_goal_satellite_orbit, w_angle=angle_weight, wN_angle=angle_end_weight)
    # model.add_satellite_goal(0, 0, goal_satellite_orbitradius + goal_satellite_radius + 4*r_robot*1.5, omega_goal_satellite_orbit, psi0_orbit=psi0_goal_satellite_orbit, w_pos=position_weight, wN_pos=position_end_weight, w_angle=angle_weight, wN_angle=angle_end_weight)
    # model.add_satellite_goal(0, 0, goal_satellite_orbitradius + goal_satellite_radius + 4*r_robot*1.5, omega_goal_satellite_orbit, psi0_orbit=psi0_goal_satellite_orbit, w_pos=position_weight, wN_pos=position_end_weight, w_vel=velocity_weight, wN_vel=velocity_weight)
    model.add_satellite_goal(0, 0, goal_satellite_orbitradius + goal_satellite_radius + 4*r_robot*1.5, omega_goal_satellite_orbit, psi0_orbit=psi0_goal_satellite_orbit, w_pos=position_weight, wN_pos=position_end_weight, w_vel=velocity_weight, wN_vel=velocity_weight, w_angle=angle_weight, wN_angle=angle_end_weight)
    model.set_horizon(horizon_length=5)

    # simulator = MPC_2DSimulator(model, max_steps=500, plot_data_bool=False, plot_map_bool=False, time_factor=10, max_size=True, output_time=True)
    simulator = MPC_2DSimulator(model, max_steps=500, simulation_time_factor=10, maximize_plotwindows=True, output_timespecs=True)
    data = simulator.run_mpc()
    simulator.plot_map(block=True)
    simulator.plot_data(block=True)