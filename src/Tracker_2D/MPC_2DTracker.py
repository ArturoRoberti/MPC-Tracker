import sys
import os

import do_mpc
from casadi import * # mainly sin, cos, SX
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from termcolor import colored
from typing import List, Dict, Tuple
import shapely

@dataclass
class MPC_2DTracker:
    '''
    Inputs:
    - r_robot (str): Radius of the robot
    - ts (float): Time step of the MPC

    Class for MPC tracking of a 2D robot with static or dynamic circular obstacles and a (possibly moving) goal position. Further, custom states (e.g. yaw angle) can also be set.
    Afterwards, all functions can (or must) be set in the following order:
    1) 'add_state()', 'add_control()', 'add_static_obstacle()': These can be called in an arbitrary order. For further details, see the respective function. 
    Note that the position and velocity states 'x', 'y', 'vx' and 'vy' must be added in order for the rest of the MPC to work.
    2) 'add_satellite_obstacle()', (OPTIONAL:)'add_difference_weight()', 'add_satellite_goal()': These can be called in an arbitrary order. For further details, see the respective function. Note that the 'add_difference_weight()' function is optional.
    3) 'set_horizon()': This function must be called after all states, controls, obstacles and varying references have been added.
    This function also compiles and finishes the MPC model (by e.g. defining the quadratic cost function and calling the internal setup() functions)
    4) 'simulate_mpc()': This function can be called after the horizon has been set. It simulates the MPC and plots the results. For further details, see the respective function.

    '''

    # Necessary initialization inputs
    r_robot: float = field()
    ts: float = field()

    # TODO(0): There are TODOs all over the code which are not yet implemented

    # TODO: (MAYBE) add bools to check order of functions called (and if no e.g. states are added) (also check that after add_obstacle(), no more states can be added - add description that says weight have to be added for each distance after)
    # TODO: Add 'add_moving_reference' function (inputs: betroffener state (e.g. 'x' oder 'y'), r0, DGL) - in it, add warning if reference (and weight!) already set
    # TODO: Add checks for invalid lower bound/upper bound limits (e.g. lower bound > upper bound oder e.g. bei control lower und upper bound = 0 oder bei state lower bound = upper bound)
    # TODO: (MAYBE) Move from custom obstacle classes to shapely objects (https://shapely.readthedocs.io/en/stable/manual.html#polygons)
    # TODO: use 'set_nl_cons' instead of new states for obstacle constraints

    # TODO(2): Add polygon obstacles (both static and rotating (around themselves and other points))
    # TODO(2): Add polygon approximation as circle
    
    # Helper classes
    class obstacle:
        '''
        Defines a static circular obstacle with center (x_center, y_center) and radius r.
        '''
        def __init__(self, x_center, y_center, r):
            self.x = x_center
            self.y = y_center
            self.r = r

    # class polyhedron_obstacle:
    #     '''
    #     Defines a static polyhedron obstacle with a numpy array. Each row represents the 3 coefficient A, B, C of a vertice's plane equation Ax + By + C = 0.
    #     '''
    #     def __init__(self, points: np.ndarray | tuple):
    #         self.coefficients = points

    #     def points_to_coefficients(self, points: np.ndarray | tuple):
    #         '''
    #         Converts a numpy array of points to the coefficients of the corresponding planes.
    #         '''
            


    # Helper functions
    def find_state_index(self, state_name: str):
        '''
        Finds the index of a state in the MPC model. Raises an error if that state does not exist.
        '''
        for name_ind, name in enumerate(self.model.x.keys()):
            if state_name == name:
                return name_ind
        raise ValueError(f"ERROR in 'find_state_index()': State '{name}' not found")
    
    def is_at_goal(self, pos_tol: float, vel_tol: float, angle_tol: float) -> bool: # TODO: Definetively needs to be corrected
        '''
        Checks if the robot is at the goal position (and angle).

        Inputs:
            - pos_tol (float):      Tolerance for the position of the robot
            - angle_tol (float):    Tolerance for the angle of the robot

        Outputs:
            - at_goal (bool):       True if the robot is at the goal position (and angle), False otherwise
        '''

        # Static goal
        if not (self.moving_goal[0] or self.moving_goal[1]):
            if np.linalg.norm(np.array([self.curr_x[self.indices['x']]-self.reference[self.indices['x']], self.curr_x[self.indices['y']]-self.reference[self.indices['y']]])) < pos_tol and np.linalg.norm(np.array([self.curr_x[self.indices['vx']]-self.reference[self.indices['vx']], self.curr_x[self.indices['vy']]-self.reference[self.indices['vy']]])) < vel_tol:
                if (self.indices['psi'] is not None and abs(self.curr_x[self.indices['psi']] - self.reference[self.indices['psi']]) < angle_tol) or (self.indices['psi'] is None):
                    return True
        # Moving goal, but non-moving reference angle
        elif self.moving_goal[0] and not self.moving_goal[1]:
            # print("Current goal position:", self.curr_x[self.indices['goal_x']], self.curr_x[self.indices['goal_y']])
            if np.linalg.norm(np.array([self.curr_x[self.indices['x']]-self.curr_x[self.indices['goal_x']], self.curr_x[self.indices['y']]-self.curr_x[self.indices['goal_y']]])) < pos_tol and np.linalg.norm(np.array([self.curr_x[self.indices['vx']]-self.curr_x[self.indices['goal_vx']], self.curr_x[self.indices['vy']]-self.curr_x[self.indices['goal_vy']]])) < vel_tol:
                if (self.indices['psi'] is not None and abs(self.curr_x[self.indices['psi']] - self.reference[self.indices['goal_psi']]) < angle_tol) or (self.indices['psi'] is None):
                    return True
        # TODO: Non-moving goal, moving reference angle
        # Moving goal, moving reference angle
        elif self.moving_goal[0] and self.moving_goal[1]:
            if np.linalg.norm(np.array([self.curr_x[self.indices['x']]-self.curr_x[self.indices['goal_x']], self.curr_x[self.indices['y']]-self.curr_x[self.indices['goal_y']]])) < pos_tol and np.linalg.norm(np.array([self.curr_x[self.indices['vx']]-self.curr_x[self.indices['goal_vx']], self.curr_x[self.indices['vy']]-self.curr_x[self.indices['goal_vy']]])) < vel_tol and abs(self.curr_x[self.indices['psi']] - self.curr_x[self.indices['psi']]) < angle_tol:
                return True
            
        return False

    # functions_called = {'add_first': False, 'add_second': False, 'set_param': False} # (Maybe TODO)

    states = []             # List of all states
    states_bounds = []      # List of all state bounds
    complex_bounds = {}     # List of all complex state bounds (e.g. for obstacles)
    EoM_str = []            # List of all equations of motion
    x0 = []                 # List of all initial values of the states
    reference = []          # List of all constant reference values of the states
    Q_diag = []             # List of all weights on states themselves (diagonal cost matrix elements)
    Q_off = {}              # Dict of all weights on differences between states (off-diagonal cost matrix elements)
    QN_diag = []            # List of all terminal weights on states themselves (diagonal cost matrix elements)
    QN_off = {}             # Dict of all terminal weights on differences between states (off-diagonal cost matrix elements)

    controls = []           # List of all controls
    controls_bounds = []    # List of all control bounds
    R_diag = []             # List of all weights on controls themselves (diagonal cost matrix elements)

    obstacles = []          # List of all obstacles
    sat_radii = []          # List of all radii of satellite obstacles - used for plotting
    do_not_plot_ind = []    # List of all indices of states which should not be plotted (obstacle states, goal state)

    moving_goal = [False, False] # [position (and velocity), angle] - Internal use
    indices = {'x': None, 'y': None, 'psi': None, 'vx': None, 'vy': None, 'dpsi': None, 'goal_x': None, 'goal_y': None, 'goal_psi': None, 'goal_vx': None, 'goal_vy': None} # Internal use

    model = do_mpc.model.Model('continuous')
    mpc  = None             # MPC controller
    simulator = None        # MPC simulator (for next step prediction)

    # During runtime/simulation
    curr_x = None           # Current states (list) of the robot, using during runtime/simulation

    def add_state(self, name: str, EoM: str, x0: float = 0, reference: float = 0, weight: float = 0, weight_N: float = 0, lower_bound: float = None, upper_bound: float = None):
        '''
        Adds a state to the current MPC controller. Can not be called after 'add_satellite_obstacle()' or later.

        Inputs:
            - name (str):           Name of the state
            - EoM (str):            Equation of motion of the state (e.g. 'vx' for state 'x', as dx/dt = vx)
            - x0 (float):           (OPTIONAL - default: 0) Initial value of the state
            - reference (float):    (OPTIONAL - default: 0) Reference value of the state
            - weight (float):       (OPTIONAL - default: 0) Weight of the state in the cost function
            - weight_N (float):     (OPTIONAL - default: 0) Terminal weight of the state in the cost function
            - lower_bound (float):  (OPTIONAL - default: None) Lower bound of the state
            - upper_bound (float):  (OPTIONAL - default: None) Upper bound of the state
        '''

        if name in self.model.x.keys():
            print(f"WARNING in 'add_state()': State '{name}' already exists. Skipping...")
            return
        
        state = self.model.set_variable(var_type='_x', var_name=name, shape=(1, 1))
        self.states.append(state)
        self.EoM_str.append(EoM)
        self.x0.append(x0)
        self.reference.append(reference)
        self.Q_diag.append(weight)
        self.QN_diag.append(weight_N)
        self.states_bounds.append([lower_bound, upper_bound])

        if name == 'x' or name == 'y' or name == 'vx' or name == 'vy' or name == 'psi' or name == 'dpsi':
            self.indices[name] = len(self.states) - 1

        globals()[name] = state

    def add_control(self, name: str, weight: float = 0, lower_bound: float = None, upper_bound: float = None):
        '''
        Adds a control to the current MPC controller. Can not be called after 'add_satellite_obstacle()' or later.

        Inputs:
            - name (str):           Name of the control
            - weight (float):       (OPTIONAL - default: 0) Weight of the control in the cost function
            - lower_bound (float):  (OPTIONAL - default: None) Lower bound of the control
            - upper_bound (float):  (OPTIONAL - default: None) Upper bound of the control
        '''

        if name in self.model.u.keys():
            print(f"WARNING in 'add_control()': Control '{name}' already exists. Skipping...")
            return
        control = self.model.set_variable(var_type='_u', var_name=name, shape=(1, 1))
        self.controls.append(control)
        self.R_diag.append(weight)
        self.controls_bounds.append([lower_bound, upper_bound])

        globals()[name] = control

    def add_static_obstacle(self, x_center: float, y_center: float, r: float, w: float = 0, wN: float = 0):
        '''
        Adds a static circular obstacle to the current MPC controller. Can not be called after 'add_satellite_obstacle()' or later.

        Inputs:
            - x_center (float):     x-position of the center of the obstacle
            - y_center (float):     y-position of the center of the obstacle
            - r (float):            Radius of the obstacle
            - w (float):            (OPTIONAL - default: 0) Weight of the distance to the obstacle in the cost function
            - wN (float):           (OPTIONAL - default: 0) Terminal weight of the distance to the obstacle in the cost function
        '''

        self.obstacles.append(self.obstacle(x_center, y_center, r))
        self.add_state(f'static_sqd{len(self.obstacles)}', f'2*(x-{x_center})*vx + 2*(y-{y_center})*vy', weight = w, weight_N = wN, x0 = (self.x0[self.indices['x']] - x_center)**2 + (self.x0[self.indices['y']] - y_center)**2, lower_bound = ((r + self.r_robot)*(1 + self.ts))**2) # Safety margin of 'ts' chosen somewhat arbitrarily
        self.do_not_plot_ind.append(len(self.states) - 1)

        # self.complex_bounds[]

    def add_satellite_obstacle(self, x_center: float, y_center: float, r_satellite: float, r_orbit: float, omega_orbit: float, theta0_orbit: float, w: float = 0, wN: float = 0):
        '''f
        Adds an obstacle going in a circular path to the current MPC controller. Can not be called after 'set_horizon()' or later.

        Inputs:
            - x_center (float):     x-position of the (imaginary) center of orbit
            - y_center (float):     y-position of the (imaginary) center of orbit
            - r_satellite (float):  Radius of the obstacle
            - r_orbit (float):      Radius of the circular orbit
            - omega_orbit (float):  Angular velocity of the circular orbit
            - theta0_orbit (float): Initial angle of the circular orbit
            - w (float):            (OPTIONAL - default: 0) Weight of the distance to the obstacle in the cost function
            - wN (float):           (OPTIONAL - default: 0) Terminal weight of the distance to the obstacle in the cost function
        '''

        if self.indices['x'] is None or self.indices['y'] is None or self.indices['vx'] is None or self.indices['vy'] is None:
            raise AssertionError("WARNING in 'add_satellite_obstacle()': States 'x', 'y', 'vx' and 'vy' must all be added before adding satellite obstacles. Skipping...")

        self.sat_radii.append(r_satellite)
        n_theta = len(self.sat_radii)
        self.add_state(f'theta{n_theta}', str(omega_orbit), x0 = theta0_orbit)

        sat_vx = f'-{r_orbit}*{omega_orbit}*sin(theta{n_theta})'
        sat_center_x0 = x_center + r_orbit*np.cos(theta0_orbit)
        self.add_state(f'sat_x{n_theta}', sat_vx, x0 = sat_center_x0)

        sat_vy = f'{r_orbit}*{omega_orbit}*cos(theta{n_theta})'
        sat_center_y0 = y_center + r_orbit*np.sin(theta0_orbit)
        self.add_state(f'sat_y{n_theta}', sat_vy, x0 = sat_center_y0)

        vd = f'2*(vx - {sat_vx})*(x - sat_x{n_theta}) + 2*(vy - {sat_vy})*(y - sat_y{n_theta})'
        d_x0 = (self.x0[self.indices['x']] - sat_center_x0)**2 + (self.x0[self.indices['y']] - sat_center_y0)**2
        self.add_state(f'sat_sqd{n_theta}', vd, weight = w, weight_N = wN, x0 = d_x0, lower_bound = ((r_satellite + self.r_robot)*(1 + self.ts))**2) # Safety margin of 'ts' chosen somewhat arbitrarily
        self.do_not_plot_ind.extend([len(self.states) - 1, len(self.states) - 2, len(self.states) - 3, len(self.states) - 4])

    def set_difference_weight(self, state_1: str, state_2: str, weight: float, weight_N: float): # TODO: Extend to inputs
        '''
        Sets the weight of the difference between two states in the cost function.
        This is done by setting both diagonal off-diagonal elements according to 'w*(a-b)**2 = w*a**2 - w*a*b  - w*b*a + w*b**2' in the cost matrices.
        Can not be called after 'set_horizon()' or later.

        Inputs:
            - state_1 (str):    Name of the first state
            - state_2 (str):    Name of the second state
            - weight (float):   Weight of the difference between the two states in the cost function
            - weight_N (float): Terminal weight of the difference between the two states in the cost function
        '''

        if state_1 == state_2:
            raise AssertionError(f"ERROR in 'set_difference_weight()': States '{state_1}' and '{state_2}' are not allowed to be the same")
        
        if len(self.Q_off) > 0 ^ len(self.QN_off) > 0: # '^' symbol represents XOR
            raise AssertionError("ERROR in 'set_difference_weight()': Either both Q_off and QN_off must be empty or both must be dicts of dicts - should not happen")

        state_1_ind = self.find_state_index(state_1)
        state_2_ind = self.find_state_index(state_2)

        if state_1_ind in self.Q_off:
            if state_2_ind in self.Q_off[state_1_ind] and self.Q_off[state_1_ind][state_2_ind] != 0:
                print(f"WARNING in 'set_difference_weight()': Weight for difference between '{state_1}' and '{state_2}' already set - it will be overwritten")
                
        if state_1_ind in self.QN_off:
            if state_2_ind in self.QN_off[state_1_ind] and self.QN_off[state_1_ind][state_2_ind] != 0:
                print(f"WARNING in 'set_difference_weight()': Terminal weight for difference between '{state_1}' and '{state_2}' already set - it will be overwritten")
        
        self.Q_off[state_1_ind] = {state_2_ind: -weight, state_1_ind: weight}
        self.Q_off[state_2_ind] = {state_1_ind: -weight, state_2_ind: weight}
        self.QN_off[state_1_ind] = {state_2_ind: -weight_N, state_1_ind: weight_N}
        self.QN_off[state_2_ind] = {state_1_ind: -weight_N, state_2_ind: weight_N}

    def add_moving_reference(): # TODO: make this general function for references which do not depend on other (possibly undefined) states
        pass
    
    def add_satellite_goal(self, x_center: float, y_center: float, r_orbit: float, omega_orbit: float, psi0_orbit: float, w_pos: float = 0, wN_pos: float = 0, w_vel: float = 0, wN_vel: float = 0, w_angle: float = 0, wN_angle: float = 0, w_angvel: float = 0, wN_angvel: float = 0):
        '''
        Adds a moving goal position and -angle to the current MPC controller. Can not be called after 'set_horizon()' or later.

        Inputs:
            - x_center (float):     x-position of the (imaginary) center of orbit
            - y_center (float):     y-position of the (imaginary) center of orbit
            - r_orbit (float):      Radius of the circular orbit
            - omega_orbit (float):  Angular velocity of the circular orbit
            - psi0_orbit (float):   Initial angle of the circular orbit
            - w (float):            (OPTIONAL - default: 0) Weight of the distance to the goal position in the cost function - if 0, then wN cannot be 0
            - wN (float):           (OPTIONAL - default: 0) Terminal weight of the distance to the goal position in the cost function - if 0, then w cannot be 0
        '''

        if self.indices['x'] is None or self.indices['y'] is None or self.indices['vx'] is None or self.indices['vy'] is None:
            raise AssertionError("ERROR in 'add_satellite_goal_position()': States 'x', 'y', 'vx' and 'vy' must all be added before adding a satellite goal position.")
        
        if self.indices['psi'] is None and (w_angle != 0 or wN_angle != 0):
            raise AssertionError("ERROR in 'add_satellite_goal_position()': Weights and terminals weight for angle are not 0, but no state 'psi' (robot yaw angle) has been added - no moving goal can be implemented")
        
        if self.indices['dpsi'] is None and (w_angvel != 0 or wN_angvel != 0):
            raise AssertionError("ERROR in 'add_satellite_goal_position()': Weights and terminals weight for omega are not 0, but no state 'dpsi' (robot yaw rate) has been added - no moving goal can be implemented")
        
        if self.indices['goal_x'] is not None or self.indices['goal_y'] is not None:
            print("WARNING in 'add_satellite_goal_position()': Moving goal position has already been set - the new position will be ignored")
            return
        
        if w_pos == 0 and wN_pos == 0 and w_angle == 0 and wN_angle == 0 and w_vel == 0 and wN_vel == 0:
            print("WARNING in 'add_satellite_goal_position()': Weights and terminals weight are all 0 - moving reference will not be added, as it is not used")
            return
        
        if w_pos != 0 or wN_pos != 0:
            self.moving_goal[0] = True
        if w_angle != 0 or wN_angle != 0:
            self.moving_goal[1] = True

        if self.Q_diag[self.indices['x']] != 0:
            print(f"WARNING in 'add_satellite_goal_position()': Previous weight for state 'x' is not 0 - weight will be overwritten.")
            self.Q_diag[self.indices['x']] = 0

        if self.QN_diag[self.indices['x']] != 0:
            print(f"WARNING in 'add_satellite_goal_position()': Previous terminal weight for state 'x' is not 0 - weight will be overwritten.")
            self.QN_diag[self.indices['x']] = 0

        if self.Q_diag[self.indices['y']] != 0:
            print(f"WARNING in 'add_satellite_goal_position()': Previous weight for state 'y' is not 0 - weight will be overwritten.")
            self.Q_diag[self.indices['y']] = 0

        if self.QN_diag[self.indices['y']] != 0:
            print(f"WARNING in 'add_satellite_goal_position()': Previous terminal weight for state 'y' is not 0 - weight will be overwritten.")
            self.QN_diag[self.indices['y']] = 0

        if self.Q_diag[self.indices['vx']] != 0:
            print(f"WARNING in 'add_satellite_goal_position()': Previous weight for state 'vx' is not 0 - weight will be overwritten.")
            self.Q_diag[self.indices['vx']] = 0
        
        if self.QN_diag[self.indices['vx']] != 0:
            print(f"WARNING in 'add_satellite_goal_position()': Previous terminal weight for state 'vx' is not 0 - weight will be overwritten.")
            self.QN_diag[self.indices['vx']] = 0

        if self.Q_diag[self.indices['vy']] != 0:
            print(f"WARNING in 'add_satellite_goal_position()': Previous weight for state 'vy' is not 0 - weight will be overwritten.")
            self.Q_diag[self.indices['vy']] = 0

        if self.QN_diag[self.indices['vy']] != 0:
            print(f"WARNING in 'add_satellite_goal_position()': Previous terminal weight for state 'vy' is not 0 - weight will be overwritten.")
            self.QN_diag[self.indices['vy']] = 0

        if self.indices['psi'] is not None:
            if self.Q_diag[self.indices['psi']] != 0:
                print(f"WARNING in 'add_satellite_goal_position()': Previous weight for state 'psi' is not 0 - weight will be overwritten.")
                self.Q_diag[self.indices['psi']] = 0

            if self.QN_diag[self.indices['psi']] != 0:
                print(f"WARNING in 'add_satellite_goal_position()': Previous terminal weight for state 'psi' is not 0 - weight will be overwritten.")
                self.QN_diag[self.indices['psi']] = 0

        if self.indices['dpsi'] is not None and (w_angvel != 0 or wN_angvel != 0):
            if self.Q_diag[self.indices['dpsi']] != 0 and w_angvel != 0:
                print(f"WARNING in 'add_satellite_goal_position()': Previous weight for state 'dpsi' is not 0 - weight will be overwritten.")
                self.Q_diag[self.indices['dpsi']] = 0

            if self.QN_diag[self.indices['dpsi']] != 0 and wN_angvel != 0:
                print(f"WARNING in 'add_satellite_goal_position()': Previous terminal weight for state 'dpsi' is not 0 - weight will be overwritten.")
                self.QN_diag[self.indices['dpsi']] = 0

            if self.reference[self.indices['dpsi']] != omega_orbit:
                print(f"WARNING in 'add_satellite_goal_position()': Reference value for state 'dpsi' is not equal to the angular velocity of the orbit - the reference value (currently {self.reference[self.indices['dpsi']]}) will be overwritten to {omega_orbit}")
            self.reference[self.indices['dpsi']] = omega_orbit


        self.add_state('goal_psi', str(omega_orbit), x0 = psi0_orbit) # TODO: Check if goal is not always "standing on" satellite, i.e. add constant angle here (or idk what to do)
        self.indices['goal_psi'] = len(self.states) - 1
        
        if self.indices['psi'] is None:
            if psi0_orbit != 0:
                raise AssertionError("ERROR in 'add_satellite_goal_position()': Initial angle of the orbit is not 0, but no state 'psi' (robot yaw angle) has been added, meaning the goal state may never be reached. Please either add a 'psi' state or set the initial angle to 0.")
            if self.indices['dpsi'] is None:
                print("WARNING in 'add_satellite_goal_position()': States 'psi' (robot yaw angle) and 'dpsi' (yaw rate) have not been added - the robot will be modeled as a non-rotating point mass")
            else:
                print("WARNING in 'add_satellite_goal_position()': State 'psi' (robot yaw angle) has not been added, but 'dpsi' has. In this case, this class assumes that 'dpsi' is not the robot's yaw rate, but some other angular velocity. If this was not intended, then please add the state 'psi' and set 'dpsi' to be the yaw rate.")
        else:
            self.set_difference_weight('psi', 'goal_psi', weight = w_angle, weight_N = wN_angle)

        # goal_vx = f'-{r_orbit}*{omega_orbit}*sin(goal_psi)'
        # goal_center_x0 = x_center + r_orbit*np.cos(psi0_orbit)
        # self.add_state('goal_x', goal_vx, x0 = goal_center_x0)
        # self.indices['goal_x'] = len(self.states) - 1
        # self.set_difference_weight('x', 'goal_x', weight = w_pos, weight_N = wN_pos)

        # # TODO: Instead of this, one has to add a state 'goal_vx' and 'goal_vy' and set the difference weight between 'vx' and 'goal_vx' and between 'vy' and 'goal_vy'
        # if not np.isclose(self.reference[self.indices['vx']], r_orbit*omega_orbit):
        #     print(f"WARNING in 'add_satellite_goal_position()': Reference velocity 'vx' is not equal to the velocity of the orbit - the reference velocity (currently {self.reference[self.indices['vx']]}) will be overwritten to {r_orbit*omega_orbit}")
        # self.reference[self.indices['vx']] = r_orbit*omega_orbit

        # self.add_state('goal_vx', goal_vx, x0 = goal_center_x0)

        # goal_vy = f'{r_orbit}*{omega_orbit}*cos(goal_psi)'
        # goal_center_y0 = y_center + r_orbit*np.sin(psi0_orbit)
        # self.add_state('goal_y', goal_vy, x0 = goal_center_y0)
        # self.indices['goal_y'] = len(self.states) - 1
        # self.set_difference_weight('y', 'goal_y', weight = w_pos, weight_N = wN_pos)

        # # TODO: Instead of this, one has to add a state 'goal_vx' and 'goal_vy' and set the difference weight between 'vx' and 'goal_vx' and between 'vy' and 'goal_vy'
        # if not np.isclose(self.reference[self.indices['vy']], r_orbit*omega_orbit):
        #     print(f"WARNING in 'add_satellite_goal_position()': Reference velocity 'vy' is not equal to the velocity of the orbit - the reference velocity (currently {self.reference[self.indices['vy']]}) will be overwritten to {r_orbit*omega_orbit}")
        # self.reference[self.indices['vy']] = r_orbit*omega_orbit
            
        v0 = r_orbit*omega_orbit
        vx0 = -v0*np.sin(psi0_orbit)
        vy0 = v0*np.cos(psi0_orbit)

        self.add_state('goal_x', 'goal_vx', x0 = x_center + r_orbit*np.cos(psi0_orbit))
        self.indices['goal_x'] = len(self.states) - 1
        self.set_difference_weight('x', 'goal_x', weight = w_pos, weight_N = wN_pos)
        self.add_state('goal_vx', f'-{r_orbit}*{omega_orbit**2}*cos(goal_psi)', x0 = vx0)
        self.indices['goal_vx'] = len(self.states) - 1
        self.set_difference_weight('vx', 'goal_vx', weight = w_vel, weight_N = wN_vel)

        self.add_state('goal_y', 'goal_vy', x0 = y_center + r_orbit*np.sin(psi0_orbit))
        self.indices['goal_y'] = len(self.states) - 1
        self.set_difference_weight('y', 'goal_y', weight = w_pos, weight_N = wN_pos)
        self.add_state('goal_vy', f'-{r_orbit}*{omega_orbit**2}*sin(goal_psi)', x0 = vy0)
        self.indices['goal_vy'] = len(self.states) - 1
        self.set_difference_weight('vy', 'goal_vy', weight = w_vel, weight_N = wN_vel)

        self.do_not_plot_ind.extend([len(self.states) - 1, len(self.states) - 2, len(self.states) - 3, len(self.states) - 4, len(self.states) - 5])

    def set_horizon(self, horizon_length: int):
        '''
        Sets the horizon length of the MPC and compiles the model. This function can only be called after all states, controls, obstacles and varying references have been added.


        Inputs:
            - horizon_length (int): Length of the horizon
        '''

        if horizon_length <= 0 or not isinstance(horizon_length, int):
            raise AssertionError("ERROR in 'set_horizon()': 'horizon_length' must be an integer greater than 0")
        
        def model_end_setup():
            '''
            Finishes the first part of MPC model by setting the equations of motion and calling an internal setup() function.
            '''

            if len(self.obstacles) > 0 and not ('x' in self.model.x.keys() and 'y' in self.model.x.keys() and 'vx' in self.model.x.keys() and 'vy' in self.model.x.keys()):
                raise AssertionError("ERROR in 'end_setup()': 'x', 'y', 'vx' and 'vy' must be set as states if obstacles are present")

            for i in range(len(self.states)):
                try:
                    expr = eval(self.EoM_str[i])
                except:
                    raise ValueError(f"ERROR in 'end_setup()': EoM string '{self.EoM_str[i]}' not valid - see if e.g. all constants, states and inputs in the EoM are defined")
                
                if isinstance(expr, float) or isinstance(expr, int):
                    expr = SX(expr)
                self.model.set_rhs(self.model.x.keys()[i], expr)

            self.model.setup()

        model_end_setup()

        # Set sampling time and horizon length
        self.mpc = do_mpc.controller.MPC(self.model)
        self.mpc.set_param(t_step=self.ts, n_horizon=horizon_length)

        def mpc_end_setup():
            '''
            Finishes the second part of MPC model by setting the cost function and bounds and calling an internal setup() function.
            '''
            # Set quadratic objective
            Q = np.diag(self.Q_diag)
            R = np.diag(self.R_diag)
            QN = np.diag(self.QN_diag)

            if len(self.Q_off) == 0 and len(self.QN_off) == 0:
                n_states = len(self.states)
                Q_off = np.zeros((n_states, n_states))
                QN_off = np.zeros((n_states, n_states))
            elif len(self.Q_off) == 0 or len(self.QN_off) == 0:
                raise ValueError("ERROR in 'end_setup()': Either both Q_off and QN_off must be empty or both must be dicts of dicts - should not happen")
            else:
                Q_off = np.zeros((len(self.states), len(self.states)))
                QN_off = np.zeros((len(self.states), len(self.states)))
                for i in range(len(self.states)):
                    if i in self.Q_off:
                        for j in self.Q_off[i]:
                            Q_off[i, j] = self.Q_off[i][j]
                    if i in self.QN_off:
                        for j in self.QN_off[i]:
                            QN_off[i, j] = self.QN_off[i][j]
            
            def weighted_norm(vec: np.ndarray, mat: np.ndarray):
                return np.dot(vec.T, np.dot(mat, vec)).item()
            st = np.array(self.states).reshape(-1, 1)
            ct = np.array(self.controls).reshape(-1, 1)
            ref = np.array(self.reference).reshape(-1, 1)
            lterm = weighted_norm(st - ref, Q) + weighted_norm(st, Q_off) + weighted_norm(ct, R)
            mterm = weighted_norm(st - ref, QN) + weighted_norm(st, QN_off)
            self.mpc.set_objective(lterm=lterm, mterm=mterm)

            # Set bounds
            xkeys = self.model.x.keys()
            for bound_ind, bound in enumerate(self.states_bounds):
                if bound[0] is not None:
                    self.mpc.bounds['lower', '_x', xkeys[bound_ind]] = bound[0]
                if bound[1] is not None:
                    self.mpc.bounds['upper', '_x', xkeys[bound_ind]] = bound[1]

            if 'default' in self.model.u.keys():
                ukeys = [i for i in self.model.u.keys() if i != 'default']
            else:
                ukeys = self.model.u.keys()
            for bound_ind, bound in enumerate(self.controls_bounds):
                if bound[0] is not None:
                    self.mpc.bounds['lower', '_u', ukeys[bound_ind]] = bound[0]
                if bound[1] is not None:
                    self.mpc.bounds['upper', '_u', ukeys[bound_ind]] = bound[1]

            self.mpc.setup()

        mpc_end_setup()

        # Set initial values
        self.mpc.set_initial_guess()
        self.simulator = do_mpc.simulator.Simulator(self.model)
        self.simulator.set_param(t_step=self.ts)
        self.simulator.setup()
        self.simulator.x0 = np.array(self.x0)
        self.curr_x = self.simulator.make_step(np.zeros((len(self.controls), 1))).flatten() # Initial prediction
    
    def next_step(self, x0: dict, pos_tol: float = 0.01, vel_tol:float = 0.01, angle_tol: float = 0.01, suppress_output: bool = True) -> Tuple[np.ndarray, np.ndarray]: # or None
        '''
        Calculates the next step of the MPC and returns the control input. This function can only be called after the model has been compiled (by setting its horizon length).

        Inputs:
            - x0 (dict):        Dictionary of the current state values (keys are the names of the states (as str), values are the values of the states (as float)).
                                All states not given in the dictionary will be set to the prediction of the mpc itself.

        Outputs (If None is returned, the goal has been reached. Otherwise, the following is returned):
            - u0 (np.ndarray):    Control input of the MPC.
            - x0 (np.ndarray):    Predicted state values of the MPC after the next step.
        '''

        for state_str, state_val in x0.items():
            if state_str in self.model.x.keys():
                # if state_str not in self.indices:
                #     print(f"WARNING in 'next_step()': State '{state_str}' should solely be used internally - your value will be ignored") # TODO: Maybe change?
                #     continue

                if isinstance(state_val, np.ndarray):
                    state_val = state_val.item()
                
                if not (isinstance(state_val, float) or isinstance(state_val, int)):
                    raise AssertionError(f"ERROR in 'next_step()': Value for state '{state_str}' is neither an int nor a float - please give valid state values")
                else:
                    self.curr_x[self.find_state_index(state_str)] = state_val
            else:
                raise AssertionError(f"ERROR in 'next_step()': State '{state_str}' not defined - please give valid state names")

        if self.is_at_goal(pos_tol, vel_tol, angle_tol):
            return None
        else:
            if suppress_output:
                original_stdout = sys.stdout
                sys.stdout = open(os.devnull, 'w')

            u0 = self.mpc.make_step(self.curr_x)            
            self.curr_x = self.simulator.make_step(u0).flatten()

            if suppress_output:
                sys.stdout = original_stdout

            return (u0, self.curr_x)
 
    def simulate_mpc(self, max_steps: float = 1000, plot_data: bool = False, plot_map: bool = True, noise: tuple = (0, 0, 0), xlim = None, ylim = None, time_factor: float = 1, max_size: bool = False, suppress_mpc_output: bool = True):
        '''
        Simulates the MPC and plots the results. This function can only be called after the model has been compiled (by setting its horizon length).

        Inputs:
            - max_steps (int):      (OPTIONAL - default: 1000) Maximum number of MPC steps to simulate
            - plot_data (bool):     (OPTIONAL - default: False) If True, the data of each state and input will be plotted separately
            - plot_map (bool):      (OPTIONAL - default: True) If True, the robot position (x, y) as well as all obstacles and the goal will be animated over time
            - noise (tuple):        (OPTIONAL - default: (0, 0, 0)) Tuple of three floats (mean, std, max) for the gaussian noise added to the x and y position # TODO: add noise to controls as well # TODO in general
            - xlim (tuple):         (OPTIONAL - default: None) Tuple of two floats (xmin, xmax) for the x-axis limits of the map plot
            - ylim (tuple):         (OPTIONAL - default: None) Tuple of two floats (ymin, ymax) for the y-axis limits of the map plot
            - time_factor (float):  (OPTIONAL - default: 1) Factor by which the time between each MPC step is multiplied for the animation
            - max_size (bool):      (OPTIONAL - default: False) If True, the map plot will be maximized
            - mpc_output (bool):    (OPTIONAL - default: False) If True, the output of the MPC will be printed in detail in each time step
        '''

        if max_steps == 0:
            print("WARNING in 'simulate_mpc()': 'max_steps' is 0 - MPC will not be simulated")
            return
        if max_steps == 1:
            print("WARNING in 'simulate_mpc()': 'max_steps' is 1 - everything stays at its initial position")

        sim_graphics = do_mpc.graphics.Graphics(self.simulator.data)
        if plot_data:
            xplot = [self.model.x.keys()[i] for i in range(len(self.model.x.keys())) if i not in self.do_not_plot_ind]
            if 'default' in self.model.u.keys():
                names = np.concatenate([xplot, [i for i in self.model.u.keys() if i != 'default']], axis=-1)
            else:
                names = np.concatenate([xplot, self.model.u.keys()], axis=-1)
            xlen = len(self.model.x.keys()) - len(self.do_not_plot_ind)
            fig, ax = plt.subplots(len(names), sharex=True)
            for name_ind, name in enumerate(names):
                ax[name_ind].set_ylabel(name)
                if name_ind < xlen:
                    sim_graphics.add_line(var_type='_x', var_name=name, axis=ax[name_ind])
                else:
                    sim_graphics.add_line(var_type='_u', var_name=name, axis=ax[name_ind])

        pos_tol = 0.01 # TODO: Add (default) input instead
        vel_tol = 0.01 # TODO: Add (default) input instead
        angle_tol = 0.01 # TODO: Add (default) input instead
        for i in range(max_steps):
            x0_dict = {}
            # print("Current x: " + str(self.curr_x))
            x0_dict = {'x': self.curr_x[self.indices['x']], 'y': self.curr_x[self.indices['y']], 'vx': self.curr_x[self.indices['vx']], 'vy': self.curr_x[self.indices['vy']]}
            x0u0 = self.next_step(x0_dict, pos_tol = pos_tol, vel_tol = vel_tol, angle_tol = angle_tol, suppress_output = suppress_mpc_output)
            if x0u0 is None:
                break
            else:
                # u0 = x0u0[0]
                # x0 = x0u0[1]
                # self.curr_x = x0
                # print("Current derivative:\t", x0u0[1][self.find_state_index('der_ds')])
                pass

        if i == 0 and max_steps > 1:
            print("The robot is already at the goal position")
        else:
            print("Necessary time steps: " + str(i+1))
            if plot_data:
                sim_graphics.plot_results()
                sim_graphics.reset_axes()
            if plot_map:
                fig, ax = plt.subplots()
                ax.set_xlabel('x-Position [m]')
                ax.set_ylabel('y-Position [m]')
                if xlim is not None:
                    ax.set_xlim(xlim)
                if ylim is not None:
                    ax.set_ylim(ylim)
                ax.set_aspect('equal')

                if max_size:
                    figManager = plt.get_current_fig_manager()
                    figManager.window.showMaximized()

                # Goal position
                if self.indices['goal_x'] is not None and self.indices['goal_y'] is not None:
                    add_plots = 2
                elif self.indices['goal_x'] is not None or self.indices['goal_y'] is not None:
                    raise AssertionError("ERROR in 'simulate_mpc()': Either both or none of the moving reference states must be set - should not happen")
                else:
                    ax.plot(self.reference[self.indices['x']], self.reference[self.indices['y']], 'go')
                    add_plots = 1

                # Static obstacles
                theta = np.linspace(0, 2*np.pi, 100)
                for obs in self.obstacles:
                    x_obs_plot = obs.x + obs.r*np.cos(theta)
                    y_obs_plot = obs.y + obs.r*np.sin(theta)
                    ax.plot(x_obs_plot, y_obs_plot, 'k')

                moving_plot = [ax.plot([], [], markersize=5)[0] for _ in range(len(self.sat_radii)+add_plots)]
                
                # Animate map: robot position over time, satellite (as circle) positions over time # TODO: Modify when adding goal posiiton over time
                time = np.arange(0, len(self.simulator.data['_x', 'x'])*self.ts, self.ts)
                for i in range(len(time)):
                    ax.plot(self.simulator.data['_x', 'x'][i], self.simulator.data['_x', 'y'][i],  'b+', markersize=5)
                    x_robot = self.simulator.data['_x', 'x'][i] + self.r_robot*np.cos(theta)
                    y_robot = self.simulator.data['_x', 'y'][i] + self.r_robot*np.sin(theta)
                    moving_plot[0].set_data(x_robot, y_robot)

                    if add_plots == 2:
                        x_goal = self.simulator.data['_x', 'goal_x'][i] + self.r_robot*np.cos(theta)
                        y_goal = self.simulator.data['_x', 'goal_y'][i] + self.r_robot*np.sin(theta)
                        moving_plot[1].set_data(x_goal, y_goal)

                    for sat_ind, sat_rad in enumerate(self.sat_radii):
                        x_obs_plot = self.simulator.data['_x', f'sat_x{sat_ind+1}'][i] + sat_rad*np.cos(theta)
                        y_obs_plot = self.simulator.data['_x', f'sat_y{sat_ind+1}'][i] + sat_rad*np.sin(theta)
                        moving_plot[sat_ind + add_plots].set_data(x_obs_plot, y_obs_plot)
                    plt.pause(self.ts/time_factor)
                
                plt.show()
               
if __name__ == "__main__":
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
    omega_goal_satellite_orbit = 0.05
    psi0_goal_satellite_orbit = 0

    model = MPC_2DTracker(r_robot=r_robot, ts=0.1)
    model.add_state('x', 'vx', reference=x_goal, weight_N=position_end_weight)# , lower_bound=-2, upper_bound=12)
    model.add_state('y', 'vy', reference=y_goal, weight_N=position_end_weight)#, lower_bound=-2, upper_bound=12)
    model.add_state('psi', 'dpsi', weight=angle_weight, weight_N=angle_end_weight)
    model.add_state('dpsi', 'Mpsi', weight=omega_weight, weight_N=omega_end_weight)
    model.add_control('Mpsi', lower_bound=-force_bound, upper_bound=force_bound, weight=force_weight)
    m = 1
    model.add_state('vx', 'Fx/m', lower_bound=-velocity_bound, upper_bound=velocity_bound, weight=velocity_weight)
    model.add_state('vy', 'Fy/m', lower_bound=-velocity_bound, upper_bound=velocity_bound, weight=velocity_weight)
    # model.add_state('der_ds', f'(x-{x_goal})*Fx/m + vx**2 + vy**2 + (y-{y_goal})*Fy/m', upper_bound=0.1, weight=ds_weight)
    # model.add_state('der_ds', f'2*((x - {x_goal})*vx + (y - {y_goal})*vy)', x0 = x_goal, upper_bound=x_goal + 0.01, weight=ds_weight)
    model.add_control('Fx', lower_bound=-force_bound, upper_bound=force_bound, weight=force_weight)
    model.add_control('Fy', lower_bound=-force_bound, upper_bound=force_bound, weight=force_weight)
    model.add_static_obstacle(9, 9, 1)
    model.add_static_obstacle(4, 3.6, 1)
    model.add_static_obstacle(1.5, 3, 1)
    model.add_static_obstacle(4, 2.1, 1)
    model.add_satellite_obstacle(1, 1, 1, 2, 0.1, 0)
    model.add_satellite_obstacle(0, 0, goal_satellite_radius, goal_satellite_orbitradius, omega_goal_satellite_orbit, theta0_orbit=psi0_goal_satellite_orbit)
    # model.add_satellite_goal(0, 0, goal_satellite_orbitradius + goal_satellite_radius + 4*r_robot*1.5, omega_goal_satellite_orbit, psi0_orbit=psi0_goal_satellite_orbit, w_angle=angle_weight, wN_angle=angle_end_weight)
    # model.add_satellite_goal(0, 0, goal_satellite_orbitradius + goal_satellite_radius + 4*r_robot*1.5, omega_goal_satellite_orbit, psi0_orbit=psi0_goal_satellite_orbit, w_pos=position_weight, wN_pos=position_end_weight, w_angle=angle_weight, wN_angle=angle_end_weight)
    # model.add_satellite_goal(0, 0, goal_satellite_orbitradius + goal_satellite_radius + 4*r_robot*1.5, omega_goal_satellite_orbit, psi0_orbit=psi0_goal_satellite_orbit, w_pos=position_weight, wN_pos=position_end_weight, w_vel=velocity_weight, wN_vel=velocity_weight)
    model.add_satellite_goal(0, 0, goal_satellite_orbitradius + goal_satellite_radius + 4*r_robot*1.5, omega_goal_satellite_orbit, psi0_orbit=psi0_goal_satellite_orbit, w_pos=position_weight, wN_pos=position_end_weight, w_vel=velocity_weight, wN_vel=velocity_weight, w_angle=angle_weight, wN_angle=angle_end_weight)
    model.set_horizon(horizon_length=5)
    model.simulate_mpc(max_steps=500, plot_data=False, plot_map=True, time_factor=10, max_size=True)