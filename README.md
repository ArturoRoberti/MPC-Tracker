# MPC-Tracker
Python toolbox to ease the use of MPC for tracking a goal with the presence of obstacles. Goal and obstacles may be moving according to known dynamics. With the use of the *do_mpc* library, both linear and nonlinear MPC is possible.

Currently (*this repo is still in its inception*), only *MPC_2DTracker* is implemented with an integrated simulator. To run it and look at some examples and its implementation, see *src/Tracker_2D/MPC_2DTracker.py*. In future releases, the simulator will be separate and the *MPC_2DTracker* can be used in connection with other (real) systems.

# Setup
To install all necessary dependencies and identify this as a package in your system, run `pip install -e .` from the package root directory. Packages within the *src* directory can then be imported as normal, e.g. `from Tracker_2D.MPC_2DTracker import MPC_2DTracker`.

# ToDo:
- Upgrade *setup.py* to *pyproject.toml*
- Simulator separately
- 3D
- Test 2D with more complex dynamics (e.g. differential drive bot incl. non-holonomic constraints)
- (Robust MPC)