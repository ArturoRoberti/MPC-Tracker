# MPC-Tracker
Python toolbox to ease the use of MPC for tracking a goal with the presence of obstacles. Goal and obstacles may be moving according to known dynamics. With the use of the *do_mpc* library, both linear and nonlinear MPC is possible.

Currently (*this repo is still in its inception*), only *MPC_2DTracker* is implemented with an integrated simulator. To run it and look at some examples and its implementation, see *src/Tracker_2D/MPC_2DTracker.py*. In future releases, the simulator will be separate and the *MPC_2DTracker* can be used in connection with other (real) systems.

# Setup
Be sure to install `poetry` to your system using (for linux at least)
```
curl -sSL https://install.python-poetry.org | python3 -
```

Then run `poetry install` in the root repo directory. This creates a new virtual environment (or uses a currently running one) and installs the packages defined in `requirements.txt` to it. To access the virtual environment, running `poetry shell` may be necessary.

Should you require importing new packages, install them to the virtual environment and add them to the package using (all commands from a shell running in the poetry virtual environment):
```
pip install <package_name>
pip freeze > requirements.txt
poetry add $(cat requirements.txt)
poetry install
```

# ToDo:
- Upgrade *setup.py* to *pyproject.toml*
- Simulator separately
- 3D
- Test 2D with more complex dynamics (e.g. differential drive bot incl. non-holonomic constraints)
- (Robust MPC)