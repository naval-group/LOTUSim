This use case is based on Katell Lagattu work: 



To do only the first time: 

```
python3 -m venv ~/venv_drl

source ~/venv_drl/bin/activate 

pip install -r controller/requirements.txt

```

Then: 

TERMINAL 1

```
lotusim --gui run  drl/poc_drl.world
```

TERMINAL 2

```
source /opt/ros/humble/setup.bash
ros2 launch ~/lotusim_ws/src/lotusim/launch/ros_bridge.launch.py
```

TERMINAL 3

```
source ~/venv_drl/bin/activate 

source /opt/ros/humble/setup.bash

python ~/lotusim_ws/src/lotusim/assets/worlds/drl/controller/lqr_controller_NGP.py
