The solution to running the it so that gazebo, the physics egnine works is including, add the following to my bashrc
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/kevo/ros_ws/src/ros_arm/src
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/kevo/arduino_bot/src
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/kevo/ws_moveit2/src
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/kevo/ros_ws/src/arduinobot_ws/src
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/kevo/ros_ws/src/ai_based_sorting_robot_arm/src
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/kevo/robotics_inc/under_actuated_ststems/cart_pole_ws/src/cart_pole
```

## HOW TO RUN THE PROJECT.
1. Go to the workspace and source the project;

```bash
cart_pole_ws
source install/setup.bash
```

2. Launch the simulated gazebo robot;

```bash
ros2 launch cart_pole_description gazebo_complete.launch.xml
```

3. The run the controller:

```bash
ros2 run cart_pole_lypunov_lqr controller
```

4. I can also have the following plotter:

```bash
ros2 run plotjuggler plotjuggler
```

TODO: I might add an initializer where I have a force to one direction at first to have a large initial oscillation before starting the project or, I might just start at an angle to save startup time.
