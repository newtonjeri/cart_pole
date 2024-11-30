![CART POLE GAZEBO IMAGE](/extra_resource/Screenshot%202024-11-27%20173413.png)

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

### Gazebo required

Here are some dependancies you will need.

```bash
sudo apt-get install gazebo
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
```


TODO: I might add an initializer where I have a force to one direction at first to have a large initial oscillation before starting the project or I might just start at an angle to save time.
