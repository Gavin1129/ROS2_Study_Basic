# Frame transformation exercise example: Turtle Follow
## Task: 
    generate two turtles in the window. One(turtle2) should follow another one(turtle1)

## Analysis:
    1. Determine the position of turtle1 relative to the position of turtle2 
    2. Set the position of turtle1 as the goal for turtle2 
    3. Control the turtle2 move to the goal
        3.1 Broadcast the position of each turtle in world frame
        3.2 Use a listener to find the positon of turtle1 relative to turtle2
## Steps:
    1. Write a program to call /spawn service to generate a new turtle(turlte2): exer01_spawn.cpp

    2. Write a node for frame transformation broadcasting, publush the location of each turtle in world frame: exer02_rf_broadcaster.cpp

    3. Write a listener, get the location of turtle1 relative to turtle2, then generate a controller to control the motion of turtle2. exer03_tf_listener.cpp

    4. Write a launch file to integrate all nodes: follow.launch.py

    5. Edit the CMakeLists.txt
    
    6. Build and Run
    ```
    colcon build --packages-select cpp05_exercise
    . install/setup.bash 
    ros2 launch cpp05_exercise follow.launch.py 
    ```

## Test:

Terminal 1:
```
. install/setup.bash 
    ros2 launch cpp05_exercise follow.launch.py 
```
Terminal 2:
```
ros2 run turtlesim turtle_teleop_key
```
Using keyboard to control the motion of turtle1. The second turtle should follow the motion of the first turtle.

