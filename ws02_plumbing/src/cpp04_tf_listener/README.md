# tf_listener example: transformation between frames

## Publish frame relationship: base_link with laser 
in a new terminal
```
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id laser --x 0.4 --z 0.2
```
## Publish frame relationship: 
in a new terminal
```
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id camera --x -0.5 
--z 0.4
```
## run listener node:
```
ros2 run cpp04_tf_listener demo01_tf_listener 
```

## result:
```
-------Tranfered Result------
Parent frame: camera, Child frame: laser, Offset: (0.90, 0.00, -0.20)
```

# Point coordinates transformation in two frames
Given:

    1. relationship between two frames (laser => base_link)
    2. Point position in laser frame
Find:

    Point position in base_link

Steps:

    1. subscribe data
        1.1 create a listener for frames
        1.2 create a subscriber for point coordinates 
    2. Find solution
        create a filter to combine above two together. 

## packages.xml
Add
```
<depend>tf2_geometry_msgs</depend>
<depend>message_filters</depend>
```
## CMakeList.txt
```
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
```
## Test:

    1. publish static_transform_publisher
```
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id laser --x 0.4 --z 0.2

```
    2. publish point location
```
ros2 run cpp03_tf_broadcaster demo03_point_tf_broadcaster 

```
    3. find solution
```
ros2 run cpp04_tf_listener demo02_msg_filter 
```

    4. Result:
```
...
[tf_point_listener]: Parent frame: base_link, Location:(0.60,0.00,0.10)
[tf_point_listener]: Parent frame: base_link, Location:(0.65,0.00,0.10)
[tf_point_listener]: Parent frame: base_link, Location:(0.70,0.00,0.10)
...
```