# Topics Example 1: 
## Prerequisites:
### 1. Create a package
```
ros2 pkg create cpp01_topic --build-type ament_cmake --dependencies rclcpp std_msgs base_interfaces_demo --node-name demo01_talker_str

ros2 pkg create py01_topic --build-type ament_python --dependencies rclpy std_msgs base_interfaces_demo --node-name demo01_talker_str_py
```
## Steps:
### 1. Write the publisher node
demo01_talker_str.cpp
### 2. Write the subscriber node
demo02_listener_str.cpp
### 3. Edit package.xml and CMakeLists.txt
### 4. Build and Run
```
colcon build
. install/setup.bash
ros2 run cpp01_topic demo01_talker_str 
```
```
ros2 run cpp01_topic demo02_listener_str
```
### 5. Check:
```
ros2 topic list  //check active topics
ros2 topic echo /chatter // check messages
```


# Topics Example 2: 
## Prerequisites:
Create a msg folder under base_interfaces_demo package.

Create Student.msg file under msg folder

Modify Student.msg file

Modify CMakeLists.txt and package.xml files in base_interfaces_demo package. 

in package.xml:
```

  <build_depend>rosidl_default_generator</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

in CMakeList.txt
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Student.msg"
)
```

Build package:
```
colcon build --packages-select base_interfaces_demo
```

Test:
```
. install/setup.bash
ros2 interface show base_interfaces_demo/msg.Student
```

Add include path:
${workspaceFolder}/install/base_interfaces_demo/include/**


## Steps:
### 1. Write the publisher node
demo03_talker_stu.cpp
### 2. Write the subscriber node
demo04_listener_stu.cpp
### 3. Edit package.xml and CMakeLists.txt
### 4. Build and Run
```
colcon build
. install/setup.bash
ros2 run cpp01_topic demo03_talker_stu 
```
```
ros2 run cpp01_topic demo04_listener_stu
```