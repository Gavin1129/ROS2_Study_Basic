# Action server and client example
Client node send an int number n. 
Server receive the number and accumulates the sum of all integers from 1 to n.
Return the result and response the process during the calculation 

# Parts:
## 1. Write an action server node
## 2. Write an action client node
## 3. Write the message file

# Prerequest: message file
## 1. Create an action file 
base_interfaces_demo/action/Progress.action
```
int64 num
---
int64 sum
---
float64 progress
```
## 2. Edit CMakeLists.txt and package.xml
CMakeLists.txt
```
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Progress.action"
)
```
package.xml
```
<depend>action_msgs</depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
## 3. Build and test
```
colcon build
. install/setup.bash
ros2 interface show base_interfaces_demo/action/Progress

```

# Steps:
## 1. server node
demo01_action_server.cpp
## 2. client node
## 3. Edit CMakeLists and package
## 4. Build and run