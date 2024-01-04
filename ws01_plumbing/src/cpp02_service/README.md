# Simple service and client example
Implement the sum of 2 int numbers

## Three parts:
1. Client node
2. Server node
3. Message file

## Prerequest: Message file
### 1. Create a srv folder under base_interfaces_demo
### 2. Write a .srv file : AddInts.srv
```
int32 num1
int32 num2
---
int32 sum
```
### 3. Edit CMakeLists and package files
CMakeLists.txt
```
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddInts.srv"
)
```
### 4. build and test
```
colcon build --packages-select base_interfaces_demo
. install/setup.bash
ros2 interface show base_interfaces_demo/srv/AddInts
```

## Steps:
### 1. Write a server node
demo01_server.cpp
### 2. Write a client node
demo02_client.cpp
### 3. Edit CMakeLists and package files
### 4. build and run