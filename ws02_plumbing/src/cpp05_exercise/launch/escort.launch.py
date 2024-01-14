from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

""""
Task: Turtle escort
steps:
    1. publishe the goal positon relevant to turtle1
    2. listen the turtle2's position relevant to the goal 
"""

def generate_launch_description():

    escort_back = DeclareLaunchArgument(name="turtle_back",default_value="turtle_back")
    escort_left = DeclareLaunchArgument(name="turtle_left",default_value="turtle_left")
    escort_right = DeclareLaunchArgument(name="turtle_right",default_value="turtle_right")

    master = Node(package="turtlesim",executable="turtlesim_node")
    spawn_pack = Node(package="cpp05_exercise",executable="exer01_spawn",
                      name="spawn_back",
                      parameters=[{"x":2.0,"y":5.0,"turtle_name":LaunchConfiguration("turtle_back")}])
    spawn_left = Node(package="cpp05_exercise",executable="exer01_spawn",
                      name="spawn_back",
                      parameters=[{"x":3.0,"y":9.0,"turtle_name":LaunchConfiguration("turtle_left")}])
    spawn_right = Node(package="cpp05_exercise",executable="exer01_spawn",
                      name="spawn_back",
                      parameters=[{"x":4.0,"y":2.0,"turtle_name":LaunchConfiguration("turtle_right")}])
    
    turtle1_world = Node(package="cpp05_exercise",executable="exer02_tf_broadcaster",name="turtle1_world")
    back_world = Node(package="cpp05_exercise",executable="exer02_tf_broadcaster",name="back_world",
                      parameters=[{"turtle":LaunchConfiguration("turtle_back")}]
                      )
    left_world = Node(package="cpp05_exercise",executable="exer02_tf_broadcaster",name="left_world",
                      parameters=[{"turtle":LaunchConfiguration("turtle_left")}]
                      )
    right_world = Node(package="cpp05_exercise",executable="exer02_tf_broadcaster",name="right_world",
                      parameters=[{"turtle":LaunchConfiguration("turtle_right")}]
                      )
    

    excort_goal_back = Node(package="tf2_ros",executable="static_transform_publisher", name="excort_goal_back",
                            arguments=["--frame-id","turtle1","--child-frame-id","escort_goal_back","--x","-1.5"])
    excort_goal_left = Node(package="tf2_ros",executable="static_transform_publisher", name="excort_goal_left",
                            arguments=["--frame-id","turtle1","--child-frame-id","escort_goal_left","--y","1.5"])
    excort_goal_right = Node(package="tf2_ros",executable="static_transform_publisher", name="excort_goal_right",
                            arguments=["--frame-id","turtle1","--child-frame-id","escort_goal_right","--y","-1.5"])


    back_escort_goal_back = Node(package="cpp05_exercise",executable="exer03_tf_listener",
                                 name="back_escort_goal_back",
                                 parameters=[{"father_frame":LaunchConfiguration("turtle_back"),
                                              "child_frame":"escort_goal_back"}])
    left_escort_goal_left = Node(package="cpp05_exercise",executable="exer03_tf_listener",
                                 name="left_escort_goal_left",
                                 parameters=[{"father_frame":LaunchConfiguration("turtle_left"),
                                              "child_frame":"escort_goal_left"}])
    right_escort_goal_right = Node(package="cpp05_exercise",executable="exer03_tf_listener",
                                 name="right_escort_goal_right",
                                 parameters=[{"father_frame":LaunchConfiguration("turtle_right"),
                                              "child_frame":"escort_goal_right"}])
    
    
    return LaunchDescription([escort_back,master,spawn_pack,turtle1_world,back_world,excort_goal_back,back_escort_goal_back,
                              escort_left,spawn_left,left_world,excort_goal_left,left_escort_goal_left,
                              escort_right,spawn_right,right_world,excort_goal_right,right_escort_goal_right,])