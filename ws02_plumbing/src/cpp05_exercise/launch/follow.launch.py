from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    #1. turtlesim_node
    turtle = Node(package="turtlesim", executable="turtlesim_node")
    #2. spawn node
    spawn = Node(package="cpp05_exercise",executable="exer01_spawn",parameters=[{"turtle_name":"t2"}])
    #3. broadcast the postion of each turtle in the world frame
    broadcaster1 = Node(package="cpp05_exercise",executable="exer02_tf_broadcaster",name="broa1")
    broadcaster2 = Node(package="cpp05_exercise",executable="exer02_tf_broadcaster",name="broa2",
                        parameters=[{"turtle":"t2"}]
                        )
    #4.  create a listener
    listener = Node(package="cpp05_exercise",executable="exer03_tf_listener",
                    parameters=[{"father_frame":"t2","child_frame":"turtle1"}])
    
    return LaunchDescription([turtle,spawn,broadcaster1,broadcaster2,listener])