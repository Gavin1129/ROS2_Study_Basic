/*
    Task: Publish student message: name, age, height
          This type of message is defined by ourselves. 
*/

// 1. Include header files
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using base_interfaces_demo::msg::Student;
using namespace std::chrono_literals;

// 3. Create a node
class TalkerStu:public rclcpp::Node{
    public:
        TalkerStu():Node("TalkerStu_node_cpp"){
            // 3.1 create a publisher
            publisher_ = this->create_publisher<Student>("chatter_stu",10);
            timer_ = this->create_wall_timer(500ms,std::bind(&TalkerStu::on_timer,this));
        }
    private:
        void on_timer(){
            auto stu = Student();
            stu.name = "Kevin";
            stu.age = age;
            age++;
            stu.height = 2.20;

            publisher_->publish(stu);
            RCLCPP_INFO(this->get_logger(),"Publish message: (%s, %d, %.2f)",stu.name.c_str(),stu.age,stu.height);
        }
        rclcpp::Publisher<Student>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        int age;
};

int main(int argc, char* argv[]){
    // 2. Initilization
    rclcpp::init(argc,argv);
    // 3. Create a publisher node
    auto node = std::make_shared<TalkerStu>();
    // 4. spin()
    rclcpp::spin(node);
    // 5. shut down
    rclcpp::shutdown();
    return 0;
}