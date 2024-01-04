/*
    Task: Receive student message: name, age, height
          This type of message is defined by ourselves. 
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using base_interfaces_demo::msg::Student;

class ListenerStu:public rclcpp::Node{
    public:
        ListenerStu():Node("listener_stu_cpp"){
            RCLCPP_INFO(this->get_logger(),"Listener created!");
            sub_ = this->create_subscription<Student>("chatter_stu",10,std::bind(&ListenerStu::do_cb,this,std::placeholders::_1));
        }
    private:
        void do_cb(const Student &stu){
            RCLCPP_INFO(this->get_logger(),"Received messages: (name = %s, age = %d, height = %.2f)",stu.name.c_str(), stu.age, stu.height);
        }
        rclcpp::Subscription<Student>::SharedPtr sub_;
};

int main(int argc, char* argv[]){
    // 2. initilization
    rclcpp::init(argc,argv);
    // 3. Create a subscriber node
    auto node = std::make_shared<ListenerStu>();
    // 4. spin()
    rclcpp::spin(node);
    // 5. 
    rclcpp::shutdown();
    return 0;
}