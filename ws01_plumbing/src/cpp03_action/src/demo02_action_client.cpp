/*
  Task:
    Send a number to server node and receive the result from server
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using namespace std::chrono_literals;
using base_interfaces_demo::action::Progress;

using std::placeholders::_1;
using std::placeholders::_2;

class ProgressActionClient: public rclcpp::Node{
  public:
    ProgressActionClient():Node("progress_action_client_node"){
      RCLCPP_INFO(this->get_logger(),"Action Client created!");
      // 3.1 cretae a client node
      // rclcpp_action::Client<ActionT>::SharedPtr <typename ActionT> 
      // create_client(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface, 
      //               rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface, 
      //               rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface, 
      //               rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
      //               const std::string &name, rclcpp::CallbackGroup::SharedPtr group = nullptr,
      //               const rcl_action_client_options_t &options = rcl_action_client_get_default_opt
      client_ = rclcpp_action::create_client<Progress>(this,"get_sum");
    }
    // 3.2 send a goal
    void send_goal(int num){
      // 1. connect to server
      if(!client_->wait_for_action_server(10s)){
        RCLCPP_ERROR(this->get_logger(),"Server connecting failed!");
        return;
      }

      // 2. send a request
      // std::shared_future<...> rclcpp_action::Client<...>::async_send_goal(
      //  const base_interfaces_demo::action::Progress::Goal &goal,
      //  const rclcpp_action::Client<...>::SendGoalOptions &options)
      auto goal = Progress::Goal();
      goal.num = num;
      rclcpp_action::Client<Progress>::SendGoalOptions options;
      options.goal_response_callback = std::bind(&ProgressActionClient::goal_response_callback,this,_1);
      options.feedback_callback = std::bind(&ProgressActionClient::feedback_callback,this,_1,_2);
      options.result_callback= std::bind(&ProgressActionClient::result_callback,this,_1);
      auto future = client_->async_send_goal(goal, options);
    }

    /*
      using GoalHandle = ClientGoalHandle<ActionT>;
      using GoalResponseCallback = std::function<void (typename GoalHandle::SharedPtr)>;
      using FeedbackCallback = typename GoalHandle::FeedbackCallback;
      using ResultCallback = typename GoalHandle::ResultCallback;
    */

    // 3. callback function: server response 
    //    server accept the number or not
    void goal_response_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle){
      if(!goal_handle){
        RCLCPP_INFO(this->get_logger(),"Target not accepted!");
      }else{
        RCLCPP_INFO(this->get_logger(),"Target accepted!");
      }
    }

    // 4.
    // using FeedbackCallback =
    // std::function<void (
    //    typename ClientGoalHandle<ActionT>::SharedPtr,
    //    const std::shared_ptr<const Feedback>)>;
    void feedback_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle,
                           const std::shared_ptr<const Progress::Feedback> feedback){
      (void)goal_handle;
      double progress = feedback->progress;
      RCLCPP_INFO(this->get_logger(),"Progress: %.2f",progress);
    }

    // 5. 
    // using ResultCallback = std::function<void (const WrappedResult & result)>;
    void result_callback(const rclcpp_action::ClientGoalHandle<Progress>::WrappedResult & result){
      // result.code
      //rclcpp_action::ResultCode
      if(result.code == rclcpp_action::ResultCode::SUCCEEDED){
        RCLCPP_INFO(this->get_logger(),"Final result: %ld", result.result->sum);
      }else if(result.code == rclcpp_action::ResultCode::ABORTED){
        RCLCPP_INFO(this->get_logger(),"Request Aborted!");
      }else if(result.code == rclcpp_action::ResultCode::CANCELED){
        RCLCPP_INFO(this->get_logger(),"Request Canceled!");
      }else{
        RCLCPP_INFO(this->get_logger(),"Unknown Error!");
      }
    }
  private:
    rclcpp_action::Client<Progress>::SharedPtr client_;
};
int main(int argc, char* argv[]){
  if(argc   != 2){
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Please input an int number!");
    return 1;
  }
  rclcpp::init(argc,argv);
  auto Client = std::make_shared<ProgressActionClient>();
  Client->send_goal(atoi(argv[1]));
  rclcpp::spin(Client);
  rclcpp::shutdown();
  return 0;
}