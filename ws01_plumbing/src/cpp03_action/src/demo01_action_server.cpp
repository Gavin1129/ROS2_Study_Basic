/*
  Server node
  Task: Write a server node to receive a request
  Steps:
    1. Create an action server object
    2. Process the number
    3. Generate the feedback (callback function)
    4. Response the final result (callback function)
    5. Process the cancle request (callback function)
*/

// 1. Include header files
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using base_interfaces_demo::action::Progress;

class ProgressActionServer: public rclcpp::Node{
  public:
    ProgressActionServer():Node("progress_action_server_node"){
      RCLCPP_INFO(this->get_logger(),"Action server created!");
      // 3.1 Create an action server object
      // rclcpp_action::Server<ActionT>::SharedPtr 
      // create_server<ActionT, NodeT>(
      //              NodeT node, 
      //              const std::string &name,
      //              rclcpp_action::Server<ActionT>::GoalCallback handle_goal, 
      //              rclcpp_action::Server<ActionT>::CancelCallback handle_cancel, 
      //              rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted, 
      //              const rcl_action_server_options_t &options = rcl_action_server_get_default_options(), 
      //              rclcpp::CallbackGroup::SharedPtr group = nullptr)
      server_ = rclcpp_action::create_server<Progress>(
            this,
            "get_sum",
            std::bind(&ProgressActionServer::handle_goal,this,_1,_2),
            std::bind(&ProgressActionServer::handle_cancel,this,_1),
            std::bind(&ProgressActionServer::handle_accepted,this,_1));
    }

    /*
      /// Signature of a callback that accepts or rejects goal requests.
      using GoalCallback = std::function<GoalResponse(
          const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
      /// Signature of a callback that accepts or rejects requests to cancel a goal.
      using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
      /// Signature of a callback that is used to notify when the goal has been accepted.
      using AcceptedCallback = std::function<void(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    */


    /****************************************/
          // 3.2 Process the number
    /****************************************/
    // using GoalCallback = std::function<GoalResponse(
    //     const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, 
      std::shared_ptr<const Progress::Goal> goal){
        // uuid is the identity for the server
        (void)uuid; 
        if(goal->num <= 1){
          RCLCPP_INFO(this->get_logger(),"The number must be larger than 1!");
          return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(),"The number is acceptable!");

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    }


    /*****************************************/
    // 3.5 Process the cancle request (callback function)
    /****************************************/
    // using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle){
      (void)goal_handle;
      RCLCPP_INFO(this->get_logger(),"Cancled!");
      return rclcpp_action::CancelResponse::ACCEPT;
    }


    /****************************************/
    // 3.3. Generate the feedback (callback function)
    // 3.4. Response the final result (callback function)
    /****************************************/
    // using AcceptedCallback = std::function<void(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle){
      // new thread
      std::thread(std::bind(&ProgressActionServer::execute,this,goal_handle)).detach();
    }

    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle){
      // 1. generate feedback
      // void publish_feedback(std::shared_ptr<base_interfaces_demo::action::Progress_Feedback> feedback_msg)
      // goal_handle->publish_feedback()
      int num = goal_handle->get_goal()->num;
      int sum = 0;
      auto feedback = std::make_shared<Progress::Feedback>();
      auto result = std::make_shared<Progress::Result>();
      rclcpp::Rate rate(1.0);
      for(int i = 0; i <= num; i++){
        sum += i;
        double progress = i / (double)num;
        feedback->progress = progress;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(),"Responsing, progress:%.2f",progress);

        if(goal_handle->is_canceling()){
          result->sum = sum;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(),"Task cancled!");
          return;
        }
        rate.sleep();
      }

      // 2. final result
      // void succeed(std::shared_ptr<base_interfaces_demo::action::Progress_Result> result_msg)
      // goal_handle->succeed()
      if(rclcpp::ok()){
        result->sum = sum;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(),"The final results: %d",sum);
      }
    }

    /****************************************/
    private:
    rclcpp_action::Server<Progress>::SharedPtr server_;

};


int main(int argc, char* argv[]){
  rclcpp::init(argc,argv);
  auto server = std::make_shared<ProgressActionServer>();
  rclcpp::spin(server);
  rclcpp::shutdown();
  return 0;
}
