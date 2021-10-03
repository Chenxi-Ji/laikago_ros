#include <ros/ros.h>
#include "../../include/body.h"
#include "../../include/CurrentState.h"
#include "gazebo_msgs/SetModelState.h"
#include <gazebo_msgs/ModelStates.h>

using namespace std;
using namespace laikago_model;

class stateCallback{
  public:
    stateCallback(){
        state_sub = nm.subscribe("/gazebo/model_states", 1, &stateCallback::Callback, this);
    }

    void Callback(const gazebo_msgs::ModelStates& msg);
    geometry_msgs::Point robot_pos;
    geometry_msgs::Point robot_vel;
    geometry_msgs::Quaternion robot_ori;

  private:
    ros::NodeHandle nm;
    ros::Subscriber state_sub;
};

void stateCallback::Callback(const gazebo_msgs::ModelStates& msg)
{
    robot_ori.w = msg.pose[2].orientation.w;
    robot_ori.x = msg.pose[2].orientation.x;
    robot_ori.y = msg.pose[2].orientation.y;
    robot_ori.z = msg.pose[2].orientation.z;
        
    robot_pos.x = msg.pose[2].position.x;
    robot_pos.y = msg.pose[2].position.y;
    robot_pos.z = msg.pose[2].position.z + 0.05; 

    robot_vel.x = msg.twist[2].linear.x;
    robot_vel.y = msg.twist[2].linear.y;
    robot_vel.z = 0;
    //std::cout << "x: " << robot_pos.x << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_block_to_robot");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    

    stateCallback callback;
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    int counter = 0;
    while(ros::ok()){
        
        ros::spinOnce();
        counter++;

        geometry_msgs::Pose pr2_pose;
    pr2_pose.position = callback.robot_pos;
    pr2_pose.orientation = callback.robot_ori;
    //pr2_pose.twist.linear = callback.robot_vel;

   std::cout << "x: " << callback.robot_pos.x << std::endl;
   
   if(callback.robot_pos.x != 0){
    gazebo_msgs::ModelState pr2_modelstate;
    pr2_modelstate.model_name = (std::string) "unit_box_small";
    pr2_modelstate.pose = pr2_pose;
    pr2_modelstate.twist.linear.x = callback.robot_vel.x;
    pr2_modelstate.twist.linear.y = callback.robot_vel.y;

    
    gazebo_msgs::SetModelState srv;
    srv.request.model_state = pr2_modelstate;


    if(client.call(srv))
    {
        ROS_INFO("PR2's magic moving success!!");
        break;
    }
    else
    {
        ROS_ERROR("Failed to magic move PR2! Error msg:%s",srv.response.status_message.c_str());
    }
   }
       
    }
    
    
    
    return 0;
}