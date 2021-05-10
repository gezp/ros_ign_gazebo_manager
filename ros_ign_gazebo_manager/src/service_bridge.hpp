/*******************************************************************************
 *  Copyright (c) Gezp (https://github.com/gezp), All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <ros_ign_interfaces/srv/control_world.hpp>
#include <ros_ign_interfaces/srv/spawn_entity.hpp>
#include <ros_ign_interfaces/srv/set_entity_pose.hpp>
#include <ros_ign_interfaces/srv/delete_entity.hpp>
#include <ros_ign_bridge/convert_decl.hpp>

namespace ros_ign_bridge
{

template<>
void
convert_ros_to_ign(const ros_ign_interfaces::msg::EntityFactory & ros_msg,
   ignition::msgs::EntityFactory & ign_msg){
    ign_msg.set_name(ros_msg.name);
    if(!ros_msg.sdf.empty()){
        ign_msg.set_sdf(ros_msg.sdf);
    }else if(!ros_msg.sdf_filename.empty()){
        ign_msg.set_sdf_filename(ros_msg.sdf_filename);
    }else if(!ros_msg.clone_name.empty()){
        ign_msg.set_clone_name(ros_msg.clone_name);
    }
    ign_msg.set_relative_to(ros_msg.relative_to);
    ign_msg.set_allow_renaming(ros_msg.allow_renaming);
    auto& t = ros_msg.pose.position;
    auto& r = ros_msg.pose.orientation;
    ignition::math::Pose3d pose{t.x, t.y, t.z, r.w, r.x, r.y, r.z};
    ignition::msgs::Set(ign_msg.mutable_pose(), pose);
}

template<>
void
convert_ros_to_ign(const ros_ign_interfaces::msg::WorldControl & ros_msg,
   ignition::msgs::WorldControl & ign_msg){
    ign_msg.set_pause(ros_msg.pause);
    ign_msg.set_multi_step(ros_msg.multi_step);
}


template<typename ROS_SERVICE_T, typename IGN_REQUEST_T, typename IGN_RESPONSE_T>
class IgnServiceBrigde{
   public:
    IgnServiceBrigde(rclcpp::Node::SharedPtr ros_node,
                    std::shared_ptr<ignition::transport::Node> ign_node, 
                    const std::string & ros_service_name,
                    const std::string & ign_service_name,
                    int ign_timeout=5000) 
    : ros_node_(ros_node),ign_node_(ign_node),
    ign_service_name_(ign_service_name),
    ign_timeout_(ign_timeout) 
    {
        //ros srv
        using std::placeholders::_1;
        using std::placeholders::_2;
        ros_srv_ = ros_node_->create_service<ROS_SERVICE_T>(ros_service_name,
            std::bind(&IgnServiceBrigde::ros_callback, this, _1,_2));
    }
    ~IgnServiceBrigde(){};

    void ros_callback(const std::shared_ptr<typename ROS_SERVICE_T::Request> request,
                      const std::shared_ptr<typename ROS_SERVICE_T::Response> response)
    { 
        // Request message
        IGN_REQUEST_T req;
        convert_ros_to_ign(*request,req);
        IGN_RESPONSE_T rep;
        bool result;
        bool executed = ign_node_->Request(ign_service_name_, req, ign_timeout_, rep, result);
        if (executed) {
            if (result) {
                convert_ign_to_ros(rep,*response);
            }else{
                RCLCPP_ERROR(ros_node_->get_logger(), "Ignition Service %s call failed\n %s",
                    ign_service_name_, req.DebugString().c_str());  
            }
        }else{
            RCLCPP_ERROR(ros_node_->get_logger(), "Ignition Service %s call timed out\n %s",
                ign_service_name_, req.DebugString().c_str());  
        } 
    } 
    private:
        rclcpp::Node::SharedPtr ros_node_;
        std::shared_ptr<ignition::transport::Node> ign_node_;
        typename rclcpp::Service<ROS_SERVICE_T>::SharedPtr ros_srv_;
        std::string ign_service_name_;
        int ign_timeout_;
};


}