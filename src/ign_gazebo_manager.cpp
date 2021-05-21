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
#include <ros_ign_interfaces/srv/control_world.hpp>
#include <ros_ign_interfaces/srv/spawn_entity.hpp>
#include <ros_ign_interfaces/srv/set_entity_pose.hpp>
#include <ros_ign_interfaces/srv/delete_entity.hpp>
#include "service_bridge.hpp"

namespace ros_ign_bridge{

//ControlWorld service
template<>
void
convert_ros_to_ign(const ros_ign_interfaces::srv::ControlWorld::Request &request,
                    ignition::msgs::WorldControl &req){
    auto & ros_msg=request.world_control;
    req.set_pause(ros_msg.pause);
    req.set_multi_step(ros_msg.multi_step);
}
template<>
void
convert_ign_to_ros(const ignition::msgs::Boolean &rep,
                    ros_ign_interfaces::srv::ControlWorld::Response &response){
    response.success = rep.data();
}

//SpawnEntity service
template<>
void
convert_ros_to_ign(const ros_ign_interfaces::srv::SpawnEntity::Request &request,
                    ignition::msgs::EntityFactory &req){
    auto &ros_msg=request.entity_factory;
    req.set_name(ros_msg.name);
    if(!ros_msg.sdf.empty()){
        req.set_sdf(ros_msg.sdf);
    }else if(!ros_msg.sdf_filename.empty()){
        req.set_sdf_filename(ros_msg.sdf_filename);
    }else if(!ros_msg.clone_name.empty()){
        req.set_clone_name(ros_msg.clone_name);
    }
    req.set_relative_to(ros_msg.relative_to);
    req.set_allow_renaming(ros_msg.allow_renaming);
    auto& t = ros_msg.pose.position;
    auto& r = ros_msg.pose.orientation;
    ignition::math::Pose3d pose{t.x, t.y, t.z, r.w, r.x, r.y, r.z};
    ignition::msgs::Set(req.mutable_pose(), pose);
}
template<>
void
convert_ign_to_ros(const ignition::msgs::Boolean &rep,
                    ros_ign_interfaces::srv::SpawnEntity::Response &response){
    response.success = rep.data();
}

//DeleteEntity service
template<>
void
convert_ros_to_ign(const ros_ign_interfaces::srv::DeleteEntity::Request &request,
                    ignition::msgs::Entity &req){
    req.set_name(request.entity.name);
    //request.entity.type
    req.set_type(ignition::msgs::Entity_Type(request.entity.type));
}
template<>
void
convert_ign_to_ros(const ignition::msgs::Boolean &rep,
                    ros_ign_interfaces::srv::DeleteEntity::Response &response){
    response.success = rep.data();
}

//SetEntityPose service
template<>
void
convert_ros_to_ign(const ros_ign_interfaces::srv::SetEntityPose::Request &request,
                    ignition::msgs::Pose &req){
    auto& t = request.pose.position;
    auto& r = request.pose.orientation;
    ignition::math::Pose3d pose{t.x, t.y, t.z, r.w, r.x, r.y, r.z};
    ignition::msgs::Set(&req, pose);
    req.set_name(request.entity.name);
}

template<>
void
convert_ign_to_ros(const ignition::msgs::Boolean &rep,
                    ros_ign_interfaces::srv::SetEntityPose::Response &response){
    response.success = rep.data();
}

}



using namespace ros_ign_bridge;
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    // ROS node
    auto ros_node = std::make_shared<rclcpp::Node>("ign_gazebo_manager");
    // Ignition node
    auto ign_node = std::make_shared<ignition::transport::Node>();
    // parameter
    std::string ign_world_name = "default";
    // create control service bridge
    using ControlServiceBrigde=IgnServiceBrigde<ros_ign_interfaces::srv::ControlWorld,
                                ignition::msgs::WorldControl,ignition::msgs::Boolean>;
    auto control_service_bridge = std::make_shared<ControlServiceBrigde>(
        ros_node,ign_node,
        "ign/"+ign_world_name+"/control","/world/"+ign_world_name+"/control");
    // create spawn entity service bridge
    using SpawnServiceBrigde=IgnServiceBrigde<ros_ign_interfaces::srv::SpawnEntity,
                                ignition::msgs::EntityFactory,ignition::msgs::Boolean>;
    auto spawn_service_bridge = std::make_shared<SpawnServiceBrigde>(
        ros_node,ign_node,
        "ign/"+ign_world_name+"/create","/world/"+ign_world_name+"/create");
    // create delete entity service bridge
    using DeleteServiceBrigde=IgnServiceBrigde<ros_ign_interfaces::srv::DeleteEntity,
                                ignition::msgs::Entity,ignition::msgs::Boolean>;
    auto delete_service_bridge = std::make_shared<DeleteServiceBrigde>(
        ros_node,ign_node,
        "ign/"+ign_world_name+"/remove","/world/"+ign_world_name+"/remove");
    // create set pose  service bridge
    using SetPoseServiceBrigde=IgnServiceBrigde<ros_ign_interfaces::srv::SetEntityPose,
                                ignition::msgs::Pose,ignition::msgs::Boolean>;
    auto set_pose_service_bridge = std::make_shared<SetPoseServiceBrigde>(
        ros_node,ign_node,
        "ign/"+ign_world_name+"/set_pose","/world/"+ign_world_name+"/set_pose");  
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
