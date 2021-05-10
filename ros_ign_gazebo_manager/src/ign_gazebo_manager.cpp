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

void
convert_ros_to_ign(const ros_ign_interfaces::msg::WorldControl & ros_msg,
   ignition::msgs::WorldControl & ign_msg){
    ign_msg.set_pause(ros_msg.pause);
    ign_msg.set_multi_step(ros_msg.multi_step);
}

class IgnGazeboManager {
   public:
    IgnGazeboManager(const rclcpp::Node::SharedPtr& nh, std::string ign_world_name) : nh_(nh),ign_world_name_(ign_world_name) {
        ign_node_ = std::make_shared<ignition::transport::Node>();
        //ros srv
        ros_control_world_srv_ = nh->create_service<ros_ign_interfaces::srv::ControlWorld>("ign/"+ign_world_name_+"/control", 
                std::bind(&IgnGazeboManager::controlWorldCb, this, std::placeholders::_1,std::placeholders::_2));
        ros_create_srv_ = nh->create_service<ros_ign_interfaces::srv::SpawnEntity>("ign/"+ign_world_name_+"/create", 
                std::bind(&IgnGazeboManager::createCb, this, std::placeholders::_1,std::placeholders::_2));
        ros_remove_srv_ =  nh->create_service<ros_ign_interfaces::srv::DeleteEntity>("ign/"+ign_world_name_+"/remove", 
                std::bind(&IgnGazeboManager::removeCb, this, std::placeholders::_1,std::placeholders::_2));
        ros_move_srv_ =  nh->create_service<ros_ign_interfaces::srv::SetEntityPose>("ign/"+ign_world_name_+"/set_pose", 
                std::bind(&IgnGazeboManager::moveCb, this, std::placeholders::_1,std::placeholders::_2));
    }
    ~IgnGazeboManager(){};

   private:

    void controlWorldCb(const std::shared_ptr<ros_ign_interfaces::srv::ControlWorld::Request> request,
                        std::shared_ptr<ros_ign_interfaces::srv::ControlWorld::Response>  response){    
        // Request message
        ignition::msgs::WorldControl req;
        convert_ros_to_ign(request->world_control,req);
        ignition::msgs::Boolean rep;
        bool result;
        unsigned int timeout = 5000;
        bool executed = ign_node_->Request("/world/"+ign_world_name_+"/control", req, timeout, rep, result);
        if (executed) {
            if (result && rep.data()) {
                RCLCPP_INFO(nh_->get_logger(), "[IgnGazeboManager] Success to control! \n");
                response->success = true;
                return;
            }
        }
        //fail
        response->success =false;
        RCLCPP_ERROR(nh_->get_logger(), "[IgnGazeboManager] Failed to control\n %s", req.DebugString().c_str());
    }

    void createCb(const std::shared_ptr<ros_ign_interfaces::srv::SpawnEntity::Request> request,
                        std::shared_ptr<ros_ign_interfaces::srv::SpawnEntity::Response> response){    
        // Request message
        ignition::msgs::EntityFactory req;
        convert_ros_to_ign(request->entity_factory,req);
        ignition::msgs::Boolean rep;
        bool result;
        unsigned int timeout = 5000;
        bool executed = ign_node_->Request("/world/"+ign_world_name_+"/create", req, timeout, rep, result);
        if (executed) {
            if (result && rep.data()) {
                RCLCPP_INFO(nh_->get_logger(), "[IgnGazeboManager] Success to create %s\n", req.name().c_str());
                response->success = true;
                return;
            }
        }
        // fail
        response->success =false;
        RCLCPP_ERROR(nh_->get_logger(), "[IgnGazeboManager] Failed to create\n %s", req.DebugString().c_str());
    }

    void removeCb(const std::shared_ptr<ros_ign_interfaces::srv::DeleteEntity::Request> request,
                        std::shared_ptr<ros_ign_interfaces::srv::DeleteEntity::Response>  response){  
        // Request message
        ignition::msgs::Entity req;
        req.set_name(request->entity.name);
        //request->entity.type
        req.set_type(ignition::msgs::Entity::MODEL);
        ignition::msgs::Boolean rep;
        bool result;
        unsigned int timeout = 5000;
        bool executed = ign_node_->Request("/world/"+ign_world_name_+"/remove", req, timeout, rep, result);
        if (executed) {
            if (result && rep.data()) {
                RCLCPP_INFO(nh_->get_logger(), "[IgnGazeboManager]Success to remove %s\n", req.name().c_str());
                response->success = true;
                return;
            }
        }
        // fail
        response->success =false;
        RCLCPP_ERROR(nh_->get_logger(), "[IgnGazeboManager] Failed to remove\n %s", req.DebugString().c_str());
    }

    void moveCb(const std::shared_ptr<ros_ign_interfaces::srv::SetEntityPose::Request> request,
                        std::shared_ptr<ros_ign_interfaces::srv::SetEntityPose::Response>  response){  
        // Request message
        ignition::msgs::Pose req;
        auto& t = request->pose.position;
        auto& r = request->pose.orientation;
        ignition::math::Pose3d pose{t.x, t.y, t.z, r.w, r.x, r.y, r.z};
        ignition::msgs::Set(&req, pose);
        req.set_name(request->entity.name);
        ignition::msgs::Boolean rep;
        bool result;
        unsigned int timeout = 5000;
        bool executed = ign_node_->Request("/world/"+ign_world_name_+"/set_pose", req, timeout, rep, result);
        if (executed) {
            if (result && rep.data()) {
                RCLCPP_INFO(nh_->get_logger(), "[IgnGazeboManager]Success to move %s\n", req.name().c_str());
                response->success = true;
                return;
            }
        }
        // fail
        response->success =false;
        RCLCPP_ERROR(nh_->get_logger(), "[IgnGazeboManager] Failed to move\n %s", req.DebugString().c_str());
    }


   private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<ignition::transport::Node> ign_node_;
    //ros srv
    rclcpp::Service<ros_ign_interfaces::srv::ControlWorld>::SharedPtr ros_control_world_srv_;
    rclcpp::Service<ros_ign_interfaces::srv::SpawnEntity>::SharedPtr ros_create_srv_;
    rclcpp::Service<ros_ign_interfaces::srv::DeleteEntity>::SharedPtr ros_remove_srv_;
    rclcpp::Service<ros_ign_interfaces::srv::SetEntityPose>::SharedPtr ros_move_srv_;
    // world name
    std::string ign_world_name_;
};

int main(int argc, char* argv[]) {
    //creat ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("ign_gazebo_manager");
    // create manager
    auto manager = std::make_shared<IgnGazeboManager>(ros_node, "default");
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
