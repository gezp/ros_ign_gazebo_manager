
import threading
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Transform
from ros_ign_interfaces.srv import ControlWorld,CreateModel,RemoveModel,MoveModel

class IgnGazeboInterface(Node):
    def __init__(self,world_name="default",nodename="IgnGazeboInterface"):
        super().__init__(nodename)
        #control client
        srv_name = '/ign/%s/control'%world_name
        self.control_cli = self.create_client(ControlWorld,srv_name)
        while not self.control_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service %s not available, waiting again...'%srv_name)
        #create client
        srv_name = '/ign/%s/create'%world_name
        self.create_cli = self.create_client(CreateModel,srv_name )
        while not self.create_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service %s not available, waiting again...'%srv_name)
        #remove client
        srv_name = '/ign/%s/remove'%world_name
        self.remove_cli = self.create_client(RemoveModel, srv_name)
        while not self.remove_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service %s not available, waiting again...'%srv_name)
        #move client
        srv_name = '/ign/%s/set_pose'%world_name
        self.move_cli = self.create_client(MoveModel, srv_name)
        while not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service %s not available, waiting again...'%srv_name)
        #
        self.get_logger().info("IgnGazeboInterface initialised successfuly")

    def start(self,is_wait=False):
        req = ControlWorld.Request()
        req.world_control.pause = False
        srv_call = self.control_cli.call_async(req)
        if is_wait:
            #wait
            while rclpy.ok(): 
                if srv_call.done():
                    break
            #result
            return srv_call.result().success
        return True

    def pause(self,step=0,is_wait=False):
        req = ControlWorld.Request()
        req.world_control.pause = True
        req.world_control.multi_step =step
        srv_call = self.control_cli.call_async(req)
        if is_wait:
            #wait
            while rclpy.ok(): 
                if srv_call.done():
                    break
            #result
            return srv_call.result().success
        return True

    def create_model(self,name,tf,model_path,is_wait=False):
        req = CreateModel.Request()
        req.entity_factory.name = name
        req.entity_factory.tf = tf
        req.entity_factory.sdf_filename = model_path
        srv_call = self.create_cli.call_async(req)
        if is_wait:
            #wait
            while rclpy.ok(): 
                if srv_call.done():
                    break
            #result
            return srv_call.result().success
        return True

    def remove_model(self,name,is_wait=False):
        req = RemoveModel.Request()
        req.name = name
        srv_call = self.remove_cli.call_async(req)
        if is_wait:
            #wait
            while rclpy.ok(): 
                if srv_call.done():
                    break
            #result
            return srv_call.result().success
        return True
        
    def set_model_pose(self,name,tf,is_wait=False):
        req = MoveModel.Request()
        req.name = name
        req.tf=tf
        srv_call = self.move_cli.call_async(req)
        if is_wait:
            #wait
            while rclpy.ok(): 
                if srv_call.done():
                    break
            #result
            return srv_call.result().success
        return True
