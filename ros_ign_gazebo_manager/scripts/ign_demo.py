#!/bin/python3
import threading
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Transform


from ros_ign_gazebo_manager.ign_gazebo_interface import  IgnGazeboInterface


if __name__ == "__main__":
    rclpy.init()
    ign = IgnGazeboInterface()
    # Spin MoveIt2 node in the background
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(ign)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    input_msg="#1:resume, 2:pause, 3:create model, 4:move model 5:delete model >> "
    while(rclpy.ok()):
        mode = input(input_msg)
        mode = int(mode)
        if mode == 1:
            ign.resume()
        elif mode == 2:
            ign.pause()
        elif mode ==3:
            tf=Transform()
            tf.translation.z=1.1
            ign.create_model("obj1",tf,"https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/LitterBin",is_wait=True)
        elif mode == 4:
            tf=Transform()
            tf.translation.z=1.905
            ign.set_model_pose("obj1",tf)
        elif mode == 5:
            ign.remove_model("obj1")
    rclpy.shutdown()     