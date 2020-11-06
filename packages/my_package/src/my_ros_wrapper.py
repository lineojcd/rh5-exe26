#!/usr/bin/env python3

import os
import rospy
import rosnode
from duckietown.dtros import DTROS,NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import gym_duckietown
from gym_duckietown.simulator import Simulator
import numpy as np
import cv2
from cv_bridge import CvBridge

# from std_msgs.msg import String
# import rosbag

class MyROSWrapperNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        #color = rospy.get_param('~color')
        super(MyROSWrapperNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct Publisher and Subscriber
        self.pub = rospy.Publisher('/fakebot/camera_node/image/compressed', CompressedImage, queue_size=10)
        self.sub = rospy.Subscriber('/fakebot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, self.callback)
        #self.sub = rospy.Subscriber('chatter', String, self.callback)
        self.iter = 0

        # create bag files:
        # self.bag = bag

        self.env = Simulator(
            seed=123,  # random seed
            map_name="loop_empty",
            max_steps=500001,  # we don't want the gym to reset itself
            domain_rand=0,
            camera_width=640,
            camera_height=480,
            accept_start_angle_deg=4,  # start close to straight
            full_transparency=True,
            distortion=True,
        )

        self.br=CvBridge()
        self.right_wheel = 0
        self.left_wheel = 0


    def callback(self, data):
        if data:
            # update wheel command to action
            self.right_wheel=data.vel_right
            self.left_wheel=data.vel_left

            action = [self.left_wheel, self.right_wheel]
            print("I got command:", action )
            observation, reward, done, misc = self.env.step(action)
            # env.render()

            # print("observation.shape:",observation.shape )
            # observation.shape: (480, 640, 3)
            # print("observation.type:", type(observation))
            # observation.type: <class 'numpy.ndarray'>

            # publish image to viewer
            # np_arr = np.fromstring(data.data, np.uint8)
            # img = cv2.imdecode(observation, cv2.IMREAD_COLOR)

            img = cv2.cvtColor(np.asarray(observation), cv2.COLOR_RGB2BGR)
            compressed_img_msg = self.br.cv2_to_compressed_imgmsg(img, dst_format='jpg')
            self.pub.publish(compressed_img_msg)

            if done:
                self.env.reset()
        else:
            rospy.loginfo("waiting for Publisher")



# class MyNode(DTROS):
#
#     def __init__(self, node_name,bag):
#         # initialize the DTROS parent class
#         #color = rospy.get_param('~color')
#         super(MyNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
#         # construct publisher
#         self.pub = rospy.Publisher('/amd64/camera_node_{}/image/compressed'.format(color), CompressedImage, queue_size=10)
#         self.sub = rospy.Subscriber('/jcdgo/camera_node/image/compressed', CompressedImage, self.callback)
#         #self.sub = rospy.Subscriber('chatter', String, self.callback)
#         self.iter = 0
#
#         # create bag files:
#         self.bag = bag
#
#     def callback(self, data):
#         if data:
#             #colorr = rospy.get_param("/my_node_red/color")
#             np_arr = np.fromstring(data.data, np.uint8)
#             img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#             #rospy.loginfo("I heard {}".format(np.shape(img)))
#             #rospy.loginfo("{}".format(ns))
#
#             print("color in call back:", color)
#             img2 = add_rectangle(img,color)
#
#             if self.iter %1000 ==0:
#                 # cv2.imwrite("mounted_volume/testimgdector.png", img2)
#                 cv2.imwrite("/code/catkin_ws/src/rh4-exe21b/mounted_v/testimgdector.png", img2)
#                 print("Saved the test dected images", self.iter)
#                 self.iter +=1
#     #     		print("info of this image:")
#
#             self.iter +=1
#
#             compressed_img_msg = br.cv2_to_compressed_imgmsg(img2, dst_format='jpg')
#             # self.bag.write('/jcdgo/camera_node/image/compressed', compressed_img_msg)
#             # rospy.loginfo("Publishing color detected img msg")
#
#             write2bag(self.bag,compressed_img_msg )
#             self.pub.publish(compressed_img_msg)
#         else:
#             rospy.loginfo("waiting for Publisher")


if __name__ == '__main__':
    # create the node
    # ns = rospy.get_namespace()
    # color = str(ns[1:-1])
    # br = CvBridge()

    # def updateEnv(action):
    #     observation, reward, done, misc = env.step(action)
    #     # env.render()
    #     if done:
    #         env.reset()
    #
    #     return img

    #color = rospy.get_param("/my_node_red/color")
    # def write2bag(bag_file, write_msg):
    #     bag_file.write( '/jcdgo/camera_node/image/compressed' ,write_msg )
    # bag = rosbag.Bag('mounted_volume/amod20-rh3-ex-color-xiaoao-song.bag', 'w')
    # bag = rosbag.Bag('/code/catkin_ws/src/rh4-exe21b/mounted_v/amod20-rh3-ex-color-xiaoao-song.bag', 'w')

    node = MyROSWrapperNode(node_name='my_ros_wrapper_node')
    # keep spinning
    rospy.spin()