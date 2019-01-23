import rospy
from sensor_msgs.msg import Image
import rosbag
from time import time, sleep


class RosbagStream(object):

    def __init__(self):

        rospy.init_node('rosbag_stream')
        pub_topic = rospy.get_param(rospy.resolve_name("~output_image"))
        fin = rospy.get_param(rospy.resolve_name("~bagfile"))

        self.img_pub = rospy.Publisher(pub_topic, Image, queue_size=10)
        self.bag = rosbag.Bag(fin, 'r')

    def img_stream(self, framerate=30, loop=False):

        flag = True
        while flag:

            for topic, msg, t in self.bag:

                self.img_pub.publish(msg)
                sleep(1./framerate)

            flag = loop
