import rospy
from sensor_msgs.msg import Image
import rosbag
from time import sleep


class RosbagStream(object):

    def __init__(self):

        rospy.init_node('rosbag_stream')
        pub_topic = rospy.get_param(rospy.resolve_name("~output_image"), "/stabilized_image")
        fin = rospy.get_param(rospy.resolve_name("~bagfile"), "/home/dickinsonlab/git/rkf_mask_launch/rkf_test01.bag")

        self.img_pub = rospy.Publisher(pub_topic, Image, queue_size=10)
        self.bag = rosbag.Bag(fin, 'r')

    def img_stream(self, framerate=30):

        flag = True
        while flag and ~rospy.is_shutdown():

            for topic, msg, t in self.bag:

                self.img_pub.publish(msg)
                sleep(1./framerate)

            loop = rospy.get_param(rospy.resolve_name("~loop"), False)
            flag = loop

            # if rospy.is_shutdown(): break
