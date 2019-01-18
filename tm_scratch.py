import rosbag
from tethermask import TetherMask

# Load bag file and pass to TetherMask class
tm = TetherMask()
bag = rosbag.Bag('rkf_test01.bag', 'r')
tm.masked_playback(bag, 30)
