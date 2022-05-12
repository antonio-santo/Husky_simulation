import rospy
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import LinkState # For getting information about link states
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


rospy.init_node("position_ground_truth")
header=0
child_frame_id=[]
twist=[]
odom_pub=rospy.Publisher("husky_ground_truth_position2", Odometry, queue_size=1)
odom=Odometry()
header=Header()
header.frame_id = '/world'
odom.header=header
r = rospy.Rate(1000)
while not rospy.is_shutdown():

    model_info_prox = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    rospy.wait_for_service('/gazebo/get_link_state')
    gt=model_info_prox("base_link", "world")
    #publica el ground truth de odometria

    pos_act=gt.link_state.pose
    odom.pose.pose=pos_act
    header.stamp= rospy.Time.now()
    odom.header=header
    odom_pub.publish(odom)
