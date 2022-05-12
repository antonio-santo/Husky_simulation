import rospy
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import LinkState # For getting information about link states
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from math import atan2, sqrt, pow, pi
import pandas as pd


if __name__ == '__main__':

    rospy.init_node("pd_control")
    points = pd.read_csv("./trajectories/waypoints_lab.csv", header=0)
    i=0
    header=0
    child_frame_id=[]
    twist=[]
    #Constantes para el control
    kp_l=1
    kd_l=0.3
    kp_a=1
    kd_a=0.4
    previous_distance=0
    previous_angle=0
    reached=0
    angular=0
    last_rotation=0
    #x_goal=4
    #y_goal=-4
    #angle_final=30
    #theta_final=(angle_final*(3.1415926))/180

    pub_vel = rospy.Publisher("/husky_velocity_controller/cmd_vel", Twist, queue_size = 1)
    speed = Twist()
    r = rospy.Rate(10)

    while not rospy.is_shutdown():

        if reached ==0:

            x_goal=points.iloc[i][0]
            y_goal=points.iloc[i][1]
            print("X_goal:%d , Y_goal: %d" % (x_goal,y_goal) )

            model_info_prox = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            rospy.wait_for_service('/gazebo/get_link_state')
            gt=model_info_prox("base_link", "world")
            #concces el ground truth de odometria
            pos_act=gt.link_state.pose
            #Paso de cuaternios a Euler
            rot_q_act=gt.link_state.pose.orientation
            (roll, pitch, theta) = euler_from_quaternion([rot_q_act.x, rot_q_act.y, rot_q_act.z, rot_q_act.w])


            #Calculo de la tangente
            x_act=pos_act.position.x
            y_act=pos_act.position.y
            inc_x = x_goal - x_act
            inc_y = y_goal - y_act
            #angle_to_goal = atan2(inc_y, inc_x)

            angle_to_goal = atan2(inc_y, inc_x)

            #Control PD v_lineal
            goal_distance=sqrt(pow(x_goal - x_act, 2) + pow(y_goal - y_act, 2))
            distance=goal_distance
            derivative=distance-previous_distance
            velocidad_lineal=kp_l*distance + kd_l*derivative
            previous_distance=distance


            print("angulo a corregir", angle_to_goal)
            print("orientacion del husky", theta)

            #Control PD v_angular
            if theta>0 and angle_to_goal>0:
                angle=angle_to_goal - theta

            elif theta<0.0 and angle_to_goal<0:
                angle=angle_to_goal - theta

            elif theta>0 and angle_to_goal<0:
                if(angle_to_goal+pi)>theta:
                    angle=angle_to_goal - theta
                else:
                    angle_to_goal=angle_to_goal+2*pi
                    angle=angle_to_goal - theta

            elif theta<0 and angle_to_goal>0:
                if(angle_to_goal+pi)<(theta+ 2*pi):
                    angle=angle_to_goal - theta
                else:
                    theta=theta +2*pi
                    angle=angle_to_goal - theta


            print(angle)
            angle_error=angle
            derivative_a=angle_error-previous_angle
            velocidad_angular=kp_a*angle_error + kd_a*derivative_a

            previous_angle=angle_error
            #last_rotation=theta

            #Logica de control
            if distance > 0.17:
                if angle > 0.12:
                    speed.linear.x = 0.00
                    speed.angular.z = 0.12 #VER COMO ARREGLAR ROTATION
                elif angle < -0.12:
                    speed.linear.x = 0.00
                    speed.angular.z = -0.12
                else:
                    speed.linear.x = velocidad_lineal
                    speed.angular.z = velocidad_angular
            else:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    reached=1


            pub_vel.publish(speed)
            r.sleep()
        else:

            print("Final position reached: X_final:%f , Y_final: %f" % (x_act,y_act))
            print("Next point")
            i=i+1
            reached=0
            previous_distance=0
            previous_angle=0


#opcional
"""
    else:
        print("Searching final orientation")
        model_info_prox = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        rospy.wait_for_service('/gazebo/get_link_state')
        gt=model_info_prox("base_link", "world")
        rot_q_act=gt.link_state.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q_act.x, rot_q_act.y, rot_q_act.z, rot_q_act.w])
        print(theta_final -theta)
        if (theta_final - theta)>0.07:
            speed.linear.x = -0.01
            speed.angular.z = 0.1
        elif(theta_final - theta)< -0.07:
            speed.linear.x = -0.01
            speed.angular.z = -0.1
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            print("Final orientation reached")

        pub.publish(speed)
"""
#control alternativo
"""
if theta<0 and angle_to_goal<0:

    print("ey")
    angle_to_goal=pi-abs(angle_to_goal) +pi
    if theta<0:
        angle=-angle_to_goal + theta
    else:
        angle=angle_to_goal - theta
else:
    angle=angle_to_goal - theta
print(inc_x, inc_y)
print("angulo a corregir",angle_to_goal)
print("theta:", theta)

if inc_x<0 and inc_y<0:
    print("tercer cuadrante")
    angle_to_goal=angle_to_goal+ 2*pi
    if theta<0:
        theta=theta+2*pi
        angle=angle_to_goal - theta
    else:
        angle=angle_to_goal - theta
elif inc_x>0 and inc_y<0:
    print("cuarto cuadrante")
    #angle_to_goal=angle_to_goal+ 2*pi
    print("angulo a corregir tras suma",angle_to_goal)
    if theta<0:
        angle_to_goal=angle_to_goal+ 2*pi
        theta=theta+2*pi
        angle=angle_to_goal - theta
    else:
        angle=angle_to_goal - theta

elif inc_x<0 and inc_y>0:
    print("segundo cuadrante")
    if theta<0:
        angle=angle_to_goal - theta
    else:
        angle=angle_to_goal - theta
else:
        angle=angle_to_goal - theta

"""
