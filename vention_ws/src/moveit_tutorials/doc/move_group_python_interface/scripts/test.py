#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import tf
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
from move_group_python_interface_tutorial import MoveGroupPythonIntefaceTutorial

box = MoveGroupPythonIntefaceTutorial()


def talker():
    
    pub = rospy.Publisher('BoxPose', String, queue_size=10)
    # boxpose = rospy.Publisher('chatter', Pose, queue_size=10)
    

    # pub_info = rospy.Publisher('chatter', String, queue_size=10 )

    rate = rospy.Rate(2) 
    time.sleep(2)

    while not rospy.is_shutdown():
        
        (trans,rot) = listener.lookupTransform('world', 'panda_hand', rospy.Time(0))
        pos_x = trans[0] + 0.1
        pos_y = trans[1]
        pos_z = trans[2]

        ori_x = rot[0]
        ori_y = rot[1]
        ori_z = rot[2]
        ori_w = rot[3]

        try:
            box_pose_inital = box.scene.get_object_poses(['box'])
            a= ("%s"%box_pose_inital['box']).replace("\n", " ")
            pub.publish(a.replace("  "," "))
        except:
            pass 

        posi_str = "position:  x: %.13f  y: %.13f  z: %.13f " % (pos_x,pos_y,pos_z) 
               
        ori_str = "orientation:  x: %.13f  y: %.13f  z: %.13f  w: %.13f" % (ori_x,ori_y,ori_z,ori_w)
    
        f = open("status.txt","r")
    
        if(f.read()=="Attached"):
            pub.publish(posi_str+ori_str) 

        rate.sleep()

if __name__ == '__main__':
  
    listener = tf.TransformListener()
    try:
        talker()
    except (rospy.ROSInterruptException, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass




    
