#!/usr/bin/env python
import Tkinter as tk
import ttk

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import euler_from_quaternion, quaternion_from_euler





def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)

        f = open("status.txt", "w")
        f.write("")
        f.close()

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        ## We create a `DisplayTrajectory`_ publisher which is used later to publish
        ## trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

    
        print "============ Printing robot state"


        robot_state = robot.get_current_state()
        print(robot_state)
        # print(robot.get_current_state.joint_state.name.)
        print ("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        

    def pick_up_box(self):
        global box_x_initial, box_y_initial, box_z_initial

        group = self.group
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        roll = 0 
        pitch = pi/2
        yaw = 0
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
 
        pose_goal.position.x = float(box_x_initial) - 0.2
        pose_goal.position.y = float(box_y_initial)
        pose_goal.position.z = float(box_z_initial)
        group.set_pose_target(pose_goal)

        print("Initial Position Quaternions are: ")
        print(pose_goal)
        print("Initial Position Euler Angles are: ")
        print(roll, pitch, yaw)

       

        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def place_box(self):
        global box_x_final, box_y_final, box_z_final
      
        group = self.group

        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        roll = 0 
        pitch = -pi/2
        yaw = 0
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
 
        pose_goal.position.x = float(box_x_final) + 0.1 # adding to compensate for the diff between panda_hand and box
        pose_goal.position.y = float(box_y_final) 
        pose_goal.position.z = float(box_z_final) 
        group.set_pose_target(pose_goal)

        print("Final Position Quaternions are: ")
        print(pose_goal)
        print("Final Position Euler Angles are: ")
        print(roll, pitch, yaw)

        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        x = pose_goal
        return all_close(pose_goal, current_pose, 0.01), x


    def plan_cartesian_path(self, scale=1):

        group = self.group
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through:
        waypoints = []

        wpose = group.get_current_pose().pose
        wpose.position.x += 0.18  
        wpose.position.y += 0 
        wpose.position.z += 0  
        waypoints.append(copy.deepcopy(wpose))
        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        return plan, fraction

    def display_trajectory(self, plan):
    
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

    def execute_plan(self, plan):

        group = self.group


        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        group.execute(plan, wait=True)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

        box_name = self.box_name
        scene = self.scene

        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
      
    def home_position(self):

        group = self.group

        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^

        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -0.79
        joint_goal[2] = 0
        joint_goal[3] = -2.36
        joint_goal[4] = 0
        joint_goal[5] = 1.57
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def user_home_position(self):
        global J1, J2, J3, J4, J5, J6, J7

        group = self.group
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = float(J1)
        joint_goal[1] = float(J2)
        joint_goal[2] = float(J3)
        joint_goal[3] = float(J4)
        joint_goal[4] = float(J5)
        joint_goal[5] = float(J6)
        joint_goal[6] = float(J7)

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()


        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def add_box(self, timeout=4):
        global box_x_initial, box_y_initial, box_z_initial

        box_name = self.box_name
        scene = self.scene
 
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene at the location of the left finger:
        box_pose = geometry_msgs.msg.PoseStamped()
        print("before positoion of box", box_pose)
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = float(box_x_initial)
        box_pose.pose.position.y = float(box_y_initial)
        box_pose.pose.position.z = float(box_z_initial)
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))
        print("Initial position is")
        print(box_pose)

        self.box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout), box_pose

    def attach_box(self, timeout=4):

        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        grasping_group = 'hand'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        f = open("status.txt", "w")
        f.write("Attached")
        f.close()
        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):

        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
   
        print("GET OBEJCTS BELOW:!!!!!!!!!!!!!!!!!!!!!")
        # print(scene.get_objects())
        print(scene.get_object_poses(['box']))

        f = open("status.txt", "w")
        f.write("Detached")
        f.close()
        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
   
        box_name = self.box_name
        scene = self.scene

        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


class gripper_control(object):

    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(gripper_control, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "hand"
        group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def gripper_open(self):

        group = self.group

        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0.035
        joint_goal[1] = 0.035

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def gripper_close(self):

        group = self.group

        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        ## thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0.026
        joint_goal[1] = 0.026


        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


# Creating objects of class
tutorial = MoveGroupPythonIntefaceTutorial()
gripper = gripper_control()


window = tk.Tk()

window.minsize(800, 600)
window.maxsize(800, 600)
window.configure(bg='black')

fontstyle = "Helvetica"
dark_yellow = "#E18A07"
grey = "#C0C0C0"


window.title("Robot GUI")

img_reset = tk.PhotoImage(file='reset.png')
img_play = tk.PhotoImage(file='play.png')
img_robot = tk.PhotoImage(file='robot2.png')



def insert_box():
    tutorial.add_box()

# Motion Pipeline
def play():
   
    start_time = rospy.get_time()
    try:
        tutorial.user_home_position()
    except:
        pass
    tutorial.pick_up_box()
    gripper.gripper_open()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()
    tutorial.execute_plan(cartesian_plan)
    gripper.gripper_close()
    tutorial.attach_box()
    tutorial.place_box()
    gripper.gripper_open()
    tutorial.detach_box()
    try:
        tutorial.user_home_position()
    except:
        tutorial.home_position()
    end_time = rospy.get_time()
    total_time = end_time - start_time
    total_time_rounded = round(total_time, 3)

    lbl_time = tk.Label(text=total_time_rounded,
                    bg="black", fg=dark_yellow,
                    font=(fontstyle, 13))
    lbl_time.place(x = 420, y= 490)

    return total_time

# INPUT VARIABLES

boxX_var_initial = tk.StringVar()
boxY_var_initial = tk.StringVar()
boxZ_var_initial = tk.StringVar()

boxX_var_final = tk.StringVar()
boxY_var_final = tk.StringVar()
boxZ_var_final = tk.StringVar()

joint1_var = tk.StringVar()
joint2_var = tk.StringVar()
joint3_var = tk.StringVar()
joint4_var = tk.StringVar()
joint5_var = tk.StringVar()
joint6_var = tk.StringVar()
joint7_var = tk.StringVar()

box_x_initial = ""
box_y_initial = ""
box_z_initial = ""

box_x_final = ""
box_y_final = ""
box_z_final = ""

J1 = ""
J2 = ""
J3 = ""
J4 = ""
J5 = ""
J6 = ""
J7 = ""

# HEADING
title = tk.Label(text="Robot GUI",
                    bg="black", fg="white",
                    font=(fontstyle, 30))
title.place(x=350, y=30)

btn_title = tk.Button(window, image = img_robot, bg="black", activebackground = "black" )
btn_title.place(x=280, y=30)


#####################3######## BOX #################################

# BOX LABELS
label_box_1 = tk.Label(text="Initial Box Position",
                    bg="black", fg=dark_yellow,
                    font=(fontstyle, 15))
label_box_1.place(x=30, y=120)


label_box_4 = tk.Label(text="Final Box Position",
                    bg="black", fg=dark_yellow,
                    font=(fontstyle, 15))
label_box_4.place(x=30, y=180)

# Robot Labels

label_box_5 = tk.Label(text="Set Home Position",
                    bg="black", fg=dark_yellow,
                    font=(fontstyle, 15))
label_box_5.place(x=30, y=270)

# INFO Labels


label_boxposition = tk.Label(text="Box position",
                    bg="black", fg=dark_yellow,
                    font=(fontstyle, 13))
label_boxposition.place(x=640, y=80)

label_jointlimits = tk.Label(text="Joint limits(rad)",
                    bg="black", fg=dark_yellow,
                    font=(fontstyle, 13))
label_jointlimits.place(x=625, y=225)


label_limits_J1357 = tk.Label(text="J1,J3,J5,J7 :  [-2.8 , 2.8]\n                   J2 :  [-1.75 , 1.75] \n             J4 :  [-3.05 , -0.98]\n             J6 :  [0.01 , 3.74] ",
                    bg="black", fg=grey,
                    font=(fontstyle, 11))
label_limits_J1357.place(x=580, y=280)

label_display_time = tk.Label(text="Process Time(secs): ",
                    bg="black", fg=grey,
                    font=(fontstyle, 13))
label_display_time.place(x=250, y=490)



# X
label_box_x_initial_initial = tk.Label(text="X:",
                    bg="black", fg=grey,
                    font=(fontstyle, 13))
label_box_x_initial_initial.place(x=200, y=120)

label_box_x_initial_final = tk.Label(text="X:",
                    bg="black", fg=grey,
                    font=(fontstyle, 13))
label_box_x_initial_final.place(x=200, y=180)



input_boxX_initial = tk.Entry(window, textvariable=boxX_var_initial, bg="dark grey", 
                    font=(fontstyle, 12), fg="black")
input_boxX_initial.place(x=230, y=124, width = 45, height = 25)

input_boxX_final = tk.Entry(window, textvariable=boxX_var_final, bg="dark grey", 
                    font=(fontstyle, 12), fg="black")
input_boxX_final.place(x=230, y=184, width = 45, height = 25)

# Y
label_box_y_initial_initial = tk.Label(text="Y:",
                    bg="black", fg=grey,
                    font=(fontstyle, 13))
label_box_y_initial_initial.place(x=300, y=120)

label_box_y_initial_final = tk.Label(text="Y:",
                    bg="black", fg=grey,
                    font=(fontstyle, 13))
label_box_y_initial_final.place(x=300, y=180)


input_boxY_initial = tk.Entry(window, textvariable=boxY_var_initial, bg="dark grey", 
                    font=(fontstyle, 12), fg="black")
input_boxY_initial.place(x=330, y=124, width = 45, height = 25)

input_boxY_final = tk.Entry(window, textvariable=boxY_var_final, bg="dark grey", 
                    font=(fontstyle, 12), fg="black")
input_boxY_final.place(x=330, y=184, width = 45, height = 25)

# Z

label_box_z_initial_initial = tk.Label(text="Z:",
                    bg="black", fg=grey,
                    font=(fontstyle, 13))
label_box_z_initial_initial.place(x=400, y=120)

label_box_z_initial_final = tk.Label(text="Z:",
                    bg="black", fg=grey,
                    font=(fontstyle, 13))
label_box_z_initial_final.place(x=400, y=180)


input_boxZ_initial = tk.Entry(window, textvariable=boxZ_var_initial, bg="dark grey", 
                    font=(fontstyle, 12), fg="black")
input_boxZ_initial.place(x=430, y=124, width = 45, height = 25)

input_boxZ_final = tk.Entry(window, textvariable=boxZ_var_final, bg="dark grey", 
                    font=(fontstyle, 12), fg="black")
input_boxZ_final.place(x=430, y=184, width = 45, height = 25)

# Joint 1

label_robot_joint1 = tk.Label(text="J1:",
                    bg="black", fg=grey,
                    font=(fontstyle, 13))
label_robot_joint1.place(x=200, y=270)

input_robot_joint1 = tk.Entry(window, textvariable=joint1_var, bg="dark grey", 
                    font=(fontstyle, 12), fg="black")
input_robot_joint1.place(x=230, y=274, width = 45, height = 25)

# Joint 2

label_robot_joint2 = tk.Label(text="J2:",
                    bg="black", fg=grey,
                    font=(fontstyle, 13))
label_robot_joint2.place(x=300, y=270)

input_robot_joint2 = tk.Entry(window, textvariable=joint2_var, bg="dark grey", 
                    font=(fontstyle, 12), fg="black")
input_robot_joint2.place(x=330, y=274, width = 45, height = 25)

# Joint 3

label_robot_joint3 = tk.Label(text="J3:",
                    bg="black", fg=grey,
                    font=(fontstyle, 13))
label_robot_joint3.place(x=400, y=270)

input_robot_joint3 = tk.Entry(window, textvariable=joint3_var, bg="dark grey", 
                    font=(fontstyle, 12), fg="black")
input_robot_joint3.place(x=430, y=274, width = 45, height = 25)

# Joint 4

label_robot_joint4 = tk.Label(text="J4:",
                    bg="black", fg=grey,
                    font=(fontstyle, 13))
label_robot_joint4.place(x=500, y=270)

input_robot_joint4 = tk.Entry(window, textvariable=joint4_var, bg="dark grey", 
                    font=(fontstyle, 12), fg="black")
input_robot_joint4.place(x=530, y=274, width = 45, height = 25)


# Joint 5

label_robot_joint5 = tk.Label(text="J5:",
                    bg="black", fg=grey,
                    font=(fontstyle, 13))
label_robot_joint5.place(x=200, y=330)

input_robot_joint5 = tk.Entry(window, textvariable=joint5_var, bg="dark grey", 
                    font=(fontstyle, 12), fg="black")
input_robot_joint5.place(x=230, y=334, width = 45, height = 25)

# Joint 6

label_robot_joint6 = tk.Label(text="J6:",
                    bg="black", fg=grey,
                    font=(fontstyle, 13))
label_robot_joint6.place(x=300, y=330)

input_robot_joint6 = tk.Entry(window, textvariable=joint6_var, bg="dark grey", 
                    font=(fontstyle, 12), fg="black")
input_robot_joint6.place(x=330, y=334, width = 45, height = 25)


# Joint 7

label_robot_joint7 = tk.Label(text="J7:",
                    bg="black", fg=grey,
                    font=(fontstyle, 13))
label_robot_joint7.place(x=400, y=330)

input_robot_joint7 = tk.Entry(window, textvariable=joint7_var, bg="dark grey", 
                    font=(fontstyle, 12), fg="black")
input_robot_joint7.place(x=430, y=334, width = 45, height = 25)


# FUNCTIONS TO PRINT BOX POSITIONS
def set_box_initial():
    global boxX_var_initial, boxY_var_initial, boxZ_var_initial
    global box_x_initial, box_y_initial, box_z_initial

    box_x_initial = boxX_var_initial.get()
    box_y_initial = boxY_var_initial.get()
    box_z_initial = boxZ_var_initial.get()

    label_box_3 = tk.Label(window, text= ("(",float(box_x_initial),"," ,float(box_y_initial), ",",float(box_z_initial),")"),
                bg="black", fg=grey,
                font=(fontstyle, 11))
    label_box_3.place(x=635, y=127)

    tutorial.add_box()


def set_box_final():
    global boxX_var_final, boxY_var_final, boxZ_var_final
    global box_x_final, box_y_final, box_z_final

    box_x_final = boxX_var_final.get()
    box_y_final = boxY_var_final.get()
    box_z_final = boxZ_var_final.get()

    label_box_6 = tk.Label(window, text= ("(",float(box_x_final),"," ,float(box_y_final), ",",float(box_z_final),")"),
                bg="black", fg=grey,
                font=(fontstyle, 11))
    label_box_6.place(x=635, y=179)


def set_robot_joints():
    global joint1_var, joint2_var, joint3_var, joint4_var, joint5_var, joint6_var, joint7_var
    global J1, J2, J3, J4, J5, J6, J7

    J1 = joint1_var.get()
    J2 = joint2_var.get()
    J3 = joint3_var.get()
    J4 = joint4_var.get()
    J5 = joint5_var.get()
    J6 = joint6_var.get()
    J7 = joint7_var.get()

    tutorial.user_home_position()

def reset_scene():
    tutorial.home_position()
    tutorial.remove_box()
    

######################3 BUTTONS ############################

# SET BUTTONS
btn_setBox_initial = tk.Button(window, command = set_box_initial, width = 5 , height = 1,
                     bg="black", fg="white" , borderwidth=1, text="SET")
btn_setBox_initial.place(x=520, y=120)


btn_setBox_final = tk.Button(window, command = set_box_final, width = 5 , height = 1,
                     bg="black", fg="white" , borderwidth=1, text="SET")
btn_setBox_final.place(x=520, y=180)


btn_setrobot_joints = tk.Button(window, command = set_robot_joints, width = 5 , height = 1,
                     bg="black", fg="white" , borderwidth=1, text="SET")
btn_setrobot_joints.place(x=520, y=330)

btn_play = tk.Button(window, command = play, image=img_play,
                     bg="black", borderwidth=0, activebackground="green", highlightbackground = "black")
btn_play.place(x = 330, y= 410)


btn_reset_robot = tk.Button(window, command = reset_scene, image=img_reset,
                     bg="black", borderwidth=0, activebackground="white", highlightbackground = "black")
btn_reset_robot.place(x=410, y= 415 )


window.mainloop()
 