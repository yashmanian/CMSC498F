#!/usr/bin/env python
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation
from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy
from std_msgs.msg import Bool

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

GRIPPER_FRAME = 'gripper_servo_link'

GRIPPER_OPENED = [0.053]
GRIPPER_CLOSED = [0.001]
GRIPPER_NEUTRAL = [0.028]
GRASP_OVERTIGHTEN = 0.002

GRIPPER_JOINT_NAMES = ['gripper_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = '/base_link'


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_demo')
gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)
arm = MoveGroupCommander(GROUP_NAME_ARM)
gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

end_effector_link = arm.get_end_effector_link()

arm.set_goal_position_tolerance(0.04)
arm.set_goal_orientation_tolerance(0.1)

arm.allow_replanning(True)
arm.set_pose_reference_frame(REFERENCE_FRAME)

arm.set_planning_time(5)

rospy.sleep(2)

place_pose = PoseStamped()
place_pose.header.frame_id = REFERENCE_FRAME
place_pose.pose.position.x = 0.01
place_pose.pose.position.y = 0.01
place_pose.pose.position.z = 0.01
place_pose.pose.orientation.w = 1.0


def gripper_callback(message):
    if message.data:
        print 'Gripper Received!!!!!'
        arm.set_named_target('pos1')
        arm.go()
        print 'After right up'
        rospy.sleep(3)
        arm.set_named_target('pos2')
        arm.go()
        print 'After pos2'
        rospy.sleep(3)
        arm.set_named_target('pos3')
        arm.go()
        print 'After pos3'
        rospy.sleep(3)
        arm.set_named_target('pos1')
        arm.go()
        print 'After pos1'
        rospy.sleep(3)
        global gripper_flag_pub
        print 'Before publish to flag'
        gripper_flag_pub.publish(Bool(True))
        print 'After publish to flag'

if __name__ == "__main__":
    # rospy.init_node('gripper_knock_down', anonymous=False)
    rospy.Subscriber('gripper_arguments', Bool, gripper_callback)
    gripper_flag_pub = rospy.Publisher('gripper_flag', Bool)
    rospy.spin()