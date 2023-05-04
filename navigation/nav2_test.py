#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def main():
    # ---- Init
    rclpy.init()
    nav = BasicNavigator()

    # ---- set initial pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)
    # ---- wait for Nav2
    nav.waitUntilNav2Active()
 
    # --- Send nav2goal
    goal_pose = create_pose_stamped(nav, 3.5, 1.0, 1.57)
    nav.goToPose(goal_pose)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)

    # ---- Follow waypoints

    goal_pose_1 = create_pose_stamped(nav, 3.5, 1.0, 1.57)
    goal_pose_2 = create_pose_stamped(nav, 3.5, 1.0, 1.57)
    waypoints = [goal_pose_1, goal_pose_2]
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)

    # --- shutdown
    rclpy.shutdown()

def create_pose_stamped(nav : BasicNavigator, position_x: float,
                        position_y : float, 
                        orientation_z : float) -> PoseStamped:
    """ Create pose for a specific position """
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0,0,0,
                                                                  orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

if __name__=='__main__':
    main()
