#!/usr/bin/python2.7
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

# Global variable to store the current position of the robot
current_pose = None
path_points = []

def amcl_pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose

def send_goal(position_x, position_y, orientation_z, orientation_w, marker_pub):
    global current_pose

    # Create a publisher for the /move_base_simple/goal topic
    goal_publisher = rospy.Publisher('/tb3_2/move_base_simple/goal', PoseStamped, queue_size=10)

    # Create a subscriber for the estimated position topic of the robot
    rospy.Subscriber('/tb3_2/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)

    # Wait for the publisher to be ready
    rospy.sleep(1)

    # Create a PoseStamped message for the specified position and orientation
    goal_msg = PoseStamped()
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = 'map'
    goal_msg.pose.position.x = position_x
    goal_msg.pose.position.y = position_y
    goal_msg.pose.position.z = 0.0
    goal_msg.pose.orientation.x = 0.0
    goal_msg.pose.orientation.y = 0.0
    goal_msg.pose.orientation.z = 0.0
    goal_msg.pose.orientation.w = 1.0

    # Publish the message on the /move_base_simple/goal topic
    goal_publisher.publish(goal_msg)

    # Display a confirmation message
    rospy.loginfo("Goal has been sent: ({}, {})".format(position_x, position_y))

    # Wait for the robot to reach the desired position
    while not rospy.is_shutdown():
        if current_pose is not None:
            # Add the robot's current position to the path points list
            path_points.append([current_pose.position.x, current_pose.position.y])
            # Publish the path (points) as a marker
            publish_path(marker_pub)

            # Calculate the distance to the goal
            distance_to_goal = ((current_pose.position.x - position_x) ** 2 +
                                (current_pose.position.y - position_y) ** 2) ** 0.5
            if distance_to_goal < 0.1:  # Tolerance of 0.1 meters
                print("Goal was reached, breaking")
                break
        rospy.sleep(1)

def publish_path(marker_pub):
    # Create a new Marker message for displaying points
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "robot_trajectory"
    marker.id = 1  # Fixed ID for trajectory visualization
    marker.type = Marker.SPHERE_LIST  # Use SPHERE_LIST for multiple points
    marker.action = Marker.ADD  # Add points to the marker

    # Set the color and scale for the points (small spheres)
    marker.scale.x = 0.1  # Sphere radius
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color = ColorRGBA(1.0, 0.5, 0.0, 1.0)  # Orange color for the points

    # Add all points to the marker
    for point in path_points:
        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = 0.1  # Slightly above ground
        marker.points.append(p)

    # Publish the path marker
    marker_pub.publish(marker)

def main():
    # Initialize the ROS node
    rospy.init_node('send_goal_node', anonymous=True)

    # Create a publisher for markers (trajectory visualization)
    marker_pub = rospy.Publisher('/tb3_2/trajectory_marker', Marker, queue_size=10)

    # Wait for the publisher to be ready
    rospy.sleep(1)

    try:
        # Call the send_goal function with the desired positions
        #send_goal(14, 8.5, 0.0, 0.0, marker_pub)
        #rospy.sleep(1)  # Wait 5 seconds after reaching the first position
        #print("Time to wake up")

        send_goal(17.5, 3.7, 0.0, 0.0, marker_pub)
        rospy.sleep(1)  # Wait 5 seconds after reaching the first position

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
