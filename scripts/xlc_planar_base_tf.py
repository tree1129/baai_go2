#!/usr/bin/env python3
import rospy
import tf.transformations
import tf2_ros
from geometry_msgs.msg import TransformStamped


def main():
    rospy.init_node("xlc_planar_base_tf")
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf2_ros.TransformListener(tf_buffer)
    broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        try:
            source = tf_buffer.lookup_transform(
                "map", "body", rospy.Time(0), rospy.Duration(0.1)
            )
            q = source.transform.rotation
            _, _, yaw = tf.transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )
            # FAST-LIO body +X and Unitree Move vx are both robot-forward.
            # Adding 90 degrees makes move_base believe the robot is facing
            # sideways and produces a persistent circular correction.
            q_flat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)

            output = TransformStamped()
            output.header.stamp = rospy.Time.now()
            output.header.frame_id = "map"
            output.child_frame_id = "go2_link"
            output.transform.translation.x = source.transform.translation.x
            output.transform.translation.y = source.transform.translation.y
            output.transform.translation.z = 0.0
            output.transform.rotation.x = q_flat[0]
            output.transform.rotation.y = q_flat[1]
            output.transform.rotation.z = q_flat[2]
            output.transform.rotation.w = q_flat[3]
            broadcaster.sendTransform(output)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logwarn_throttle(2.0, "Waiting for map -> body localization TF")
        rate.sleep()


if __name__ == "__main__":
    main()
