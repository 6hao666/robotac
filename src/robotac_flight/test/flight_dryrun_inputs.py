#!/usr/bin/env python3
"""Publish deterministic fake MAVROS/FAST-LIO inputs for an offline dry-run."""

import rospy
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def main():
    rospy.init_node("robotac_flight_dryrun_inputs")
    vision_output_enabled = bool(rospy.get_param("~vision_output_enabled", False))
    rate = rospy.Rate(20.0)
    odom_pub = rospy.Publisher("/Odometry", Odometry, queue_size=2)
    local_pub = rospy.Publisher("/mavros/local_position/odom", Odometry, queue_size=2)
    state_pub = rospy.Publisher("/mavros/state", State, queue_size=2)
    extended_pub = rospy.Publisher("/mavros/extended_state", ExtendedState, queue_size=2)
    estimator_pub = rospy.Publisher("/mavros/estimator_status", EstimatorStatus, queue_size=2)
    vision_pub = rospy.Publisher("/robotac/fastlio_vision/healthy", Bool, queue_size=1, latch=True)
    output_pub = rospy.Publisher("/robotac/fastlio_vision/output_enabled", Bool, queue_size=1, latch=True)
    rospy.sleep(1.0)
    vision_pub.publish(Bool(data=True))
    output_pub.publish(Bool(data=vision_output_enabled))
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "camera_init"
        odom.child_frame_id = "body"
        odom.pose.pose = Pose(position=Point(), orientation=Quaternion(w=1.0))
        odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[14] = 0.01
        odom.pose.covariance[21] = odom.pose.covariance[28] = odom.pose.covariance[35] = 0.01
        odom_pub.publish(odom)
        local = Odometry()
        local.header.stamp = now
        local.header.frame_id = "map"
        local.child_frame_id = "base_link"
        local.pose.pose = Pose(position=Point(), orientation=Quaternion(w=1.0))
        local_pub.publish(local)

        state = State(connected=True, armed=False, mode="STABILIZED")
        state_pub.publish(state)
        extended_pub.publish(ExtendedState(landed_state=ExtendedState.LANDED_STATE_ON_GROUND))
        estimator_pub.publish(EstimatorStatus(
            attitude_status_flag=True,
            pos_horiz_rel_status_flag=True,
            # Deliberately omit absolute vertical position: local flight only
            # needs a valid relative/AGL vertical estimate.
            pos_vert_agl_status_flag=True))
        rate.sleep()


if __name__ == "__main__":
    main()
