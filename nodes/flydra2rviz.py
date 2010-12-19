#!/usr/bin/env python
"""
Forward objects from a Flydra mainbrain server to rviz,
as small spheres.

Listener code cribs heavily from Shuo's rigit_listener.py

Scott Livingston  <slivingston@caltech.edu>
2010.
"""

import roslib; roslib.load_manifest('flydra_viz')
import rospy
import ros_flydra.msg as flydra_msgs
import visualization_msgs.msg as viz_msgs
import geometry_msgs.msg as geom_msgs
import std_msgs.msg as std_msgs
import numpy as np

class Flydra2Vizmarker:
    def __init__(self, max_freq):
        self.max_freq = max_freq
        self.world_pts = np.array([])

        self.pub = rospy.Publisher("visualization_marker", viz_msgs.Marker)
        rospy.Subscriber('flydra_mainbrain_super_packets',
                         flydra_msgs.flydra_mainbrain_super_packet,
                         self.callback)

        rospy.init_node("flydra2rviz", anonymous=True)

    def callback(self, super_packet):
        packet = super_packet.packets[0] # super_packet legacy
        self.world_pts = np.array([[obj.position.x, obj.position.y, obj.position.z] for obj in packet.objects])

    def run(self):
        loop_rate = rospy.Rate(self.max_freq)
        while not rospy.is_shutdown():
            pt_count = 0
            for pt in self.world_pts:
                self.pub.publish(viz_msgs.Marker(header=roslib.msg.Header(stamp=rospy.Time.now(),
                                                                          frame_id="/my_frame"),
                                                 ns="tracked_points",
                                                 id=pt_count,
                                                 type=viz_msgs.Marker.SPHERE,
                                                 action=viz_msgs.Marker.ADD,
                                                 pose=geom_msgs.Pose(geom_msgs.Point(pt[0], pt[1], pt[2]),
                                                                     geom_msgs.Quaternion(0., 0., 0., 0.)),
                                                 scale=geom_msgs.Vector3(.25, .25, .25),
                                                 color=std_msgs.ColorRGBA(0., 1., 0., 1.),
                                                 lifetime=roslib.rostime.Duration()))
                pt_count += 1
            loop_rate.sleep()

if __name__ == "__main__":
    f2v = Flydra2Vizmarker(20)
    f2v.run()
