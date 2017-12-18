#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray

import math
import copy

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber( '/current_velocity', TwistStamped, self.velocity_cb )

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_wps = None
        self.traffic_wp = -1
        self.velocity = 0

        v = rospy.get_param('/waypoint_loader/velocity', 15. )
        self.vel_max = ( v * 1000. ) / ( 60. * 60. )

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        # pos = msg.pose.position
        # ort = msg.pose.orientation
        # rospy.loginfo( "[waypoint_updater.pose_cb] position = (%f, %f, %f)", \
        #     pos.x, pos.y, pos.z )
        # rospy.loginfo( "[waypoint_updater.pose_cb] orientation = (%f, %f, %f, %f)", \
            # ort.x, ort.y, ort.z, ort.w )
        if self.base_wps == None:
            rospy.loginfo( "[waypoint_updater.pose_cb] No base_waypoints." )
            return

        cpos = msg.pose.position
        cort = msg.pose.orientation

        near_i = self.get_closest_waypoint( cpos ) + 1
        num_wps = len( self.base_wps.waypoints )

        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)

        # self.traffic_wp = -1
        if self.traffic_wp == -1:
            if near_i + LOOKAHEAD_WPS > num_wps:
                lane.waypoints = self.base_wps.waypoints[ near_i : ] + \
                            self.base_wps.waypoints[ : near_i + LOOKAHEAD_WPS - num_wps ]
            else:
                lane.waypoints = self.base_wps.waypoints[ near_i : near_i + LOOKAHEAD_WPS ]
        elif self.traffic_wp >= near_i:
            lane.waypoints = self.base_wps.waypoints[ near_i : self.traffic_wp + 1 ]
        elif near_i - self.traffic_wp <= 10:
            lane.waypoints = self.base_wps.waypoints[ near_i : near_i + 1 ]
        else:
            lane.waypoints = self.base_wps.waypoints[ near_i : ] + \
                                self.base_wps.waypoints[ : self.traffic_wp + 1 ]

        # Set velocities
        if self.traffic_wp == -1:
            self.accelerate( lane.waypoints, near_i )
        else:
            self.decelerate( lane.waypoints, near_i )

        # rospy.loginfo( "[waypoint_updater ===>] car = %d, red = %d", near_i, self.traffic_wp )
        self.final_waypoints_pub.publish( lane )

    def velocity_cb( self, msg ):
        self.velocity = msg.twist.linear.x

    def get_closest_waypoint( self, pos ):
        if self.base_wps == None:
            return None

        dist = float( "inf" )
        wpi = None
        for i, wp in enumerate( self.base_wps.waypoints ):
            p = wp.pose.pose.position
            d = math.sqrt( ( p.x - pos.x ) ** 2 + ( p.y - pos.y ) ** 2 + ( p.z - pos.z ) ** 2 )
            if d < dist:
                wpi = i
                dist = d

        return wpi

    def waypoints_cb(self, lane):
        # TODO: Implement
        self.base_wps = lane

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        if self.traffic_wp != msg.data:
            rospy.loginfo( "[waypoint_updater.traffic_cb] traffic_wp = %d", msg.data )
        self.traffic_wp = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def _distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance(self, waypoints, wp1, wp2):
        num = len( waypoints )
        wp1, wp2 = wp1 % num, wp2 % num
        if wp2 > wp1:
            return self._distance( waypoints, wp1, wp2 )
        else:
            num_wps = len( self.base_wps.waypoints )
            return self._distance( waypoints, wp1, num_wps - 1 ) + \
                    self._distance( waypoints, 0, wp2 )

    def accelerate( self, waypoints, near_i ):
        for wp in waypoints:
            wp.twist.twist.linear.x = self.vel_max

    def decelerate( self, waypoints, near_i ):
        MAX_DECEL = 0.5
        MAX_ACCEL = 1.0

        last = waypoints[-1]
        last.twist.twist.linear.x = 0.

        num = len( waypoints )

        if num > 1:
            d0 = self.distance( self.base_wps.waypoints, near_i, near_i + 1 )
            meet = False
            for i in range( num - 1 ):
                dist = self.distance( waypoints, i, num - 1 )
                vel = math.sqrt(2 * MAX_DECEL * dist)
                if vel < 1.:
                    vel = 0.
                vel = min( vel, self.vel_max )

                waypoints[ i ].twist.twist.linear.x = vel

                # if not meet:
                #     dist2 = self.distance( waypoints, 0, i ) + d0
                #     vel2 = math.sqrt( self.velocity ** 2 + 2 * MAX_ACCEL * dist2 )
                #     vel2 = min( vel2, self.vel_max )

                #     meet = vel2 >= vel
                #     waypoints[ i ].twist.twist.linear.x = min( vel, vel2 )
                # else:
                #     waypoints[ i ].twist.twist.linear.x = vel

        return waypoints


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
