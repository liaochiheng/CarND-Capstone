from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import math
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        # self.steer_pid = PID( -1.0987, -0.0047, -7.4110, mn = -0.52, mx = 0.52 )
        wheel_base 		= kwargs[ 'wheel_base' ]
        steer_ratio 	= kwargs[ 'steer_ratio' ]
        max_lat_accel 	= kwargs[ 'max_lat_accel' ]
        max_steer_angle = kwargs[ 'max_steer_angle' ]

        self.vehicle_mass 	= kwargs[ 'vehicle_mass' ]
        self.fuel_capacity 	= kwargs[ 'fuel_capacity' ]
        self.wheel_radius 	= kwargs[ 'wheel_radius' ]
        self.decel_limit 	= kwargs[ 'decel_limit' ]
        self.accel_limit 	= kwargs[ 'accel_limit' ]
        self.brake_deadband = kwargs[ 'brake_deadband' ]

        self.last_cmd = None

        min_speed = 0.0
        self.yaw_controller = YawController( wheel_base, steer_ratio, min_speed, \
        	max_lat_accel, max_steer_angle )

        # self.lowpass_filter = LowPassFilter( 0.5, 0.1 )
   

    def control( self, proposed_linear_v, proposed_angular_v, current_linear_v, dbw_enable ):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        throttle = 1.
        brake = 0.
        steer = 0.

        if not dbw_enable:
        	self.last_cmd = None
        	return 0., 0., 0.

        dv = math.fabs( current_linear_v - proposed_linear_v )

    	# rospy.logwarn( "[twist_controller] proposed_v = %.2f, current_v = %.2f, dv = %.2f", \
    	# 	proposed_linear_v, current_linear_v, dv )

        if proposed_linear_v > 0 and current_linear_v > 0 and dv < 0.05: # Reach proposed velocity
        	throttle = 0.
        	brake = 0.
        	self.last_cmd = None

       #  	rospy.loginfo( "[twist_controller] == NONE === %.2f - %.2f = %.2f, throttle = %.1f, brake = %.1f", \
    			# proposed_linear_v, current_linear_v, proposed_linear_v - current_linear_v, throttle, brake )
        elif self.last_cmd and proposed_linear_v > 0 and current_linear_v > 0 and dv < 0.5:
        	throttle, brake = self.last_cmd

       #  	rospy.loginfo( "[twist_controller] == KEEP === %.2f - %.2f = %.2f, throttle = %.1f, brake = %.1f", \
    			# proposed_linear_v, current_linear_v, proposed_linear_v - current_linear_v, throttle, brake )
        elif current_linear_v < proposed_linear_v:
        	throttle = 1.
        	brake = 0.
        	self.last_cmd = [ throttle, brake ]

       #  	rospy.loginfo( "[twist_controller] == Accr === %.2f - %.2f = %.2f, throttle = %.1f, brake = %.1f", \
    			# proposed_linear_v, current_linear_v, proposed_linear_v - current_linear_v, throttle, brake )
        else:
        	throttle = 0.
        	# dv = current_linear_v - proposed_linear_v
        	brake = ( self.vehicle_mass + self.fuel_capacity * GAS_DENSITY ) \
        				* math.fabs( self.decel_limit ) * self.wheel_radius
        	self.last_cmd = [ throttle, brake ]

       #  	rospy.loginfo( "[twist_controller] == Dccr === %.2f - %.2f = %.2f, throttle = %.1f, brake = %.1f", \
    			# proposed_linear_v, current_linear_v, proposed_linear_v - current_linear_v, throttle, brake )

        steer = self.yaw_controller.get_steering( proposed_linear_v, proposed_angular_v, current_linear_v )
    
        return throttle, brake, steer
