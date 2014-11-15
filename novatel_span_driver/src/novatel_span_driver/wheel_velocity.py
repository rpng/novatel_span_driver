#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
#  file      @wheel_velocity.py
#  authors   Mike Purvis <mpurvis@clearpathrobotics.com>
#            NovAtel <novatel.com/support>
#  copyright Copyright (c) 2012, Clearpath Robotics, Inc., All rights reserved.
#            Copyright (c) 2014, NovAtel Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#    following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#    following disclaimer in the documentation and/or other materials provided with the distribution.
#  * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
# RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
# DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
# OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
from nav_msgs.msg import Odometry
from math import pi


class NovatelWheelVelocity(object):
    """ Subscribes to a platform's odom topic and sends SPAN
        wheelvelocity messages about the platform's linear movement. """

    def __init__(self, port):
        self.port = port

        # The Odometry message doesn't expose this information to us, so we
        # make up a fake wheel which is rotated and ticked based on how far
        # the odom topic tells us we traveled.
        self.fake_wheel_diameter = rospy.get_param("~fake_wheel/diameter", 0.33)
        self.fake_wheel_ticks = rospy.get_param("~fake_wheel/ticks", 1000)

        # SPAN wants to know how much delay is associated with our velocity report.
        # This is specified in milliseconds.
        self.latency = rospy.get_param("~wheel_velocity_latency", 100)

        # Send setup command.
        self.circumference = self.fake_wheel_diameter * pi
        cmd = 'setwheelparameters %d %f %f' % (
            self.fake_wheel_ticks,
            self.circumference,
            self.circumference / self.fake_wheel_ticks)

        rospy.logdebug("Sending: %s" % cmd)

        self.cumulative_ticks = 0
        rospy.Subscriber('odom', Odometry, self.odom_handler)

    def odom_handler(self, odom):
        # Robot's linear velocity in m/s.
        velocity = abs(odom.twist.twist.linear.x)
        self.cumulative_ticks += velocity * self.fake_wheel_ticks / self.circumference

        cmd = 'wheelvelocity %d %d %d 0 %f 0 0 %d \r\n' % (
            self.latency,
            self.fake_wheel_ticks,
            int(velocity),
            velocity,
            self.cumulative_ticks)

        rospy.logdebug("Sending: %s" % cmd)

        self.port.send(cmd)