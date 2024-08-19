# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
ros2 run keyboard_control keyboard_control --ros-args -p vessel_name:=<name of vessel to control> -p thrusters_name:=[PSPropRudd, SBPropRudd]
"""

import sys
import threading

import liquidai_msgs.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j         l

anything else : stop

q/z : increase/decrease max speeds by 10%

CTRL-C to quit
"""

moveBindings = {
    'u': (30.0),
    'i': (0.0),
    'o': (-30.0),
    'j': (-90.0),
    'l': (90.0),
}

speedBindings = {
    'q': (10.0),
    'z': (-10.0),
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(rpm):
    return 'currently:\trpm %s ' % (rpm)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    # parameters
    frame_id = node.declare_parameter('frame_id', '').value
    vessel_name = node.declare_parameter('vessel_name', '').value
    node.declare_parameter('thrusters_name', ['propeller'])
    thrusters_name = node.get_parameter('thrusters_name').get_parameter_value().string_array_value

    pubMsgType = liquidai_msgs.msg.Xdyncmd
    pubMsg = pubMsgType()
    pubMsg.vessel_name = vessel_name
    
    for prop in thrusters_name:
        thruster = liquidai_msgs.msg.XdynThrustercmd()
        thruster.name = prop
        thruster.rpm = 0.01
        thruster.pd = 0.79
        thruster.beta = 0.0
        pubMsg.cmd.append(thruster)
    
    pub = node.create_publisher(pubMsgType, vessel_name+'/cmd_vel', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    rpm = 20.0
    status = 0.0

    print(vels(rpm))
    
    try:
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                beta = moveBindings[key]
            elif key in speedBindings.keys():
                rpm = rpm + speedBindings[key]
                if rpm<0:
                    rpm =0.0
                print(vels(rpm))
       
            else:
                if (key == '\x03'):
                    break

            for msg in pubMsg.cmd:
                msg.rpm = rpm
                msg.beta = beta
            pub.publish(pubMsg)

    except Exception as e:
        print(e)

    finally:
        for msg in pubMsg.cmd:
            msg.rpm = 0.0
            msg.beta = 0.0
        pub.publish(pubMsg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()