#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from sbrobot_control.msg import control
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math as m

class controller:
    def __init__(self):
        rospy.Subscriber("imu", Imu, self.icallback)
        rospy.Subscriber("joint_states", JointState, self.jcallback)
        rospy.Subscriber("control", control, self.ccallback)
        self.lspdpub = rospy.Publisher("left_motor_controller/command",Float64,queue_size = 20)
        self.rspdpub = rospy.Publisher("right_motor_controller/command",Float64,queue_size = 20)
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.aw = 0.0
        self.axdot = 0.0
        self.aydot = 0.0
        self.azdot = 0.0
        self.sumerr = 0.0
        self.lvm = 0.0
        self.rvm = 0.0
        self.err = 0.0
        self.setpoint = 0.0
        self.diffvel = 0.0

    def ccallback(self,msg):
        self.setpoint = msg.set_point
        self.diffvel = msg.diff_vel

    def icallback(self,msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        self.ay = m.asin(2*(w*y - z*x))
        self.err = self.ay - self.setpoint
        self.sumerr += self.err/50.0
        self.axdot = msg.angular_velocity.x
        self.aydot = msg.angular_velocity.y
        self.azdot = msg.angular_velocity.z

    def jcallback(self,msg):
        i = 0
        for j in msg.velocity:
            if i == 0:
                self.lvm = j
            if i == 1:
                self.rvm = j
            i += 1
        
        
    def listener(self):
        rate = rospy.Rate(50)
        kp = 100.0
        kd = 0.0
        ki = 300.0
        while(True):
            spd = kp*(self.err) + kd*(self.aydot) + ki*(self.sumerr)
            ls = Float64()
            rs = Float64()
            print(self.setpoint,self.ay)
            ls.data = spd - self.diffvel
            rs.data = spd + self.diffvel
            self.lspdpub.publish(ls)
            self.rspdpub.publish(rs)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pid_balance_controller', anonymous=True)
    a = controller()
    try:
        a.listener()
    except(KeyboardInterrupt):
        print("Shutting Down")
        exit()