#!/usr/bin/env python
import rospy
from sbrobot_control.msg import control
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math as m

class controller:
    def __init__(self):
        rospy.Subscriber("joint_states", JointState, self.jcallback)
        rospy.Subscriber("cmd_vel", Twist, self.cmdcallback)
        self.controlpub = rospy.Publisher("control",control,queue_size = 20)
        self.serr = 0.0
        self.sumserr = 0.0
        self.prevserr = 0.0
        self.diffserr = 0.0
        self.ds = 0.0
        self.ss = 0.0
        self.svm = 0.0
        self.dvm = 0.0
        self.baseradius = 0.175
        self.wheelradius = 0.1
        self.setpoint = 0.0

    def cmdcallback(self,msg):
        vx = msg.linear.x
        wz = msg.angular.z
        self.ds = self.baseradius*wz/self.wheelradius
        self.ss = vx/self.wheelradius

    def jcallback(self,msg):
        i = 0
        lvm = 0.0
        rvm = 0.0
        for j in msg.velocity:
            if i == 0:
                lvm = j
            if i == 1:
                rvm = j
            i += 1
        self.svm = (lvm + rvm)/2
        self.dvm = (rvm - lvm)/2
        
        
    def listener(self):
        rate = rospy.Rate(50)

        kps = 0.005
        kis = 0.0
        kds = 0.0000001

        while(True):
            d = self.ds

            self.serr = self.ss - self.svm
            self.sumserr += self.serr/50
            self.diffserr = 50*(self.serr - self.prevserr)
            self.prevserr = self.serr
            s = kps*self.serr + kis*self.sumserr + kds*self.diffserr

            c = control()
            c.set_point = s
            c.diff_vel = d
            self.controlpub.publish(c)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pid_speed_controller', anonymous=True)
    a = controller()
    try:
        a.listener()
    except(KeyboardInterrupt):
        print("Shutting Down")
        exit()