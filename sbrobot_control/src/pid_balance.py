#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math as m

class controller:
    def __init__(self):
        rospy.Subscriber("imu", Imu, self.icallback)
        rospy.Subscriber("joint_states", JointState, self.jcallback)
        self.lspdpub = rospy.Publisher("left_motor_controller/command",Float64,queue_size = 20)
        self.rspdpub = rospy.Publisher("right_motor_controller/command",Float64,queue_size = 20)
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.axdot = 0.0
        self.aydot = 0.0
        self.azdot = 0.0
        self.sumay = 0.0
        self.lvm = 0.0
        self.rvm = 0.0

    def icallback(self,msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        ang = 2*m.acos(w)
        self.ax = ang*(x)/m.sqrt(1-w*w)
        self.ay = ang*(y)/m.sqrt(1-w*w)
        self.az = ang*(z)/m.sqrt(1-w*w)
        self.sumay += self.ay/50.0
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
            spd = kp*(self.ay) + kd*(self.aydot) + ki*(self.sumay)
            ls = Float64()
            rs = Float64()
            print(spd,self.lvm,self.rvm)
            ls.data = spd
            rs.data = spd
            self.lspdpub.publish(ls)
            self.rspdpub.publish(rs)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pidcontroller', anonymous=True)
    a = controller()
    try:
        a.listener()
    except(KeyboardInterrupt):
        print("Shutting Down")
        exit()