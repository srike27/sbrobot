#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math as m

class controller:
    def __init__(self):
        rospy.Subscriber("imu", Imu, self.callback)
        self.lspdpub = rospy.Publisher("left_motor_controller/command",Float64,queue_size = 20)
        self.rspdpub = rospy.Publisher("right_motor_controller/command",Float64,queue_size = 20)
        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        self.axdot = 0.0
        self.aydot = 0.0
        self.azdot = 0.0
        self.sumay = 0.0

    def callback(self,msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        ang = 2*m.acos(w)
        self.ax = ang*(x)/m.sqrt(1-w*w)
        self.ay = ang*(y)/m.sqrt(1-w*w)
        self.az = ang*(z)/m.sqrt(1-w*w)
        if(self.ay > 0.0001 and self.ay < -0.0001):
            self.sumay += self.ay/50
        else:
            self.sumay = 0.0
        self.axdot = msg.angular_velocity.x
        self.aydot = msg.angular_velocity.y
        self.azdot = msg.angular_velocity.z
        
    def listener(self):
        rate = rospy.Rate(50)
        kp = 300.0
        kd = 10.0
        ki = 3.0
        while(True):
            spd = kp*(self.ay) + kd*(self.aydot) + ki*(self.sumay)
            ls = Float64()
            rs = Float64()
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