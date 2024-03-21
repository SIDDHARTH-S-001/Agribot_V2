import rospy
from sensor_msgs.msg import JointState
import json
import serial

ser = serial.Serial("/dev/ttyUSB0", 115200)


class MinimalSubscriber:

    def __init__(self):
        self.position = []

        rospy.init_node('serial_ctrl', anonymous=True)

        rospy.Subscriber('joint_states', JointState, self.listener_callback)

    def posGet(self, radInput, direcInput, multiInput):
        if radInput == 0:
            return 2047
        else:
            getPos = int(2047 + (direcInput * radInput / 3.1415926 * 2048 * multiInput) + 0.5)
            return getPos

    def listener_callback(self, msg):
        a = msg.position

        # Only Robotic Arm
        join1 = self.posGet(a[0], -1, 1)
        join2 = self.posGet(a[1], -1, 3)
        join3 = self.posGet(a[2], -1, 1)
        join4 = self.posGet(a[3], 1, 1)
        join5 = self.posGet(a[4], -1, 1)

        # Mobile Platform + Robotic Arm
        # join1 = self.posGet(a[4], -1, 1)
        # join2 = self.posGet(a[5], -1, 3)
        # join3 = self.posGet(a[6], -1, 1)
        # join4 = self.posGet(a[7], 1, 1)
        # join5 = self.posGet(a[8], -1, 1)

        data = json.dumps({'T': 3, 'P1': join1, 'P2': join2, 'P3': join3, 'P4': join4, 'P5': join5, 'S1': 0, 'S2': 0,
                           'S3': 0, 'S4': 0, 'S5': 0, 'A1': 60, 'A2': 60, 'A3': 60, 'A4': 60, 'A5': 60})

        ser.write(data.encode())

        rospy.loginfo(data)
        

def main():
    minimal_subscriber = MinimalSubscriber()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
