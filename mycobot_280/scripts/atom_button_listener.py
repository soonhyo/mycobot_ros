#!/usr/bin/env python3

from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
import rospy


class AtomButtonListener:
    def __init__(self):
        self.clicked = False

    def button_cb(self, msg):
        if self.clicked == True and msg.data == False:
            try:
                resp1 = self.get_servo()
                rospy.loginfo(resp1.message)
                if resp1.message == "False":
                    resp2 = self.set_servo(True)
                else:
                    resp2 = self.set_servo(False)
            except rospy.ServiceException as e:
                rospy.logerr(e)

        self.clicked = msg.data

    def main(self):
        rospy.init_node("atom_button_listener")
        rospy.wait_for_service("/set_servo")
        rospy.wait_for_service("/get_servo")

        self.get_servo = rospy.ServiceProxy("/get_servo", Trigger)
        self.set_servo = rospy.ServiceProxy("/set_servo", SetBool)

        rospy.Subscriber("/atom_button", Bool, self.button_cb)

        rospy.spin()

if __name__ == "__main__":
    app = AtomButtonListener()
    app.main()
