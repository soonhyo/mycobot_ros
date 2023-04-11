import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('end_effector_listener')
    listener = tf.TransformListener()
    source = "/link1"
    target = "/link7"

    # Wait for the transformation between the robot's end effector and the base frame
    listener.waitForTransform(source, target, rospy.Time(), rospy.Duration(4.0))

    while not rospy.is_shutdown():
        try:
            # Get the transformation from the end effector frame to the base frame
            (trans, rot) = listener.lookupTransform(source, target, rospy.Time(0))

            # Print the translation vector (i.e. the coordinates of the end effector in the base frame)
            print(f"End effector coordinates: x = {trans[0]}, y = {trans[1]}, z = {trans[2]}")
            print(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
