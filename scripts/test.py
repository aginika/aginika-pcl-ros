#!/usr/bin/env python
if __name__ == "__main__":
    import rospy
    import imp
    import sys

    argvs = sys.argv
    argc = len(argvs)

    rospy.init_node('leaner', anonymous=True)
    rospy.loginfo("Random Forest Before")

    f = open("/home/inagaki/test_a/a", "w");

    f.write("hello")

    f.close()

    rospy.loginfo("Random Forest After")
