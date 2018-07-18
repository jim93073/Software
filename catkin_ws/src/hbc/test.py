#!/usr/bin/env python
import rospy


class test():
    def __init__(self,debug=False):
        self.sub_tag = rospy.Subscriber("/hbc/apriltag_detector_node/switch
", AprilTagsWithInfos, self.tagCallback)
        print "=================================================================="
        print 
    
        
if __name__ == '__main__':
    rospy.init_node('test',anonymous=False)
    test = Test()
    rospy.spin()