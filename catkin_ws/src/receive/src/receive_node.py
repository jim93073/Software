#!/usr/bin/env python
import rospy
from hbc_msgs.msg import ReceiveStamped
import requests



class Receive(object):
    def __init__(self,debug=False):
        sub_receive = rospy.Subscriber('/topic_receive', ReceiveStamped, self.cbReceive, queue_size=1)

    def cbReceive(self, receive_msg):
        self.receive_msg = receive_msg
        sendData = {'data1':self.receive_msg.data1,'data2':self.receive_msg.data2,'token':'7XWE32L52T03DH61'}
        r = requests.get("http://coursesrv.nutn.edu.tw/S10582015/receive.php",params=sendData)
        
if __name__ == '__main__':
    rospy.init_node('receive',anonymous=False)
    receive = Receive()
    rospy.spin()