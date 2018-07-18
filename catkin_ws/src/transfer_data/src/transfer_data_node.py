#!/usr/bin/env python
import rospy
from hbc_msgs.msg import TransferData
import requests

class Transferdata(object):
    def __init__(self,debug=False):
        sub_transferData = rospy.Subscriber('/topic_transfer_data', TransferData, self.cbTransferData, queue_size=1)

    def cbTransferData(self, transferData_msg):
        self.transferData_msg = transferData_msg
        sendData = {'data1':self.transferData_msg.data1,'data2':self.transferData_msg.data2,'token':'7XWE32L52T03DH61'}
        r = requests.get("http://coursesrv.nutn.edu.tw/S10582015/receive.php",params=sendData)
        print "[X button] Upload data successfully!!"
        
if __name__ == '__main__':
    rospy.init_node('transferData',anonymous=False)
    transferData = Transferdata()
    rospy.spin()