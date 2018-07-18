#!/usr/bin/env python
import rospy
from hbc_msgs.msg import QrcodeStamped, SendData
import zbar
from PIL import Image

class Qrcode(object):
    def __init__(self,debug=False):
        #self.pub_sendData = rospy.Publisher('/topic_sendData', SendData, queue_size=1)
        self.sub_qrcode = rospy.Subscriber('/topic_qrcode', QrcodeStamped, self.cbQrcode, queue_size=1)

    def cbQrcode(self, qrcode_msg):
        qrcode_msg = qrcode_msg
        for i in range(10):
            uploadData = self.identify()
            if uploadData != "":
                override_msg = SendData()                
                override_msg.data = True
                override_msg.upload = uploadData
                self.pub_sendData.publish(override_msg)
                print "publish sendData Successfully!!"
        
    def identify():
        scanner = zbar.ImageScanner()
        scanner.parse_config('enable')
        while True:
            try:
                img = Image.open("/home/ubuntu/duckietown/catkin_ws/src/qrcode/src/qrcode_temp.jpeg").convert('L')
                break
            except:
                pass
        width, height = img.size
        
        qrCode = zbar.Image(width, height, 'Y800', img.tobytes())
        scanner.scan(qrCode)
        data = ''
        for s in qrCode:
            data += s.data
        if data == "":
            print "empty\n"
        else:
            print data,"\n"
            rospy.set_param("~data",data)
        return data
        
if __name__ == '__main__':
    rospy.init_node('qrcode',anonymous=False)
    qrcode = Qrcode()
    rospy.spin()