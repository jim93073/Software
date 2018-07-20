#!/usr/bin/env python
'''import rospy
import RPi.GPIO as GPIO
import dht11
import time
from hbc_msgs.msg import TransferData

class Dht11(object):
    def __init__(self,debug=False):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.cleanup()
        self.instance = dht11.DHT11(pin=4)
        self.pub_transferData = rospy.Publisher('/topic_transfer_data', TransferData, queue_size=1)
        self.sendingTemperatureHumidity()
        
    def sendingTemperatureHumidity(self):
        count = 1
        while count <= 3:
            result = self.instance.read()
            if result.is_valid():
                transferData_msg = TransferData()
                transferData_msg.data = True
                sysTime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                transferData_msg.time = sysTime
                transferData_msg.data1 = "data"
                temperatureStr = "%d C" % result.temperature
                humidityStr = ", %d %%" % result.humidity
                tempStr = temperatureStr + humidityStr
                transferData_msg.data2 = tempStr
                print "Runnung %d/3" % count
                print("Temperature: %d C" % result.temperature)
                print("Humidity: %d %%" % result.humidity)
                count += 1
                time.sleep(5)
            time.sleep(1)
        
if __name__ == '__main__':
    rospy.init_node('dht11',anonymous=False)
    dht11 = Dht11()
    rospy.spin()'''
import RPi.GPIO as GPIO
import dht11
import time
import requests

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()

instance = dht11.DHT11(pin=4)
print "start to upload temperature and humidity"
count = 1
while count <= 3:
    result = instance.read()
    if result.is_valid():
        print "Runnung %d/3" % count
        print "Temperature: %d C" % result.temperature
        print "Humidity: %d %%" % result.humidity
        sysTime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        sendData = {'datatype':"data",'temperature':result.temperature,'humidity':result.humidity,'time':sysTime,'token':'7XWE32L52T03DH61'}
        r = requests.get("http://coursesrv.nutn.edu.tw/S10582015/receive.php",params=sendData)
        print "Upload temperature and humidity successfully!"
        time.sleep(5)
        count += 1
    time.sleep(1)

