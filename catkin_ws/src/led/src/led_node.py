#!/usr/bin/env python
import rospy
from Adafruit_PWM_Servo_Driver import PWM
from hbc_msgs.msg import LEDState

class LED(object):
    def __init__(self,debug=False):
        self.pwm = PWM(address=0x40,debug=False)
        #self.sub_led = rospy.Subscriber('/topic_led', LEDState, self.cbLed, queue_size=1)
    
    def initState(self):
        self.pwm.setPWM(0,4095,4095)
        self.pwm.setPWM(1,4095,4095)
        self.pwm.setPWM(2,4095,4095)
        self.pwm.setPWM(3,4095,4095)
        
    def cbLed(self, led_msg):
        initState()
        self.led_msg = led_msg
        if self.led_msg.mode == "left":
            self.pwm.setPWM(2,0,4095)
        elif self.led_msg.mode == "right":
            self.pwm.setPWM(3,0,4095)
        elif self.led_msg.mode == "foward":
            self.pwm.setPWM(0,0,4095)
        elif self.led_msg.mode == "backward":
            self.pwm.setPWM(1,0,4095)
        
        
if __name__ == '__main__':
    rospy.init_node('led',anonymous=False)
    led = LED()
    rospy.spin()