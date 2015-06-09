#!/usr/bin/env python

import os
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
#import subprocess

class ITFTalker:
    NODE_NAME = 'itf_talker'
    pub = rospy.Publisher('/act/tts/get_speech_active', Bool, queue_size=1)

    def __init__(self):
        rospy.init_node(ITFTalker.NODE_NAME, log_level=rospy.INFO)
        rospy.Subscriber("/act/tts/set_text", String, self.callback)
    def mute_mic(self):
        #mic="alsa_input.usb-Omniechnologies__Inc.538-2655-08.12.30.4_Monitor_Webcam-02-Webcam.analog-stereo"
        mic="bluez_source.00_1A_7D_10_73_62"
        os.system("pacmd set-source-mute "+mic+" 1")
    def unmute_mic(self):
        #mic="alsa_input.usb-Omniechnologies__Inc.538-2655-08.12.30.4_Monitor_Webcam-02-Webcam.analog-stereo"
        mic="bluez_source.00_1A_7D_10_73_62"
        os.system("pacmd set-source-mute "+mic+" 0")

    def speakSpeechFromText(self, phrase):
        ##self.mute_mic()
        ITFTalker.pub.publish(True)
        speak='echo "'+phrase+'" | festival --tts'
        os.system(speak)
        ITFTalker.pub.publish(False)
        ##self.unmute_mic()

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
        self.speakSpeechFromText(data.data)

if __name__ == '__main__':
    rospy.loginfo("Starting {0}...".format(ITFTalker.NODE_NAME))
    talker = ITFTalker()

    rospy.loginfo("{0} started, listening for text input on topic itf_talk...".format(ITFTalker.NODE_NAME))

    rospy.spin()

    rospy.loginfo("Stopping {0}...".format(ITFTalker.NODE_NAME))
    rospy.loginfo("{0} stopped.".format(ITFTalker.NODE_NAME))
