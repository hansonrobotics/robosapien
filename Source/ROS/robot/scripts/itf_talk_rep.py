#!/usr/bin/env python

import rospy
import urllib, pycurl
from threading import Thread
from std_msgs.msg import String
from std_msgs.msg import Bool
import subprocess

class ITFTalker(Thread):
    NODE_NAME = 'itf_talker'
    pub = rospy.Publisher('/act/tts/get_speech_active', Bool, queue_size=1)

    def __init__(self):
        Thread.__init__(self)
        rospy.init_node(ITFTalker.NODE_NAME, log_level=rospy.INFO)
        rospy.Subscriber("/act/tts/set_text", String, self.callback)

    def split_text_rec(self, input_text, max_length=45):
            """
            Split a string into substrings which are at most max_length.
            """
            combined_text = []

            if (len(input_text) > max_length):
                while (input_text > max_length):
                    lastSpace = max_length
                    section = ""

                    while (lastSpace > 0):
                        if (input_text[lastSpace] == ' '):
                            section = input_text[:lastSpace]
                            input_text = input_text[lastSpace:]
                        lastSpace -= 1

                    combined_text.append(section)

            else:
                combined_text.append(input_text)

            return combined_text

    def downloadFile(self, url, fileName):
        fp = open(fileName, "wb")
        curl = pycurl.Curl()
        curl.setopt(pycurl.URL, url)
        curl.setopt(pycurl.WRITEDATA, fp)
        curl.perform()
        curl.close()
        fp.close()

    def getGoogleSpeechURL(self, phrase):
        googleTranslateURL = "http://translate.google.com/translate_tts?tl=en&"
        parameters = {'q': phrase}
        data = urllib.urlencode(parameters)
        googleTranslateURL = "%s%s" % (googleTranslateURL,data)
        return googleTranslateURL

    def speakSpeechFromText(self, phrase):
        phraseSections = self.split_text_rec(phrase, '')

        for index, section in enumerate(phraseSections):
            print "At index " + str(index) + " is " + section + "\n"

        for index, section in enumerate(phraseSections):
            googleSpeechURL = self.getGoogleSpeechURL(section)
            print "Downloading " + googleSpeechURL + " to " + "tts" + str(index).zfill(index) + ".mp3\n"
            self.downloadFile(googleSpeechURL,"tts" + str(index).zfill(index) + ".mp3")
            print index, section

        ITFTalker.pub.publish(True)

        for index, section in enumerate(phraseSections):
            print 'Calling mplayer with parameter ' + 'tts' + str(index).zfill(index) + '.mp3'
            subprocess.call(['mplayer', 'tts' + str(index).zfill(index) + '.mp3'])
            #ITFTalker.pub.publish("Google Voice completed.") ..mandeep
        ITFTalker.pub.publish(False)
        #os.system("mplayer tts " + str(index).zfill(index) + ".mp3 -af extrastereo=0 &")

    #speakSpeechFromText("Four score and seven years ago our fathers brought forth on this continent, a new nation, conceived in Liberty, and dedicated to the proposition that all men are created equal.")

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
        self.speakSpeechFromText(data.data)

if __name__ == '__main__':
    rospy.loginfo("Starting {0}...".format(ITFTalker.NODE_NAME))
    talker = ITFTalker()

    talker.start()
    rospy.loginfo("{0} started, listening for text input on topic itf_talk...".format(ITFTalker.NODE_NAME))

    rospy.spin()

    rospy.loginfo("Stopping {0}...".format(ITFTalker.NODE_NAME))
    talker.join()
    rospy.loginfo("{0} stopped.".format(ITFTalker.NODE_NAME))
