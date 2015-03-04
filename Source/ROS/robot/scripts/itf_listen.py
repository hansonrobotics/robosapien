#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Int8, Bool
import shlex,subprocess,os
import simplejson
import random
cmd1='sox -r 16000 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%'
cmd2='wget -q -U "Mozilla/5.0" --post-file recording.flac --header="Content-Type: audio/x-flac; rate=16000" -O - "http://www.google.com/speech-api/v2/recognize?lang=en-us&client=chromium&key=REPLACEWITHKEY"'
#listen_off=True
#def callback_talking(dat):
	#listen_off= dat.data
	#rospy.loginfo("listen 0ff:"+str(listen_off))
	

def process_speech():
    #pubs = rospy.Publisher('itf_listen', String)
    #pubc = rospy.Publisher('confidence', Int8)
    #pubActive = rospy.Publisher('itf_listen_active', Bool)
    pubs = rospy.Publisher('/sense/stt/get_text', String)
    pubc = rospy.Publisher('/sense/stt/confidence', Int8)
    pubActive = rospy.Publisher('/act/robot/set_listen_led', Bool)
    #rospy.Subscriber("/act/tts/speech_active", Bool, callback_talking)

    keys = ["AIzaSyAFygjZbN2MHH2stfoj4JzqonWctWxKQoY",
        "AIzaSyB0KJ20Hu61LihTuZOilGGccKoirKaaj1M",
        "AIzaSyBB1H15zfUezWeOhpJSPip57GS25zpDi54",
        "AIzaSyAt45KPVubikEYglLViQ7Ijyp4vk6AdAfc",
        "AIzaSyBPWoAKCSROZlcwN7DokO1JOhZ8OgxxZO0",
        "AIzaSyCjELnUutglzEP5Cw0nc4keudaYkVnIhQ0",
        "AIzaSyCrI4f0on71JFmHl1UIQLZIlCUeTTNUi9c",
        "AIzaSyAI1_BkxPqHlakDYAGSsh96UaRpca5j7LE",
        "AIzaSyD5jUzPm-ajshmXjKw2d4YiEbyNED9g3qc",
        "AIzaSyAcout1WkipZo-siXHCnsCJiQJbuIvRG4s",
        "AIzaSyDV8AQOciGeJpSx0sKkl9Xi6O-OZF3IygA",
        "AIzaSyAM2aOh-HWEhH8XZI7foQycswpsjwEWsZo"]
    #keys = [] # Doesn't work


    #keys = [""]
    while not rospy.is_shutdown():
		#if listen_off: continue
        cmdCopy = cmd2[:]
        keyToUse = random.choice(keys)
        print "Key to use: " + keyToUse
        cmdCopy = cmdCopy.replace("REPLACEWITHKEY", keyToUse)
        args2 = shlex.split(cmdCopy)

        print "Start recording"
        #os.system('sox -r 16000 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 3%')

        success_flag = '/tmp/success.flag'
        pubActive.publish(True)
        os.system("rec -r 16000 recording.flac silence 1 0.1 1% 1 2.5 3% && echo 'succcess' > /tmp/success.flag")
        pubActive.publish(False)

        if os.path.isfile(success_flag):
            os.remove(success_flag)
        else:
            return

        print "Posting file to Google..."
        output,error = subprocess.Popen(args2,stdout = subprocess.PIPE, stderr= subprocess.PIPE).communicate()

        output = output.replace('{"result":[]}', '')

        if not error and len(output)>39:
            jsonobj = simplejson.loads(output)

            print "New output is " + output + ": end"

            alternatives = jsonobj['result'][0]['alternative']

            indexer = 0

            done = False

            for lol in alternatives:
                confidence = -1
                try:
                    confidence = alternatives[indexer]['confidence']
                except:
                    pass

                if (confidence >= 0):
                    print "Number " + str(indexer) + " is " + alternatives[indexer]['transcript'] + ", confidence is " + str(confidence)
                    pubs.publish(alternatives[indexer]['transcript'])
                    pubc.publish(confidence)
                    done = True
                else:
                    print "Number " + str(indexer) + " is " + alternatives[indexer]['transcript']
                    pubs.publish(alternatives[indexer]['transcript'])
                    pubc.publish(confidence)
                    done = True
                    break

                if done:
                    break

                indexer += 1
        else:
            print "*** Recognition failed ***"
            pubs.publish("BADINPUT")

        print ""
        print "---------------------------------------------"
        print ""


if __name__ == '__main__':
    print "Initializing node 'itf_listen'"
    rospy.init_node('itf_listen')
    process_speech()
