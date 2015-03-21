#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Int8, Bool
import shlex,subprocess,os
import simplejson
import random
cmd1='sox -r 16000 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%'
cmd2='wget -q -U "Mozilla/5.0" --post-file recording.flac --header="Content-Type: audio/x-flac; rate=16000" -O - "http://www.google.com/speech-api/v2/recognize?lang=en-us&client=chromium&key=REPLACEWITHKEY"'
listen_active=True
def callback_talking(dat):
    global listen_active
    listen_active= dat.data
    rospy.set_param('/sense/stt/get_listen_active',listen_active)
    pubActive = rospy.Publisher('/act/robot/set_listen_led', Bool)
    pubActive.publish(listen_active)
    rospy.loginfo("listen active:"+str(listen_active))

def callback_stop(dat):
    global listen_active
    listen_active= not dat.data
    rospy.set_param('/sense/stt/get_listen_active',listen_active)
    pubActive = rospy.Publisher('/act/robot/set_listen_led', Bool)
    pubActive.publish(listen_active)
    rospy.loginfo("listen active:"+str(listen_active))

def process_speech():
    #pubs = rospy.Publisher('itf_listen', String)
    #pubc = rospy.Publisher('confidence', Int8)
    #pubActive = rospy.Publisher('itf_listen_active', Bool)
    pubs = rospy.Publisher('/sense/stt/get_text', String)
    pubc = rospy.Publisher('/sense/stt/confidence', Int8)
    pubActive = rospy.Publisher('/act/robot/set_listen_led', Bool)
    rospy.set_param('/sense/stt/get_listen_active',listen_active)
    rospy.Subscriber("/sense/stt/set_listen_active", Bool, callback_talking)
    rospy.Subscriber("/act/tts/get_speech_active",Bool,callback_stop)

    rate=rospy.Rate(10)
    #keys = ["your_dev_key1","your_dev_key2"]
    while not rospy.is_shutdown():
        global listen_active
        if not listen_active: #continue
            rospy.loginfo("sleeping")
            rate.sleep()
        else:
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
            if listen_active: #continue

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
