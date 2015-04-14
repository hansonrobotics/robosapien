#!/usr/bin/env python

import rospy
import urllib, pycurl
from threading import Thread
from std_msgs.msg import String
from std_msgs.msg import Bool
import subprocess

import pydub
import pygame.mixer, pygame.sndarray, numpy as np

#sudo apt-get install python-pygame

class ITFTalker(Thread):
    NODE_NAME = 'itf_talker'
    pub = rospy.Publisher('/act/tts/get_speech_active', Bool, queue_size=1)

    def __init__(self):
        Thread.__init__(self)
        rospy.init_node(ITFTalker.NODE_NAME, log_level=rospy.INFO)
        rospy.Subscriber("/act/tts/set_text", String, self.callback)
        #pygame.mixer.init(44100,-16,2,4096)
        pygame.mixer.init(16000)

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
#algorithms from:
#  http://zulko.github.io/blog/2014/03/29/soundstretching-and-pitch-shifting-in-python/
    def pitchshift(self,snd_array, n, window_size=2**13, h=2**11):
        """ Changes the pitch of a sound by ``n`` semitones. """
        factor = 2**(1.0 * n / 12.0)
        stretched = self.stretch(snd_array, 1.0/factor, window_size, h)
        return self.speedx(stretched[window_size:], factor)

    def stretch(self,sound_array, f, window_size, h):
        """ Stretches the sound by a factor `f` """

        phase  = np.zeros(window_size)
        hanning_window = np.hanning(window_size)
        result = np.zeros( len(sound_array) /f + window_size)

        for i in np.arange(0, len(sound_array)-(window_size+h), h*f):

            # two potentially overlapping subarrays
            a1 = sound_array[i: i + window_size]
            a2 = sound_array[i + h: i + window_size + h]

            # resynchronize the second array on the first
            s1 =  np.fft.fft(hanning_window * a1)
            s2 =  np.fft.fft(hanning_window * a2)
            phase = (phase + np.angle(s2/s1)) % 2*np.pi
            a2_rephased = np.fft.ifft(np.abs(s2)*np.exp(1j*phase))

            # add to result
            i2 = int(i/f)
            result[i2 : i2 + window_size] += hanning_window*a2_rephased

        result = ((2**(16-4)) * result/result.max()) # normalize (16bit)

        return result.astype('int16')

    def speedx(self,sound_array, factor):
        """ Multiplies the sound's speed by some `factor` """
        indices = np.round( np.arange(0, len(sound_array), factor) )
        indices = indices[indices < len(sound_array)].astype(int)
        return sound_array[ indices.astype(int) ]

    def pitch_shifted(self,sound_file):
        sound = pygame.mixer.Sound(sound_file)
        snd_array = pygame.sndarray.array(sound)
        print(pygame.sndarray.get_arraytype())
        sa1=snd_array[0:22]
        #sa2=snd_array[23::2]
        sa3=self.speedx(snd_array[23:],0.8)
        snd_r=np.append(sa1,sa3,axis=0)
        snd_out = pygame.sndarray.make_sound(snd_r)
        snd_out.play()
        #snd_resample=numpy.array()
        #for a in range(0,len(snd_array),2):numpy.append(snd_resample,(snd_array[a]))
        #snd_resample=[pygame.sndarray.get_arraytype()]
        #numpy.resize(snd_resample,[len(snd_array)/2])
        #for a in range(0,len(snd_array),2):snd_resample[a/2]=(snd_array[a])

        #snd_out = pygame.sndarray.make_sound(snd_resample)
        #snd_out.play()

    def speakSpeechFromText(self, phrase):
        ITFTalker.pub.publish(True)
        phraseSections = self.split_text_rec(phrase, '')

        for index, section in enumerate(phraseSections):
            print "At index " + str(index) + " is " + section + "\n"

        for index, section in enumerate(phraseSections):
            googleSpeechURL = self.getGoogleSpeechURL(section)
            print "Downloading " + googleSpeechURL + " to " + "tts" + str(index).zfill(index) + ".mp3\n"
            self.downloadFile(googleSpeechURL,"tts" + str(index).zfill(index) + ".mp3")
            filen=pydub.AudioSegment.from_mp3("tts" + str(index).zfill(index) + ".mp3")
            filen.export("tts" + str(index).zfill(index) + ".wav",format="wav")
            print index, section

        for index, section in enumerate(phraseSections):
            print 'Calling mplayer with parameter ' + 'tts' + str(index).zfill(index) + '.mp3'
            self.pitch_shifted('tts' + str(index).zfill(index) + '.wav')
            #subprocess.call(['mplayer', 'tts' + str(index).zfill(index) + '.mp3'])
#            pygame.mixer.music.load('tts' + str(index).zfill(index) + '.mp3')
#            pygame.mixer.music.play()
            while pygame.mixer.get_busy() == True:
                continue
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
