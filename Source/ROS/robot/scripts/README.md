body_driver
Author: Mandeep Singh Bhatia
Year: 2015

Current defaults : itf_tak_rep2.py, itf_listen.py(alternate use with weblistner as in sample launch file)

*******************************************
Author: Alex Van Der Peet
Year: 2014

itf_talk (depricated - can't be used as library api broken with updates, use itf_talk_rep2.py)
Use itf_talk_rep3.py as of may 2015 [ Author - Mandeep Singh Bhatia] 
==========

itf_talk_rep3 requires installation of festival speech synthesis software [executable called festival]

itf_listen uses Google's text-to-speech API to convert a string of text into several audio files.

Because the service is limited to strings of 100 characters, the node will cut up longer texts into pieces, download the separate MP3 files, and then play them in sequence.

Prerequisites
-------------
sudo apt-get -y install mplayer python-pycurl libcurl3

New pre-reqs after adding lip-synch support:

sudo apt-get install libavbin-dev libavbin0 python-pyglet libav-tools python-pip

sudo pip install pydub

Usage
-----
Clone into your catkin workspace, to run:

rosrun itf_talk itf_talk.py

It will monitor the /act/tts/set_text topic for input. 

Notes
-----
If at any time Google decides to shutdown / switch API's this code will probably require some changes.

**************************************************************************
itf_listen (should be replaced by weblistner using chrome browser)
==========
Author: Alex Van Der Peet
Year: 2014

itf_listen uses Google's speech-to-text API to convert a sound recording obtained using sox into a string representation

Prerequisites
-------------
Depends on sox for audio recording, tested with ROS Hydro. Installed sox components on my system:

sudo apt-get install sox libsox-fmt-base libsox-fmt-alsa
sudo apt-get -y install python-simplejson

Usage
-----
Clone into your catkin workspace, edit itf_listen.py to add your google development keys in keys array.
to run:
rosrun itf_listen itf_listen.py

Output will be published on the /sense/stt/get_text topic. If Google fails to recognize the voice input, the message BADINPUT will be posted instead.

Notes
-----
If at any time Google decides to shutdown / switch API's (as was the reason for rewriting this to support the v2 API rather than the v1 API) this code will probably require some changes.

**************************************************************************
itf_talk_rep2 (Made by Mandeep Singh Bhatia - 25 Mar 2015, is a modification of old itf_talk by alex van der peet)
==========

itf_listen uses Google's text-to-speech API to convert a string of text into several audio files.

Because the service is limited to strings of 100 characters, the node will cut up longer texts into pieces, download the separate MP3 files, and then play them in sequence.

Prerequisites
-------------
sudo apt-get -y install mplayer python-pycurl libcurl3

sudo apt-get install libavbin-dev libavbin0 python-pyglet libav-tools python-pip

sudo pip install pydub

sudo apt-get install python-pygame numpy

Usage
-----
Clone into your catkin workspace, to run:

rosrun robot itf_talk_rep2.py

It will monitor the /act/tts/set_text topic for input.

Notes
-----
If at any time Google decides to shutdown / switch API's this code will probably require some changes.

**************************************************************************
For using chrome instead of itf_listen(it takes away certain limitations)
openssl req -new -newkey rsa:2048 -days 365 -nodes -x509 -keyout server.key -out server.crt
http://stackoverflow.com/questions/7580508/getting-chrome-to-accept-self-signed-localhost-certificate
https://major.io/2007/08/02/generate-self-signed-certificate-and-key-in-one-line/
use localhost for domain and names if using with localhost name
Use above keys in flask in weblistner.py [robot package]
In chrome export the certificate to PKCS #7 single certificate(clicking on https) and then in chrome advanced settings choose to manage https certificates and import in Authorities tab
*********************************************************************
additional : festival and pulse audio

echo "This is an example. Arch is the best." | festival --tts
[below used for muting microphone while speaking in itf_talk_rep2.py]
pacmd list-sources
pacmd set-source-mute INDEX 0  //unmute
pacmd set-source-mute INDEX 1  //mute
pacmd set-source-mute "alsa_input.usb-Omniechnologies__Inc.538-2655-08.12.30.4_Monitor_Webcam-02-Webcam.analog-stereo" 0

