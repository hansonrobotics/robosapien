body_driver
Author: Mandeep Singh Bhatia
Year: 2015

*******************************************
Author: Alex Van Der Peet
Year: 2014

itf_talk
==========

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

It will monitor the /itf_talk topic for input. 

Notes
-----
If at any time Google decides to shutdown / switch API's this code will probably require some changes.

**************************************************************************
itf_listen
==========
Author: Alex Van Der Peet
Year: 2014

itf_listen uses Google's speech-to-text API to convert a sound recording obtained using sox into a string representation

Prerequisites
-------------
Depends on sox for audio recording, tested with ROS Hydro. Installed sox components on my system:

sudo apt-get install sox libsox1b libsox-fmt-base libsox-fmt-alsa
sudo apt-get -y install python-simplejson

Usage
-----
Clone into your catkin workspace, to run:

rosrun itf_listen itf_listen.py

Output will be published on the /itf_listen topic. If Google fails to recognize the voice input, the message BADINPUT will be posted instead.

Notes
-----
If at any time Google decides to shutdown / switch API's (as was the reason for rewriting this to support the v2 API rather than the v1 API) this code will probably require some changes.
