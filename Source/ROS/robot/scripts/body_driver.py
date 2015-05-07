#!/usr/bin/env python

#rs1 body driver
#author:mandeep singh bhatia
#started: 2 March 2015


#/act/robot/send_move_command - text,auto_stop_time_10ms - buffer, a slot removed at stop
#/act/robot/set_pan_angle - int 0-180 degree,time to take
#/act/robot/set_tilt_angle - int 0 - 180 degree, time to take
#/sense/robot/get_sonar_cm - int distance in cm
#/sense/robot/get_compass_deg
#/act/robot/set_listen_led

#/robot/get_pan_range - [min,max]
#/robot/get_tilt_range - [min,max]
#/robot/get_pan
#/robot/get_tilt

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from robot.msg import sonar
from robot.msg import compass #only when imu is installed
from robot.msg import robot_cmd
from robot.msg import gesture_r #only if gesture sensor is installed
import serial

#pan or tilts assumed start from 0 degrees
MAX_PAN=90
MAX_TILT=90
ser = serial.Serial()
distpub = rospy.Publisher('/sense/robot/get_sonar_cm', sonar, queue_size=10)
dirpub = rospy.Publisher('/sense/robot/get_compass_deg', compass, queue_size=10)
gesturepub=rospy.Publisher('/sense/robot/get_gesture',gesture_r,queue_size=1)
robosapien_v1_ir_codes={"turn right":"80",
                        "right arm up":"81",
                        "right arm out":"82",
                        "right arm down":"84",
                        "right arm in":"85",
                        "walk forward":"86",
                        "walk backward":"87",
                        "turn left":"88",
                        "left arm up":"89",
                        "left arm out":"8a",
                        "left arm down":"8c",
                        "left arm in":"8d",
                        "stop":"8e",
                        "right turn step":"a0",
                        "right hand thump":"a1",
                        "right hand throw":"a2",
                        "sleep":"a3",
                        "right hand pickup":"a4",
                        "forward step":"a6",
                        "backward step":"a7",
                        "left turn step":"a8",
                        "left hand thump":"a9",
                        "left hand throw":"aa",
                        "left hand pickup":"ac",
                        "reset":"ae",
                        "wake up":"b1",
                        "right hand strike 3":"c0",
                        "right hand sweep":"c1",
                        "right hand strike 2":"c3",
                        "right hand strike 1":"c5",
                        "left hand strike 3":"c8",
                        "left hand sweep":"c9",
                        "left hand strike 2":"cb",
                        "left hand strike 1":"cd",
                        "demo karate":"d2",
                        "dance":"d4"}

def serial_write(strng):
    try:
        ser.write(strng)
    except:
        if (ser.isOpen()):ser.close()
        if not(ser.isOpen()):ser.open()
        rospy.logerr("Error writing to serial port")

def callback_move(cmd):
    data=cmd.cmd
    dur=cmd.duration_10ms
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

    if data in robosapien_v1_ir_codes:
        command="r"+robosapien_v1_ir_codes[data]
        hx=(hex(dur)[2:4])
        if len(hx)<2: hx="0"+hx
        command=command+hx
        serial_write(command)
        rospy.loginfo("command= %s",command)

def callback_pan(dat):
    data=dat.data
    rospy.loginfo(rospy.get_caller_id() + " pan %d", data)
    if data<0: data=0
    if data>MAX_PAN: data=MAX_PAN
    hexdata=hex(data)[2:4]
    if len(hexdata)<2: hexdata="0"+hexdata
    command="s10"+hexdata
    serial_write(command)
    rospy.set_param('/robot/get_pan',data)

def callback_tilt(dat):
    data=dat.data
    rospy.loginfo(rospy.get_caller_id() + " tilt %d",data)
    if data<0: data=0
    if data>MAX_TILT: data=MAX_TILT
    hexdata=hex(data)[2:4]
    if len(hexdata)<2: hexdata="0"+hexdata
    command="s00"+hexdata
    serial_write(command)
    rospy.set_param('/robot/get_tilt',data)

def callback_led(dat):
    data=dat.data
    cmmd="l"
    if (data):cmmd=cmmd+"1"
    else: cmmd=cmmd+"0"
    serial_write(cmmd)

def init():
     rospy.init_node('body_node', anonymous=False)
     rospy.set_param('/robot/get_pan_range',[0,MAX_PAN])
     rospy.set_param('/robot/get_tilt_range',[0,MAX_TILT])
     rospy.set_param('/robot/get_pan',45)
     rospy.set_param('/robot/get_tilt',45)
     rospy.Subscriber("/act/robot/send_move_command", robot_cmd, callback_move)
     rospy.Subscriber("/act/robot/set_pan_angle", Int32, callback_pan)
     rospy.Subscriber("/act/robot/set_tilt_angle",Int32,callback_tilt)
     rospy.Subscriber("/act/robot/set_listen_led",Bool,callback_led)
     ser.baudrate=9600
     ser.port='/dev/rfcomm0'
     ser.timeout=2
     ser.open()
     if not(ser.isOpen()):
         rospy.logerr("Error: Could not open bluetooth Serial Port")

def parse(txt):
    if txt[0]=='!':
        ttxt=txt[2:len(txt)-1]
        ltxt=ttxt.split(",")
        #for item in ltxt:
        if txt[1]=='d':
            dist=int(ttxt)
            tsonar=sonar()
            #to add timestamp to header
            tsonar.distance_cm=dist
            #publish distance
            distpub.publish(tsonar)
        elif txt[1]=='c':
            heading=float(ttxt)
            tcompass=compass()
            #to add timestamp
            tcompass.heading_deg=heading
            dirpub.publish(tcompass)
        elif txt[1]=='r':
            direction=ltxt[0]
            speed=ltxt[1]
            ges=gesture_r()
            ges.direction=direction
            if speed=='h':
                ges.speed_level=2
            elif speed=='m':
                ges.speed_level=1
            elif speed=='l':
                ges.speed_level=0
            #publish gesture
            gesturepub.publish(ges)

def poll():
    rate=rospy.Rate(1000)
    while not rospy.is_shutdown():
        try:
            parse_txt=ser.readline()
            if len(parse_txt)>2: parse(parse_txt)
        except:
            if(ser.isOpen()):ser.close()
            if not(ser.isOpen()):ser.open()
            if not(ser.isOpen()): rospy.logerr("could not open bluetooth serial port")
        rate.sleep()


if __name__=='__main__':
    init()
    poll()
