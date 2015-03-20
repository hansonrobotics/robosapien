#! /usr/bin/env python
__author__ = 'mandeep'
import rospy
import owyl
from owyl import blackboard
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32
from robot.msg import robot_cmd
from vision.msg import face_box
from vision.msg import targets
from tld_msgs.msg import BoundingBox
# reset tld tracker each time track command is given or subject is lost or low confidence for a few seconds
#cmds std_msgs::Char ... may need to make it private subscription in future
#        case 'b':
#          clearBackground();
#        case 'c':
#          stopTracking();
#        case 'l':
#          toggleLearning();
#        case 'a':
#          alternatingMode();
#        case 'e':
#          exportModel();
#        case 'i':
#          importModel();
#        case 'r':
#          reset();
#from tld_msgs.msg import Target
from robot.msg import compass
from robot.msg import sonar
stt_cmd_map= {
             "BADINPUT":(0,0,""),
             "walk forward":(1,800,"walk forward"),#body class,800*10 ms timer,command for body
             "walk back":(1,800,"walk backward"),
             "turn left":(1,800,"turn left"),
             "turn right":(1,800,"turn right"),
             "look up":(2,400,"t+"),#tilt up
             "look down":(2,400,"t-"),
             "look left":(2,400,"p-"),#pan,400*10ms,left
             "look right":(2,400,"p+"),
             "look center":(2,400,"tp"),
             "give faces":(3,500,"faces"),#test speech active?
             "give distance":(3,500,"sonar"),
             "give heading":(3,500,"compass")#,
             #"what do you see":(3,800,""),
             #"track me":(4,0,"friend")
             #"track object":(4,1,"object")
}

class body_cmd:
    def __init__(self,blackboard):
        self.bb=blackboard

class behavior:
    def __init__(self,bb):
        self.blackboard=bb#blackboard.Blackboard("behavior")
        rospy.loginfo (self.blackboard["stt"])
        self.main_tree=self.create_main_tree()
        #while not rospy.is_shutdown():
        #    self.main_tree.next()

    def create_main_tree(self):
        tree=owyl.repeatAlways(
                owyl.sequence(
                    self.is_stt_on(),
                    self.is_stt_done(),
                    owyl.selector(
                        self.run_stt_command(),
                        self.run_stt_chat()
                    )
                )
        )
        return owyl.visit(tree,blackboard=self.blackboard)

    @owyl.taskmethod
    def is_stt_on(self,**kwargs):
        #if self.blackboard["stt_counter"]<=0 and (not self.blackboard["speech_active"]):
        #self.b_update_bb()
        #rospy.loginfo(" stop counter="+str(self.blackboard["stop_counter"]))
        if self.blackboard["stop_counter"]<=0:
            #rospy.loginfo("ready..")
            yield True
        else:
            #rospy.loginfo("waiting...")
            yield False

    @owyl.taskmethod
    def is_stt_done(self,**kwargs):
        if self.blackboard["stt_read"]==False:
            rospy.loginfo (self.blackboard["stt"])
            self.blackboard["stt_read"]=True
            yield True
        else:
            yield False

    @owyl.taskmethod
    def run_stt_command(self,**kwargs):
        #rospy.loginfo("processing stt= "+self.blackboard["stt"])
        if self.blackboard["stt"] in stt_cmd_map:
            rospy.loginfo("processing stt= "+self.blackboard["stt"])
            self.stt_cmd(self.blackboard["stt"])
            yield True
        else:
            yield False

    @owyl.taskmethod
    def run_stt_chat(self,**kwargs):
        self.blackboard["pub_chat"].publish(self.blackboard["stt"])
        yield True #needs to run last with zeno dial

    def stt_cmd(self,stt):
        func=stt_cmd_map[stt]
        tp=func[0]
        self.blackboard["stop_counter"]=func[1]
        todo=func[2]
        if tp==1:
            rospy.loginfo("move bot ="+todo)
            r_cmd=robot_cmd()
            r_cmd.cmd=todo
            r_cmd.duration_10ms=0
            self.blackboard["pub_move"].publish(r_cmd)
        elif tp==2:
            if todo=="t+" or todo=="t-":
                ta=rospy.get_param("/robot/get_tilt")
                tlt=0
                if todo=="t+":tlt=ta+5
                else:tlt=ta-5
                v=Int32(tlt)
                self.blackboard["pub_tilt"].publish(v)
            elif todo=="p+" or todo=="p-":
                pa=rospy.get_param("/robot/get_pan")
                pn=0
                if todo=="p+":pn=pa+5
                else:pn=pa-5
                v=Int32(pn)
                self.blackboard["pub_pan"].publish(v)
            else:
                v=Int32(45)
                self.blackboard["pub_pan"].publish(v)
                self.blackboard["pub_tilt"].publish(v)
        elif tp==3:
            to_say="What"
            if todo=="faces":
                to_say="I see "+str(board["num_faces"])+" faces"
            elif todo=="sonar":
                to_say=str(board["sonar_cm"])+" centimeters"
            elif todo=="compass":
                to_say=str(board["compass_deg"])+" degrees"
            self.blackboard["pub_speak"].publish(to_say)

def set_speech(dat):
    if board["stt_read"]:
        board["stt"]=dat.data
        rospy.loginfo("Heard : "+board["stt"])
        board["stt_read"]=False


def set_sonar(dat):
    board["sonar_cm"]=dat.distance_cm

def set_compass(dat):
    board["compass_deg"]=dat.heading_deg

def update_bb():
    #read decrement counter
    if board["stop_counter"]>0:
        board["stop_counter"]=board["stop_counter"]-1
        if board["stop_counter"]==0:
            #publish stop,
            stp_cmd=robot_cmd()
            stp_cmd.cmd="stop"
            stp_cmd.duration_10ms=0
            pub_move.publish(stp_cmd)
            #reset num faces seen

def get_faces(boxes):
    board["num_faces"]=len(boxes.faces)


#run ant again
#/ZenoDial/output_text, zenodial.java, output.java
#/ZenoDial/text_input
#/facedetect -- face detector
#.. to add also pub and subscribe for

#map zenodial output directly to itf_talk
#upload changes to zenodial+robot, launch with ant ZenoDial
#connect speech active and enable listen

if __name__=="__main__":
    rospy.init_node("RS1_behavior")
    #subscribe to distance, compass, face bound box, objects models to be loaded
    board=blackboard.Blackboard("behavior")
    board["stt"]=""
    board["stt_read"]=True
    board["stop_counter"]=0
    board["sonar_cm"]=0
    board["compass_deg"]=0
    board["num_faces"]=0
    rospy.Subscriber("/sense/robot/get_sonar_cm",sonar,set_sonar)
    rospy.Subscriber("/sense/robot/get_compass_deg",compass,set_compass)
    rospy.Subscriber("/sense/stt/get_text",String,set_speech)
    rospy.Subscriber('/facedetect',targets,get_faces)
    pub_enable_listen=rospy.Publisher("/sense/stt/set_listen_active", Bool)
    board["pub_enable_listen"]=pub_enable_listen
    pub_speak=rospy.Publisher("/act/tts/set_text", String)
    board["pub_speak"]=pub_speak
    pub_move=rospy.Publisher("/act/robot/send_move_command", robot_cmd)
    board["pub_move"]=pub_move
    pub_pan=rospy.Publisher("/act/robot/set_pan_angle", Int32)
    board["pub_pan"]=pub_pan
    pub_tilt=rospy.Publisher("/act/robot/set_tilt_angle",Int32)
    board["pub_tilt"]=pub_tilt
    pub_chat=rospy.Publisher("/ZenoDial/text_input",String)
    board["pub_chat"]=pub_chat

    be=behavior(board)
    be_tree=be.main_tree
    rate=rospy.Rate(100)#100hz
    while not rospy.is_shutdown():
        update_bb()
        be_tree.next()
        rate.sleep()
    rospy.loginfo("bye")
