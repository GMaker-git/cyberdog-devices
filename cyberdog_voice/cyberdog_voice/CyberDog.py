import array
import time
from rclpy.node import Node
from rclpy.client import Client
from protocol.msg import AudioPlayExtend
from protocol.msg import MotionStatus
from protocol.msg import MotionServoCmd
from protocol.srv import MotionResultCmd
from std_msgs.msg import Bool, UInt8, String
import logging
import queue
from . import manual

class CyberDog2():

    _VoiceNode:Node=None
    _Speechpublisher=None
    _wakeuppublisher=None
    _voiceplaysub=None
    _logger=None
    _ttsqueue:queue.Queue
    _ttsplaying:bool
    _motioncli:Client
    _actionmap=None
    _currmotion_id=0

    def __init__(self,node:Node) -> None:
        self._VoiceNode=node
        self._logger=logging.getLogger("CyberDog2Voice")
        macnamespace=manual.get_namespace()
        self.Speechpublisher=self._VoiceNode.create_publisher(
            msg_type=AudioPlayExtend,topic="/"+macnamespace+"/speech_play_extend",qos_profile=100
        )
        self._wakeuppublisher=self._VoiceNode.create_publisher(
            msg_type=Bool,topic="/"+macnamespace+"/dog_wakeup",qos_profile=100
        )
        self._motionpublisher=self._VoiceNode.create_publisher(
            msg_type=MotionServoCmd,topic="/"+macnamespace+"/motion_servo_cmd",qos_profile=100
        )
        self._voiceplaysub = self._VoiceNode.create_subscription(msg_type=UInt8, topic='/'+macnamespace+'/audio_board_state', callback=self.AudioStateCallBack, qos_profile=10)
        self._motionstatussub = self._VoiceNode.create_subscription(msg_type=MotionStatus, topic='/'+macnamespace+'/motion_status', callback=self.MotionStatusCallBack, qos_profile=10)
        self._currmotion_id=0
        self._ttsqueue=queue.Queue(200)
        self._ttsplaying=False
        self._motioncli = self._VoiceNode.create_client(MotionResultCmd, '/'+macnamespace+'/motion_result_cmd')
        self._actionmap={
            '装死':101,
            '休息一下':101,
            '发现':101,
            '查下':101,
            '跟着我':1003,
            '跟我来':1003,
            '比心':175,
            '旋转右跳':131,
            '旋转左跳':130,
            '原地跳远':133,
            '原地跳起':136,
            '握右手':141,
            '握左手':142,
            '俯卧撑':174,
            '伸懒腰':146,
            '扭屁股':150,
            '扭头':145,
            '恭喜发财':123,
            '卧倒':101,
            '坐下':143,
            '跳上台阶':126,
            '跳下台阶':137,
            '跳舞':140,
            '前后跳':301,
            '狗粮来了':153,
            '蹦跶蹦跶':166,
            '高兴一下':164,
            '鞠躬':155,
            '拜年':123,
            '点头':161,
            '立正':111,
            '太空步':152,
            '摇头':145,
            '摇尾巴':1002,
            '握手':141,
            '跟我走':1003,
            '原地转圈':1004,
            '跳跃':162,
            '跳起来':162,
            '过来':1001,
            '芭蕾舞':151,
            '作揖':123,
            '起立':111,
            '站起来':111,
            '起来':111,
            '趴下':101
        }

    # 动作指令
    def Motion_Move(self,duration, x_vel, y_vel,  omega):
        print(f"SendMove:{duration}")
        msg=MotionServoCmd()        
        msg.motion_id=304
        msg.vel_des = array.array('f',[x_vel, y_vel, omega])
        msg.step_height = [0.05, 0.05]
        usetimes=0
        while usetimes<duration:
            self._motionpublisher.publish(msg)
            time.sleep(0.05)
            usetimes=usetimes+0.05

    # 动作指令
    def SendMotionID(self,motionid):
        print(f"SendMotion:{motionid}")
        cmd=MotionResultCmd.Request()
        cmd.motion_id=motionid
        self._motioncli.call_async(request=cmd)

    def SendRound(self,angle):
        cmd=MotionResultCmd.Request()
        cmd.motion_id=304
        cmd.vel_des = {0, 0, 0.1}
        cmd.step_height = {0.05, 0.05}
        self._motioncli.call_async(request=cmd)
    
    def TailShake(self):
        stdmess:Bool=Bool()
        stdmess.data=True
        self._wakeuppublisher.publish(stdmess)

    # 摆姿势
    def ShowPos(self,posid):
        # 所有动作详细见cyberdog_ws/interaction/cyberdog_vp/cyberdog_vp_abilityset/include/cyberdog_vp_abilityset/common.hpp
        
        print(f"展示姿态:{posid}")
        motionid=0
        if posid in self._actionmap:
            motionid=self._actionmap[posid]
            if motionid<1000:
                self.SendMotionID(motionid)
            elif(motionid==1001):
                #过来
                self.PlayTTS(posid+"正在导航到你的位置")
            elif(motionid==1003):    
                #跟我来
                self.PlayTTS(posid+"来了，主人带我去浪吧")
            elif(motionid==1004):    
                #原地打转
                self.Motion_Move(1.0,0,0,0.3)
            elif(motionid==1002):    
                #摆动尾巴
                self.TailShake()
            else:
                self.PlayTTS(posid+" 是啥？我有点蒙")
        else:
            self.PlayTTS(posid+" 这个动作我还没学会，说个我会的吧")

    def _publishTTSPlay(self,say:str):
        atts_msg:AudioPlayExtend=AudioPlayExtend()
        atts_msg.module_name = "audio_action"
        atts_msg.is_online = True
        atts_msg.text = say
        self.Speechpublisher.publish(atts_msg)
        
    def AudioStateCallBack(self,msg):
        # self._logger.info(msg=f"播放回调{msg.data}")
        if self._ttsplaying==True:
            if msg.data==10:
                # 播放完成
                if self._ttsqueue.empty():
                    self._ttsplaying=False
                else:
                    try:
                        nextsay=self._ttsqueue.get(timeout=0.1)
                        self._ttsqueue.task_done()
                        self._publishTTSPlay(say=nextsay)
                    except Exception as err:
                        return
    # 更新当前动作
    def MotionStatusCallBack(self,msg:MotionStatus):        
        self._currmotion_id=msg.motion_id
        # print(f"MotionStatusCallBack:{self._currmotion_id}")

    # 旋转角度
    def TrunAngle(self,angle):
        print(f"旋转姿势:{angle}")
        # 站起来
        if self._currmotion_id!=111:
            self.SendMotionID(111)
            time.sleep(1)
        # self.Motion_Move(1,0,0,0.3)

    # 语音播放
    def PlayTTS(self,say:str):    
        self._logger.info(msg="PlayTTS")
        if self._ttsplaying==False:
            self._publishTTSPlay(say)
            self._ttsplaying=True
        else:
            self._ttsqueue.put(say)

    # 执行意图
    def StopTTS(self):
        self._logger.info(msg="stoptts")
        if self._ttsplaying:
            self._ttsplaying=False
            while not self._ttsqueue.empty():
                self._ttsqueue.get()
            self._publishTTSPlay("  ")

    #显示语音识别过程
    def ShowAsrText(self,text:str):
        print(f"显示屏显示识别文字:{text}")
        pass

    #唤醒
    def WakeUp(self):
        # self._logger.info("woowoo")
        audio_msg:AudioPlayExtend=AudioPlayExtend()
        audio_msg.module_name = "cyberdog_vp"
        audio_msg.is_online = False
        audio_msg.speech.module_name = "cyberdog_vp"
        audio_msg.speech.play_id = 4000
        self.Speechpublisher.publish(audio_msg)

        # 通知尾巴
        self.TailShake()
        # 等待狗叫结束
        time.sleep(0.6)

    # 执行意图
    def RunTask(self,intent):
        print(intent)
        if "action" in intent:
            action=intent["action"]
            if action=="showpos":
                self.ShowPos(intent["postype"])
            else:
                self.PlayTTS("这个技能我还在学习中")
