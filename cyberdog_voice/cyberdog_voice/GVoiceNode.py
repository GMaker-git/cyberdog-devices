
import time
import queue
import logging
import os
import threading
from venv import create
from rclpy.node import Node
from std_msgs.msg import String


from . import R818Voice
from . import XFAIUIAPI
from . import VoiceBase
from . import ZhiPuLLM
from . import CyberDog
from . import manual

# 讯飞语音识别API,APPID和Key
XUNFEIAPPID="xxx"
XUNFEIKEY="xxx"

# 智普清言APPID
ZhiPuLLMAPPID="xxx"

#讯飞六mic通讯串口
MICTTYSPORT="/dev/ttyACM1"

class GVoiceNode(Node):
    _eventqueue:queue.Queue
    _voiceDevice:R818Voice.VoiceDevice
    _asrAPI:XFAIUIAPI.AIUIApi
    _islisten:bool=False
    _logger:logging.Logger
    _cyberdog:CyberDog.CyberDog2
    _llmodel:ZhiPuLLM.LLMApi
    #是否记录过程文件
    _savedialoglog:bool=True
    _savedialogpath:str="/SDCARD/voiceRec/"
    _savedialogsession:str=None

    #语音识别是否打开语义理解
    _isAsrWithNlp:bool=True

    def onAsrResult(self,event):
        self._eventqueue.put(event,block=False)

    def onNLPResult(self,event):
        self._eventqueue.put(event,block=False)

    def get_logger(self):
        return self._logger
    
    def onLLMResult(self,event):
        self._eventqueue.put(event,block=False)

    def onWakeup(self,event):        
        self._eventqueue.put(event,block=False)

    def getlogsession(self):
        return self._savedialogsession

    def resetlogsession(self):
        if self._savedialoglog:
            self._savedialogsession=self._savedialogpath+str(time.time())
    def setwakeupwords(self, msg):
        self._logger.info(f"设置唤醒词:{msg.data}")
        self._voiceDevice.setWakeup(msg.data,400)

    def __init__(self,nodename) -> None:
        Node.__init__(self, nodename)
        logging.basicConfig(level = logging.INFO,format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        self._logger=logging.getLogger("CyberVoice")
        self._eventqueue=queue.Queue(-1)
        self._voiceDevice=R818Voice.VoiceDevice()
        self._voiceDevice.setWakeUpEventHandel(self.onWakeup)
        #讯飞AIUI接口
        self._asrAPI=XFAIUIAPI.AIUIApi(R818Voice.VoiceDevice.getDeviceIndex(),XUNFEIAPPID,XUNFEIKEY,"main_box")
        self._cyberdog=CyberDog.CyberDog2(self)
        #智谱清言接口
        self._llmodel=ZhiPuLLM.LLMApi(ZhiPuLLMAPPID)
        #保存对话记录日后训练用
        if self._savedialoglog:
            if os.path.exists(self._savedialogpath)==False:
                os.mkdir(self._savedialogpath)
        
        self._eventtask = threading.Thread(target=self.startListen,daemon=True)
        self._eventtask.start()
        macnamespace=manual.get_namespace()
        self._voicewake = self.create_subscription(msg_type=String, topic='/'+macnamespace+'/voicewakeup', callback=self.setwakeupwords, qos_profile=10)

    def startListen(self):
        self._islisten=True
        self._voiceDevice.startWakeUpEvent(MICTTYSPORT)
        while self._islisten:
            baseevevt=None
            try:
                baseevevt=self._eventqueue.get(timeout=1)  
            except queue.Empty:
                time.sleep(0.01)
                continue    
            except Exception as err:
                #崩溃退出
                return

            self._eventqueue.task_done()
            # print(baseevevt.eventType)
            if baseevevt.eventType=="R818WakeupEvent":
                event:R818Voice.WakeupEvent=baseevevt
                self._logger.info(f"唤醒,角度:{event.angle},唤醒词:{event.keywords},匹配度:{event.score},来自麦克风:{event.frommicindex},距离:{event.distence}")
                if event.score<500:
                    # 可能错误识别
                    continue
                #正在识别中
                if self._asrAPI.isRecognitioning():continue
                self.resetlogsession()
                #回应唤醒转动身体
                self._cyberdog.StopTTS()
                self._cyberdog.WakeUp()
                self._cyberdog.TrunAngle(event.angle)               
                #开始识别

                self._asrAPI.setSaveSession(self.getlogsession())
                self._asrAPI.startgetIntentionFromMic(self.onAsrResult,self.onNLPResult)

            elif baseevevt.eventType=="ASRResultEvent":
                #有了识别结果
                asrevent:VoiceBase.ASRResultEvent=baseevevt
                self._logger.info(f"语音识别,识别结果:{asrevent.asrres},结束:{asrevent.isfinish}")
                self._cyberdog.ShowAsrText(asrevent.asrres)
                #如果语音识别不带语义理解就直接调用大语言
                if self._isAsrWithNlp==False and asrevent.isfinish:
                    self._llmodel.startInputText(asrevent.asrres,self.onLLMResult,self.getlogsession())
            elif baseevevt.eventType=="NLPResultEvent":
                #意图识别结果                
                nlpevent:VoiceBase.NLPResultEvent=baseevevt
                self._logger.info(f"意图识别,语音:{nlpevent.asrtext},有意图:{nlpevent.matchintent},意图:{nlpevent.intent},回复:{nlpevent.answer}")
                if nlpevent.matchintent:
                    if len(nlpevent.answer)>0:
                        self._cyberdog.PlayTTS(nlpevent.answer)
                    self._cyberdog.RunTask(nlpevent.intent)
                else:
                    if self._llmodel.isProcessing()==False:
                        self._llmodel.startInputText(nlpevent.asrtext,self.onLLMResult,self.getlogsession())
            elif baseevevt.eventType=="LLMResultEvent":
                #大语言模型的返回
                llmevent:VoiceBase.LLMResultEvent=baseevevt
                self._logger.info(f"大语言模型回复,回复:{llmevent.answer},有意图:{llmevent.matchintent},意图:{llmevent.intent},结束:{llmevent.isfinish}")
                if len(llmevent.answer)>0:
                    self._cyberdog.PlayTTS(llmevent.answer)
                if llmevent.matchintent:
                    self._cyberdog.RunTask(llmevent.intent)


    def stopListen(self):
        self._islisten=False
        self._eventtask.join()
        self._voiceDevice.stopWakeUpEvent()
        self._asrAPI.cancelgetIntentionFromMic()

    def __del__(self):
        self.stopListen()
