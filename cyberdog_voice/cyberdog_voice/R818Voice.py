import serial
import json
import io
import time
import threading
import pyaudio
import logging
from . import VoiceBase

class WakeupEvent(VoiceBase.VoiceEvent):    
    isvalid:bool=False
    frommicindex:int=0
    score:float=0
    angle:float=0
    keywords:str=""
    distence:float=0
    def __init__(self,jsonobj):
 # {
# 	"start_ms": 53005930,
# 	"end_ms": 53006470,
# 	"beam": 0,
# 	"physical": 0,
# 	"score": 701.0,
# 	"power": 1456558.125,
# 	"angle": 11.0,
# 	"keyword": "ze2 ta3"
# }   
    
        if "content" in jsonobj:
            if "info" in jsonobj["content"]:
                try:
                    info=json.loads(jsonobj["content"]["info"])["ivw"]
                    self.frommicindex=info["physical"]
                    self.score=info["score"]
                    self.angle=info["angle"]
                    self.keywords=info["keyword"]
                    self.distence=info["power"]
                    self.isvalid=True
                except Exception as e:
                    print(e)
                    self._isvalid=False
                    
    @property
    def eventType(self)->str:
        return "R818WakeupEvent"

        

class VoiceDevice:
    # 最后设备活动时间
    _lastalivetime:int=0
    # 通讯的串口
    _serial_port:serial.Serial=None
    #异步任务
    _eventtask:threading.Thread=None
    #正在监听线程
    _isListenEvent:bool=False
    #回调信息
    _callbackhandel=None

    #日志记录
    _logger = logging.getLogger("R818Voice")

    def __init__(self) -> None:
        pass

    def _r818_sumdata(self,data,length):
        sumddata=0
        for i in range(0,length-1):
            sumddata+=data[i]
        return (256-(sumddata & 0xff)) & 0xff
    

    def _sendmsg(self,msgtype,dataid,msg):
        databack = bytearray(b'\xa5\x01')
        databack.append(msgtype)
        datalen=len(msg)
        databack.append(datalen & 0xff)
        databack.append((datalen >>8) & 0xff)
        databack.append(dataid & 0xff)
        databack.append((dataid >>8) & 0xff)
        databack.extend(msg)
        databack.append(0)
        databack[datalen+7]=self._r818_sumdata(databack,datalen+8)
        self._serial_port.write(databack)

    
    def _backmsg(self,dataid):
        backmsg = bytearray(b'\xa5\x00\x00\x00')
        self._sendmsg(0xff,dataid,backmsg)

    def _serial_data_receive(self):
        databuffers=b''
        while self._isListenEvent:
            if self._serial_port.in_waiting==0:
                time.sleep(0.01)
                continue

            buffer=self._serial_port.read(100)  # 读取最多100个字节的数据
            databuffers+=buffer
            if databuffers[0]!=0xa5:
                self._logger.warning(f"HeadError: {databuffers}")
                # 向后查找包头
                findhead=0
                for i in range(1,len(databuffers)):
                    if databuffers[i]==0xa5:
                        findhead=i
                        break
                if findhead>0:
                    databuffers=databuffers[findhead:]
                else:
                    databuffers=b''
                    continue
                                
            if len(databuffers)<7:
                #self._logger.warning(f"R818 LengthError: {databuffers}")
                #长度不够
                continue            
            datatype=databuffers[2]
            datalength=databuffers[4]<<8 | databuffers[3]
            if len(databuffers)<datalength+8:
                # print(f"R818 DataLengthError: {data}")
                if len(databuffers)>500:
                    self._logger.warning(f"R818 DataBufferError: {databuffers}")
                    databuffers=b''
                continue
            dataid=databuffers[6]<<8 | databuffers[5]            
            datasum=self._r818_sumdata(databuffers,datalength+8)
            if datasum!=databuffers[datalength+7]:
                self._logger.warning(f"R818 DataCheckError: {databuffers}")
                databuffers=b''
                continue
            # print(databuffers)   
            # print(datatype) 
            self._lastalivetime=time.time()  
            #非确认消息都需要确认
            if datatype!=0xff:
                self._backmsg(dataid)     
            if datatype==4:
                #关键字唤醒回调
                if self._callbackhandel!=None:
                    jsondata=json.loads(databuffers[7:datalength+7])
                    event=WakeupEvent(jsondata)
                    if event.isvalid:
                        self._callbackhandel(event)
            elif datatype==1:
                self._logger.info(f"R818握手")
                      
            databuffers=b''

    def stopWakeUpEvent(self):
        """
        结束监听唤醒事件
        """
        if self._eventtask==None:return
        if not self._isListenEvent: return
        self._isListenEvent=False
        self._eventtask.join()
        self._serial_port.close()

    def setWakeup(self,keywords:str,score:int)->bool:
        """
        此方法只需要设置一次，设备会自己保存，设置会重启设备很耗时，需要开启监听唤醒才能设置成功
        keywords:设置关键字(类似xiao3 fei1拼音加声调)
        score:匹配度一般800,需要更敏感设置到700,如果600以下很可能误识别,900以上远距离会识别不到
        mictype:麦克风类型mic4:线性4麦,mic6:线性6麦,mic6_circle:环形6麦
        返回是否设置成功
        """    
        datajson="{\"type\": \"wakeup_keywords\",\"content\": {\"keyword\": \""+keywords+"\",\"threshold\": \""+str(score)+"\"}}"
        msgdate=datajson.encode("UTF-8")
        self._sendmsg(0x05,10,msgdate)
        return True

    @staticmethod
    def getDeviceIndex()->int:
        """
        获取麦克风的序号
        """
        p = pyaudio.PyAudio()
        device_index = 0
        # 列出所有音频输入设备
        for i in range(p.get_device_count()):
            device_info = p.get_device_info_by_index(i)
            if device_info['maxInputChannels'] > 0:  # 判断是否为输入设备（即麦克风）
                print(f'Device {i}: {device_info["name"]}')
                if device_info["name"].startswith("XFM-DP"):
                    device_index=i
                    break
        return device_index
    
    def isDeviceAlive(self)->bool:
        """
        获取设备是否在线
        """
        return time.time()-self._lastalivetime>2

    def isEnableListenEvent(self)->bool:
        """
        获取是否在监听状态
        """
        return self._isListenEvent

    def startWakeUpEvent(self,portname:str):
        """
        开始监听唤醒事件
        portname=串口号
        """
        self.stopWakeUpEvent()
        self._serial_port = serial.Serial(portname, 115200, timeout=0.02)
        if self._eventtask==None:
            self._eventtask = threading.Thread(target=self._serial_data_receive,daemon=True)
        self._isListenEvent=True
        self._eventtask.start()        
        self._logger.info("startWakeUpEvent")

    def setWakeUpEventHandel(self,callback):
        """
        设置回调函数，将返回Json
        portname=串口号
        """
        self._callbackhandel=callback


    def __del__(self):
        self.stopWakeUpEvent()