from timeit import Timer
import websocket
import base64
import hashlib
import json
import time
import uuid
import pyaudio
import audioop
import threading
import wave
import logging
from . import VoiceBase

class AIUIApi(VoiceBase.AsrAPI):
    '''
    讯飞AIUI
    '''
    #应用账号
    _app_id:str
    _api_key:str
    #场景
    _scene:str
    #录音设备序号
    _micindex:int=0
    #语言识别结果
    _asrcallbackhandel=None
    #语义识别结果
    _nlpcallbackhandel=None
    #正在发送麦克风数据
    _issendingmicdata:bool=False 
    #是否取消回调
    _iscancelcallback:bool=False 
    #接口WS
    _aiuiws:websocket.WebSocketApp=None
    #nlp已经回复
    _isnlpcallback:bool=False
    #日志记录
    _logger =None

    #最后保存录音的文件
    _lastsession:str=None

    def _issavesession(self):
        return self._lastsession!=None

    def _getsavewavfile(self):
        return self._lastsession+".wav"
    
    def getsaveasrfile(self):
        return self._lastsession+".asr.txt"

    def _startsenddatafrommic(self):
        self._logger.debug("开始发送麦克风数据")
        chunk = 16000  # 每次读取的音频流块大小
        sample_format = pyaudio.paInt16  # 音频采样格式
        channels = 1  # 声道数
        fs = 16000  # 采样频率
        blocksize=int(chunk/4) #每次拿的数据量（1/4秒）
        _audiodevice=pyaudio.PyAudio()
        wf=None
        if self._issavesession():
            filename = self._getsavewavfile()
            wf = wave.open(filename, 'wb')
            wf.setnchannels(channels)
            wf.setsampwidth(_audiodevice.get_sample_size(sample_format))
            wf.setframerate(fs)
        _audiostream = _audiodevice.open(format=sample_format,
                channels=channels,
                rate=fs,
                frames_per_buffer=chunk,
                input_device_index=self._micindex,  # 指定设备索引
                input=True)
        rmsline=300  #静音阈值
        self._issendingmicdata=True
        novoicetime=0
        for i in range(0, 60):#最多拿40次也就是15秒数据    
            if not self._issendingmicdata:break #云端返回停止        
            voicedata = _audiostream.read(blocksize)
            if wf:wf.writeframes(voicedata)
            rms=audioop.rms(voicedata, 2)    
            print(f"麦克风静音度:{rms}")        
            if i>5:
                if rms>rmsline:
                    novoicetime=0
                else:
                    # self._logger.debug(f"麦克风静音度:{rms}")
                    novoicetime+=1
            if novoicetime>1: break #超过两次也就是500ms没有声音            
            self._aiuiws.send(voicedata)     
        #结束发送
        self._aiuiws.send(bytes("--end--".encode("utf-8")))
        if wf:wf.close()
        _audiostream.stop_stream()
        _audiostream.close()
        _audiodevice.terminate()
        self._logger.debug("结束发送麦克风数据")

    def _stopsenddatafrommic(self):
        self._issendingmicdata=False

    def _wsopened(self,ws):
        th=threading.Thread(target=self._startsenddatafrommic)
        th.start()

    def _wsclosed(self, ws, code, reason=None):
        self._stopsenddatafrommic()
        self._aiuiws=None
        if code != 1000:
            self._logger.error("连接异常关闭,code:" + str(code) + " .reason:" + str(reason))

    def _wsmessage(self,ws, m):
        if self._iscancelcallback:return
        s = json.loads(str(m))
        # print(s)
        p = pyaudio.PyAudio()
        if s['action'] == "started":
            pass   

        elif s['action'] == "result":
            #{'action': 'result', 'data': {'sub': 'iat', 'auth_id': 'd3e2439c72042205c2a8a82ce7a19609', 'text': '永远也给我发了', 'result_id': 7, 'is_last': False, 'is_finish': False, 'json_args': {'language': 'zh-cn', 'accent': 'mandarin'}}, 'sid': 'awa037b56d3@dx000118e45f4ca10c00', 'code': '0', 'desc': 'success'}
            #{'action': 'result', 'data': {'sub': 'nlp', 'auth_id': 'd3e2439c72042205c2a8a82ce7a19609', 'intent': {'answer': {'text': '好的', 'type': 'T'}, 'category': 'OS10284065294.NAV2P', 'data': {'result': None}, 'intentType': 'custom', 'rc': 0, 'semantic': [{'entrypoint': 'ent', 'hazard': False, 'intent': 'showpos', 'score': 1, 'slots': [{'begin': 4, 'end': 7, 'name': 'postype', 'normValue': '芭蕾舞', 'value': '芭蕾舞'}], 'template': '表演一个{postype}'}], 'semanticType': 0, 'service': 'OS10284065294.NAV2P', 'sessionIsEnd': False, 'shouldEndSession': False, 'sid': 'awa037b6bbd@dx000118e45fd3a10c00', 'state': None, 'test': '', 'text': '表演一个芭蕾舞', 'uuid': 'awa037b6bbd@dx000118e45fd3a10c00', 'vendor': 'OS10284065294', 'version': '4.0', 'voice_answer': [{'content': '好的', 'type': 'TTS'}]}, 'result_id': 1, 'is_last': True, 'is_finish': True}, 'sid': 'awa037b6bbd@dx000118e45fd3a10c00', 'code': '0', 'desc': 'success'}
            #无内容时{'action': 'result', 'data': {'sub': 'iat', 'auth_id': 'd3e2439c72042205c2a8a82ce7a19609', 'text': '', 'result_id': 1, 'is_last': True, 'is_finish': False, 'json_args': {'language': 'zh-cn', 'accent': 'mandarin'}}, 'sid': 'awa03c01ff2@dx000118e624aea10d00', 'code': '0', 'desc': 'success'}
            #无内容时{'action': 'result', 'data': {'sub': 'nlp', 'auth_id': 'd3e2439c72042205c2a8a82ce7a19609', 'intent': {}, 'result_id': 1, 'is_last': True, 'is_finish': True}, 'sid': 'awa03c01ff2@dx000118e624aea10d00', 'code': '0', 'desc': 'success'}
            data = s['data']
            if data['sub'] == "iat":
                #语音识别结果
                if self._asrcallbackhandel!=None:
                    if ("text" in data) and len(data["text"])>0:                    
                        event:VoiceBase.ASRResultEvent=VoiceBase.ASRResultEvent()
                        event.isfinish=data["is_last"]
                        event.asrres=data["text"]
                        self._asrcallbackhandel(event)     

            elif data['sub'] == "nlp":
                #语义理解结果           
                if self._nlpcallbackhandel!=None:
                    if self._isnlpcallback==False:   
                        self._isnlpcallback=True  
                        intent = data['intent']
                        if 'rc' in intent:
                            event:VoiceBase.NLPResultEvent=VoiceBase.NLPResultEvent()
                            event.matchintent=intent['rc'] == 0
                            event.asrtext=intent['text']
                            if event.matchintent:
                                event.answer=intent['answer']['text']
                                event.intent={}
                                semantic=intent["semantic"][0]
                                event.intent["action"]=semantic["intent"]
                                for slot in semantic["slots"]:
                                    event.intent[slot["name"]]=slot["value"]
                            self._nlpcallbackhandel(event)
            # elif data['sub'] == "tts":
            #     # TODO 播报pcm音频
            #     print("tts: " + base64.b64decode(data['content']).decode())
        elif s['action'] == "vad":
            #云端判断断句
            self._stopsenddatafrommic()
        else:
            self._logger.debug(s)
        
        #保存历史数据以便以后训练
        if self._issavesession():
            filename=self.getsaveasrfile()
            with open(filename, mode='at',encoding='utf-8') as txtfile:
                txtfile.write(m)
    
    def _wserror(self,ws, error):
        self._stopsenddatafrommic()
        self._logger.error(f"WSError occurred: {error}")

    def _get_auth_id(self):
        mac = uuid.UUID(int=uuid.getnode()).hex[-12:]
        return hashlib.md5(":".join([mac[e:e + 2] for e in range(0, 11, 2)]).encode("utf-8")).hexdigest()
    
    def __init__(self, micindex:int, app_id:str,apikey:str,scene:str):
        self._logger=logging.getLogger("XFAIUI")
        self._app_id=app_id
        self._api_key=apikey
        self._micindex=micindex
        self._scene=scene
        self._issendaudio=False

    def isRecognitioning(self):
        return self._issendingmicdata
    
    def setSaveSession(self,session:str):
        self._lastsession=session

    def startgetIntentionFromMic(self,asrresulthandel,nlpresulthandel):
        self._asrcallbackhandel=asrresulthandel
        self._nlpcallbackhandel=nlpresulthandel
        curTime = int(time.time())
        auth_id = self._get_auth_id()
        param_iat = """{{
            "auth_id": "{0}",
            "result_level": "plain",
            "data_type": "audio",
            "aue": "raw",
            "scene": "{1}",
            "sample_rate": "16000",
            "close_delay": "100",
            "cloud_vad_eos":"1000",
            "vad_info":"end",
            "context": "{{\\\"sdk_support\\\":[\\\"iat\\\",\\\"nlp\\\",\\\"tts\\\"]}}"
        }}"""
        # cloud_vad_eos=停顿多久算结束
        # close_delay=延迟关闭连接
        self._isnlpcallback=False
        param = param_iat.format(auth_id,self._scene).encode(encoding="utf-8")
        paramBase64 = base64.b64encode(param).decode()
        checkSumPre = self._api_key + str(curTime) + paramBase64
        checksum = hashlib.md5(checkSumPre.encode("utf-8")).hexdigest()
        connParam = "?appid=" + self._app_id + "&checksum=" + checksum + "&param=" + paramBase64 + "&curtime=" + str(curTime) + "&signtype=md5"
        self._aiuiws = websocket.WebSocketApp(url="ws://wsapi.xfyun.cn/v1/aiui" + connParam,header={"Origin", "https://wsapi.xfyun.cn"},subprotocols=['chat'],on_open=self._wsopened, on_message=self._wsmessage, on_error=self._wserror, on_close=self._wsclosed)
        self._eventtask = threading.Thread(target=self._aiuiws.run_forever,daemon=True)
        self._iscancelcallback=False
        self._lastrecordname=str(time.time())
        self._eventtask.start()
    
    def cancelgetIntentionFromMic(self):
        self._iscancelcallback=True
        self._stopsenddatafrommic()

    def __del__(self):
        self._stopsenddatafrommic()