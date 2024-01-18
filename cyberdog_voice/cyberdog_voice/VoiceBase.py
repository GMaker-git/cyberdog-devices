
from abc import ABC, abstractmethod,abstractproperty


class VoiceEvent():
    '''
    事件抽象类
    '''
    @abstractproperty
    def eventType(self)->str:
        '''
        事件类型
        '''
        pass

class LLMResultEvent(VoiceEvent):
    '''
    大模型应答结果事件
    '''

    #是否识别到意图
    matchintent:bool=False
    #回应的文本
    answer:str=""
    #识别意图
    intent:object=None
    #是否结束了
    isfinish:bool=False

    @property
    def eventType(self)->str:
        return "LLMResultEvent"

class NLPResultEvent(VoiceEvent):
    '''
    语义识别结果事件
    '''

    #是否识别到意图
    matchintent:bool=False
    #识别的文本
    asrtext:str=""
    #识别意图
    intent:object=None
    #回答
    answer:str=""

    @property
    def eventType(self)->str:
        return "NLPResultEvent"

class ASRResultEvent(VoiceEvent):
    '''
    语音识别结果事件
    '''

    #是否结束
    isfinish:bool=False
    #识别的文本
    asrres:str=""

    @property
    def eventType(self)->str:
        return "ASRResultEvent"

class AsrAPI():
    @abstractmethod
    def startgetIntentionFromMic(self,asrresulthandel,nlpresulthandel):
        pass

    @abstractmethod
    def cancelgetIntentionFromMic(self):
        pass

