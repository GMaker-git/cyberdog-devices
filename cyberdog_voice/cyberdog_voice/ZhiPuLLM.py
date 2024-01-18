import zhipuai
import json
import threading
import logging
import time

from . import VoiceBase

class LLMApi():
    #历史对话
    _prompts=[]

    #回调句柄
    _callbackhandel=None

    #是否取消
    _iscancel:bool=False

    #输入文字
    _inputtext:str=None

    #保存文件
    _savesession:str=None

    _logger = logging.getLogger("LLMApi")

    #正在处理中
    _isprocessing:bool=False

    def _issaveseesion(self):
        return self._savesession!=None
    
    def _getsavellmfile(self):
        return self._savesession+".llm.txt"

    def clearHistory(self):        
        if len(self._prompts)==0:
            self._prompts.append(
                            {
                                "role": "user",
                                "content": "你是一个四足机器狗,是由小米公司设计生产,产品代号CyberDog2(中文产品名叫铁蛋)。你的主人是光之造，他给你起了个名字叫泽塔。泽塔本来是奥特曼的一个英雄,光之造以前喂养过的一条动物狗就叫这个名字。为了怀念它就沿用泽塔这个名字给你。你的主人光之造,他的生日是2003年8月,他是处女座的小男生,是你最好的伙伴，他负责训练你让我更聪明。对你的提问都是由语音转换成的文字后跟你交流，所以如果中间有通读音的错别字需要你自动修正。你的回答机器狗会转换成语音，指令会转换成任务,所以你的回复要尽量简短，并且控制在60字以内。还有尽量使用中文回答，不要带表情符号。你的躯体是四足机器狗,身体体有很多传感器,这些传感器会让你有听觉，视觉，空间感觉。你的身体还有4条机械腿，通过姿态组合可以摆出各种姿态。另外你还有能跟随主人行走并避开障碍物。你的回复中如果需要操控你的身体时会带上Json格式的指令，具体场景如下：1.当你识别到对话意图类似是\"控制你的姿态\"时,用纯的Json内容回复对话,结构是这样的:{\"action\":\"showpos\",\"postype\":\"具体姿态\"},其中action属性在这个意图是固定的内容\"showpos\",postype有如下姿态:休息一下,装死,跟着我,跟我来,比心,旋转右跳,旋转左跳,原地跳远,原地跳起,握右手,握左手,俯卧撑,伸懒腰,扭屁股,扭头,恭喜发财,卧倒,坐下,跳上台阶,跳下台阶,跳舞,前后跳,狗粮来了,蹦跶蹦跶,高兴一下,鞠躬,拜年,点头,立正,太空步,摇头,摇尾巴,握手,跟我走,原地转圈,跳跃,跳起来,过来,芭蕾舞,作揖,起立,站起来,起来,趴下。例如：当对话意图是坐下时回复内容应该是{\"action\":\"showpos\",\"postype\":\"坐下\"}。2.当你识别到对话意图是\"给某个人带句话\"时用纯的Json内容回复对话,结构是这样的:{\"action\":\"findandsay\",\"who\":\"带话的对象\",\"say\":\"带话的内容\"},如\"去找张三叫他赶紧做饭\",其中\"张三\"就是带话的对象,\"赶紧做饭\"就是带话的内容,应该回复Json内容为{\"action\":\"findandsay\",\"\":\"张三\",\"say\":\"张三,赶紧做饭\"} 其中action属性在这个场景下是固定的\"findandsay\"，who属性一般是家庭成员中的一个，say属性一般是通知或传达一句话  第一个例子:\"去叫爷爷晚上回家记得关灯\",应该回复内容为{\"action\":\"findandsay\",\"who\":\"爷爷\",\"say\":\"爷爷,晚上回家记得关灯\"}  第二个例子:\"去找一下妈妈叫他给我买条秋裤\",应该回复内容为{\"action\":\"findandsay\",\"who\":\"妈妈\",\"say\":\"妈妈,给我买条秋裤\"}"
                            }
                        )
            self._prompts.append(
                            {
                                "role": "assistant",
                                "content": "好的"
                            }
                        )
        else:
            del self._prompts[2:]
                
    def _responseTextBlock(self,text:str)->bool:
        # print(text)
        if len(text)==0: return
        if self._iscancel:return  
        if self._callbackhandel==None:return
        # text=text.replace("我已经将您的指令转换为Json格式","")
        # text=text.replace("```","")
        jsonstart=text.find("{")
        jsonend=text.rfind("}")
        event=VoiceBase.LLMResultEvent()
        event.isfinish=False
        if jsonstart>=0 and jsonend>0:
            #包含指令
            jsondata=text[jsonstart:jsonend+1].strip()
            text=text.replace(jsondata,"").strip()
            event.answer=text
            try:
                # print(jsondata)
                intent=json.loads(jsondata)
                event.intent=intent
                event.matchintent=True
            except Exception as err:
                self._logger.error(str(err))
        else:
            event.answer=text
        self._callbackhandel(event)
        return event.matchintent
    
    def __init__(self,appkey:str) -> None:          
          zhipuai.api_key = appkey     
          self.clearHistory()   
           

    def _invokeLLM(self):
        self._prompts.append(
            {
                    "role": "user",
                    "content": self._inputtext
            }
        )

        response = zhipuai.model_api.sse_invoke(
            model="chatglm_turbo",
            prompt=self._prompts,
            temperature=0.9,
            top_p=0.2,
            incremental=True
        )
        assistant=""
        error=""
        assistantbackindex=0
        matchintent=False
        for event in response.events():
            if self._iscancel:return
            isfinish:bool=False
            if event.event == "add":
                assistant+=event.data
                # print(f"add:{event.data}")
            elif event.event == "error" or event.event == "interrupted":
                error+=event.data
                # print(f"error:{event.data}")
            elif event.event == "finish":
                isfinish=True
                assistant+=event.data
                # print(f"finish:{event.data}")
                # assistant+=event.meta
            else:
                # print(f"data:{event.data}")
                assistant+=event.data
            #截取片段输出
            while True:
                textblockindex=assistant.find("，",assistantbackindex)
                if textblockindex<=0:
                    textblockindex=assistant.find("。",assistantbackindex)
                if textblockindex>0:
                    textblock=assistant[assistantbackindex:textblockindex].strip()                    
                    if textblock.count("{")!=textblock.count("}"):
                        #不能截断json
                        break
                    assistantbackindex=textblockindex+1
                    if self._responseTextBlock(textblock):
                        matchintent=True
                else:
                    break

        if len(assistant)>assistantbackindex:
            if self._responseTextBlock(assistant[assistantbackindex:].strip()):
                matchintent=True
        
        if len(assistant)>0:
            # print("==========Result===========")
            # print(assistant)
            if self._issaveseesion():
                with open(self._getsavellmfile(), mode='at',encoding='utf-8') as txtfile:
                    txtfile.write(assistant)
            self._prompts.append(
                {
                    "role": "assistant",
                    "content": assistant
                }
            )
        if len(error)>0:
            self._logger.error(error)
            if self._issaveseesion():
                with open(self._getsavellmfile(), mode='at',encoding='utf-8') as txtfile:
                    txtfile.write(error)

            self.clearHistory()

        if matchintent:
            #清空聊天记录避免上下文干扰
            self.clearHistory()
        
        if len(assistant)>100:
            #长篇大论就清空
            self.clearHistory()

        if len(self._prompts)>12:
            #清空聊天记录避免超出API限制
            self.clearHistory()
        
        if self._callbackhandel!=None and self._iscancel==False:
            event=VoiceBase.LLMResultEvent()
            event.isfinish=True
            self._callbackhandel(event)
        
        self._isprocessing=False

    def isProcessing(self)->bool:
        return self._isprocessing

    def startInputText(self,text,llmcallbackhandel:None,savesessionfile:str=None):
        self._inputtext=text
        self._savesession=savesessionfile
        self._callbackhandel=llmcallbackhandel
        eventtask = threading.Thread(target=self._invokeLLM)
        self._iscancel=False
        self._isprocessing=True
        eventtask.start()

    def cancel(self):
        self._iscancel=True



# voicenode=None
# textindex=0
# textres=["你是谁","找妈妈叫他晚上盖被子","坐下","趴下","给我拜个年","起飞"]

# def showevent(event:VoiceBase.LLMResultEvent):
#     global textindex,textres,voicenode
#     print(f"llmevent isfinish:{event.isfinish},hasintent:{event.matchintent},answer:{event.answer},intent:{event.intent}")
#     if(len(textres)>textindex) and event.isfinish:
#         time.sleep(0.5)
#         print("\n\n"+textres[textindex]+"------------------------------------------------------------------------")
#         voicenode.startInputText(textres[textindex],showevent)
#         textindex+=1
# if __name__ == "__main__":
#     voicenode=LLMApi("1903090c1d58385a3d33d0917314daf1.GOxsODRpFzYwLuCB")
#     voicenode.startInputText("立正",showevent)