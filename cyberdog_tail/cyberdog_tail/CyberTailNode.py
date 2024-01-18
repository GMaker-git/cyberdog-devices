#!/usr/bin/python3
#
# Copyright (c) 2023 Gmaker
#
# Sway tail while dog wakeup
# Open source on https://github.com/GMaker-git

import os
import time

from bluepy.btle import Scanner, DefaultDelegate,UUID, Peripheral

import traceback
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from . import manual

class CyberTailNode(Node, DefaultDelegate):
    """ROS2 node for cyberdog_tail."""

    #节点初始化
    def __init__(self, node_name: str):
        Node.__init__(self, node_name)
        DefaultDelegate.__init__(self)
        self.__logger = self.get_logger()  # ROS2 logger
        self.__logger.info('cybertailV1.0 start')
        #是否已连接硬件
        self.bleisConnected=False
        #蓝牙连接
        self.bleConn=None
        #蓝牙角色
        self.bleChar=None
        #当前状态
        self.deviceConnectting=False
        #最后连接的尾巴地址
        #self.writelastconnectMac("b0:a7:32:c5:2d:22")
        self.lastConnectTail=self.readlastconnectMac()
        #最后摇摆时间
        self.LastSwayTime=0
        #最后尝试连接时间
        self.TryConnectTimes=0
        #订阅语音唤醒消息
        macnamespace=manual.get_namespace()
        self.__logger.info("Mac-namespace:%s"%(macnamespace))
        self.sub = self.create_subscription(Bool, '/'+macnamespace+'/dog_wakeup', self.wakeup_callback, 10)
        #打开定时器
        self.tmr = self.create_timer(3, self.timer_callback)
        self.scanner = Scanner().withDelegate(self)

    #发现设备
    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            self.__logger.info("Discovered device:%s"%(dev.addr))
        elif isNewData:
            self.__logger.info("Received new data from:%s"%(dev.addr))

    #定时器触发
    def timer_callback(self):
        #已经连上不用扫描
        if self.bleisConnected==True : 
            #self.startswaytail(True)
            return
        
        #如果已经在扫描
        if self.deviceConnectting==True : return
        #已经尝试过很多次
        if self.TryConnectTimes>100 : return        

        self.__logger.info('connect device[%d]'%(self.TryConnectTimes))        

        self.deviceConnectting=True

        #优先连接上次成功的设备
        if self.lastConnectTail!=None:
            self.__logger.info("try connect last mac:%s"%(self.lastConnectTail))
            self.tryconnecttail(self.lastConnectTail)

        #连接上次没成功
        if self.bleisConnected!=True:
            #扫描尾巴
            self.scanandconnecttails()

        self.deviceConnectting=False

        #重试次数
        if self.bleisConnected!=True:
            self.TryConnectTimes+=1
        else:
            self.TryConnectTimes=0
    
    #尝试连接尾巴
    def tryconnecttail(self,macadr:str):
        try:
            self.bleConn = Peripheral(macadr)
            bleChars=self.bleConn.getCharacteristics(uuid="b6e8fc9b-6033-1a83-fb61-c5e160cf6874")
            if len(bleChars)>0 :
                self.bleChar=bleChars[0]
                self.bleisConnected=True
                if macadr!=self.lastConnectTail:
                    self.lastConnectTail=macadr
                    self.writelastconnectMac(macadr)
                self.__logger.info("Tail connect success:%s"%str(macadr))
                self.startswaytail(False)
            else:
                self.bleConn.disconnect()
                self.bleConn=None
        except:
            self.__logger.info("Tail connect error:%s"%str(macadr))
            self.bleisConnected==False
            self.bleConn=None
            self.bleChar=None
        return self.bleisConnected
    
    #扫描并且尝试连接尾巴
    def scanandconnecttails(self):
        devices=None
        try:
            scanner = Scanner().withDelegate(self)
            devices = scanner.scan(5.0)
        except Exception as err:
            self.__logger.info('scan error')
            self.__logger.error(traceback.format_exc())
            return False

        for dev in devices:
            #print("Device %s (%s), RSSI=%d dB" % (dev.addr, dev.addrType, dev.rssi))
            for (adtype, desc, value) in dev.getScanData():
                print("  %s = %s" % (desc, value))
                #狗尾服务
                if value =="275edae9-f311-bbc2-638b-d76122ef9ce2":
                    if self.tryconnecttail(dev.addr):
                        break
            if self.bleisConnected==True: break
        return self.bleisConnected

    #断开连接
    def disconnectBle(self):
        if self.bleConn!=None:
            try:
                self.bleConn.disconnect()
            except:
                self.__logger.info('cybertail disconnect error')

        self.bleisConnected=False
        self.bleConn=None
        self.bleChar=None
        self.TryConnectTimes=0
        self.deviceConnectting=False

    #读取最后连接成功的尾巴
    def readlastconnectMac(self):
        try:
            with open('tailmac.txt', mode='r', encoding='utf8') as f:
                return f.read()
        except:
            return None
    
    #更新最后连接的尾巴
    def writelastconnectMac(self,macaddr):
        try:
            with open('tailmac.txt', mode='w', encoding='utf8') as f:
                f.write(macaddr)
        except:
            self.__logger.info('cyberdog write mac error')
        
    #通知尾巴动作
    def startswaytail(self,tryreconnect:bool):
        if self.bleisConnected !=True: return
        self.LastSwayTime=int(time.time())
        #摆动总时间(毫秒)
        swaytimemsec:int=2000
        #延迟（毫秒）
        swaydelay:int=80

        swaytimemsecbt = swaytimemsec.to_bytes(length=2, byteorder='big', signed=False)
        swaydelaybt = swaydelay.to_bytes(length=2, byteorder='big', signed=False)
        data = bytes([swaytimemsecbt[0],swaytimemsecbt[1],swaydelaybt[0],swaydelaybt[1]])
        try:
            self.bleChar.write(data, withResponse=False)
            self.__logger.info('cybertail write success')
        except:
            self.__logger.info('cybertail write error')
            if tryreconnect==True:
                self.disconnectBle()
                self.tryconnecttail(self.lastConnectTail)

    #语音话题有消息
    def wakeup_callback(self, msg):
        #语音唤醒了msg.Data
        self.__logger.info('cyberdog wakeup')
        self.startswaytail(True)
        
    #节点关闭
    def __del__(self):
        self.disconnectBle()
        self.tmr.cancel()
        self.__logger.info('cybertail stop')
    
