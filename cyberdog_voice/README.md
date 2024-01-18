# 光之造的铁蛋2的语音改进
## 1.特点
1.使用讯飞六麦克风矩阵实现语音唤醒，回声消除，声源定位

2.使用讯飞AIUI平台进行语音识别以及初步意图识别

3.使用智谱清言开放平台进行大语言模型语义识别

## 2.安装

下载本仓库并用命令行安装依赖
安装依赖组件时需要安装pyaudio的编译环境

```
sudo mv /etc/mr813_version /etc/mr813_version.bak

sudo apt-get install pip3

sudo apt-get install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0

pip3 install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple

sudo mv /etc/mr813_version.bak /etc/mr813_version
```

## 2.查找设备

cyberdog 通过  dmesg | grep tty* 查看串口设备名字
windows通过  ls -l /dev/ttyUSB* 查找串口
通过命令拥有硬件操作权限
```
sudo usermod -aG dialout $USER
```

通过命令查看录音设备

```
cat /proc/asound/cards
```

设置唤醒词
ros2 topic pub /mi_desktop_48_b0_2d_7b_09_01/voicewakeup std_msgs/String "{data: 'ze2 ta3 ze2 ta3'}" --once

测试机器狗的各种动作
ros2 service call /mi_desktop_48_b0_2d_7b_09_01/motion_result_cmd protocol/srv/MotionResultCmd "{motion_id: 111}" --once