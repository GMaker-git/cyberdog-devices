#!/bin/bash

colcon build

cyberdog2_ip='192.168.3.143'

echo 正在复制文件

sshpass -p 123 scp -r install mi@${cyberdog2_ip}:/home/mi/Downloads/CyberVoiceNode/
sshpass -p 123 scp -r cybervoice.service mi@${cyberdog2_ip}:/home/mi/Downloads/CyberVoiceNode/
sshpass -p 123 scp -r InstallService.sh mi@${cyberdog2_ip}:/home/mi/Downloads/CyberVoiceNode/

# ssh mi@${cyberdog2_ip} -t '/bin/bash /home/mi/Downloads/CyberVoiceNode/InstallService.sh'

echo 全部安装完成