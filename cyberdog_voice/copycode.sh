#!/bin/bash

cyberdog2_ip='192.168.3.143'

echo 正在复制文件

sshpass -p 123 scp -r ../cyberdog_voice mi@${cyberdog2_ip}:/home/mi/Downloads/

echo 全部安装完成