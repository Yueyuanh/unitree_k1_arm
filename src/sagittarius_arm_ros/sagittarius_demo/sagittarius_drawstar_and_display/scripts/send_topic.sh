#!/bin/bash

echo -e "${Info} 请等待机械臂蜂鸣器停止鸣响后，按回车键开始程序，否则退出请输入：Ctrl + c    " 
echo -e "${Info} ***轨迹测试***" 

echo && stty erase '^H' && read -p "按回车键确定：" 


rostopic pub /start_topic std_msgs/String "data: 'start'" -1
