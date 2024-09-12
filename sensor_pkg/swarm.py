
# import required libraries
# pip3 install pymavlink pyserial

import cv2
import numpy as np
import time
import VisionCaptureApi
import math

import ReqCopterSim
import RflyRosStart

import sys
# 启用ROS发布模式

print(RflyRosStart.isLinux,RflyRosStart.isRosOk)
req = ReqCopterSim.ReqCopterSim()
VisionCaptureApi.isEnableRosTrans = True

StartCopterID = 1 # 初始飞机的ID号
TargetIP = req.getSimIpID(StartCopterID)
# 自动开启mavros
# if not (RflyRosStart.isLinux and RflyRosStart.isRosOk):
#     print('This demo can only run on with Ros')
#     sys.exit(0)

# 自动开启RosCore
# ros = RflyRosStart.RflyRosStart(StartCopterID,TargetIP)

# VisionCaptureApi 中的配置函数
vis = VisionCaptureApi.VisionCaptureApi()
vis.jsonLoad(jsonPath = "/home/sensor_pkg/swarmConfig.json") # 加载Config.json中的传感器配置文件
isSuss = vis.sendReqToUE4(
    0, "127.0.0.1"
)
vis.startImgCap()  # 开启取图循环，执行本语句之后，已经可以通过vis.Img[i]读取到图片了
print('Start Image Reciver')

vis.sendImuReqCopterSim(StartCopterID,TargetIP)
