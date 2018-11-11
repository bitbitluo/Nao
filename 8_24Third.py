# encoding: UTF-8
from naoqi import ALProxy
import sys
import cv2
import numpy as np
import almath
import time
import math
sys.path.append('D:\python\Lib')


robotIP = "192.168.43.124"
PORT = 9559
_searchBallTimes = 0
stop1 = 0
# 0代表正常,非零则没有识别
landmarkFlag = 0
redballFlag = 0
# 步态和击球姿式
maxstepx = 0.04
maxstepy = 0.14
maxsteptheta = 0.3
maxstepfrequency = 0.6
stepheight = 0.02
torsowx = 0.0
torsowy = 0.0
_stepStatue = [["MaxStepX", 0.04], ["MaxStepY", 0.14], ["MaxStepTheta", 0.3], ["MaxStepFrequency", 0.6],
               ["StepHeight", 0.02], ["TorsoWx", 0], ["TorsoWy", 0]]
_stepStatue2 = [["MaxStepX", 0.02], ["MaxStepY", 0.14], ["MaxStepTheta", 0.3], ["MaxStepFrequency", 0.6],
               ["StepHeight", 0.02], ["TorsoWx", 0], ["TorsoWy", 0]]
PositionJointNamesR = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
PositionJointNamesL = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"]
golfPositionJointAnglesR1 = [1.4172073526193958, 0.061086523819801536, 1.6022122533307945, 1.4835298641951802,
                             0.038397243543875255, 0.12]
golfPositionJointAnglesR2 = [1.4172073526193958, 0.061086523819801536, 1.6022122533307945, 1.4835298641951802,
                             -0.538397243543875255, 0.12]
golfPositionJointAnglesR31 = [1.35787, 0.05760, 1.50098, 1.50971, 0.667945, 0.12]
golfPositionJointAnglesR101 = [1.3, 0.05760, 1.50098, 1.35, 0.767945, 0.12]
golfPositionJointAnglesR111 = [1.35787, 0.05760, 1.50098, 1.50971, -0.738397, 0.12]
golfPositionJointAnglesR11 = [1.35787, 0.05760, 1.50098, 1.50971, -0.538397, 0.12]
golfPositionJointAnglesR32 = [1.35787, 0.05760, 1.50098, 1.50971, -0.767944870877505, 0.12]        
golfPositionJointAnglesR12 = [1.35787, 0.05760, 1.50098, 1.50971, 0.767944870877505, 0.12]
golfPositionJointAnglesR42 = [1.02974, 0.24958, 1.61094, 1.10828, -0.43633, 0.12]
golfPositionJointAnglesR5 = [1.35787, 0.05760, 1.50098, 1.50971, 0.0, 0.6]  
golfPositionJointAnglesR6 = [1.35787, 0.05760, 1.50098, 1.50971, 0.0, 0.12] 
golfPositionJointAnglesR7 = [1.02629, 0.314159, 1.62907, 1.48342, 0.230058, 0.12]  
golfPositionJointAnglesR8 = [1.18857, -0.67719, 1.17635, 1.52193, 0.666716, 0.50]
golfPositionJointAnglesR9 = [1.47480, -0.17453, 1.18159, 0.41190, 0.10996, 0.12]
golfPositionJointAnglesR10 = [1.46084, 0.26005, -1.37008, -0.08901, -0.02792, 0.12]
GPositionJointNamesR = ["RElbowRoll", "RShoulderRoll", "RShoulderPitch", "RElbowYaw", "RWristYaw", "RHand"]
GgolfPositionJointAnglesR1 = [1.4835298641951802, 0.061086523819801536, 1.4172073526193958, 1.6022122533307945,
                              0.038397243543875255, 0.12]
GgolfPositionJointAnglesR11 = [1.4835298641951802, 0.061086523819801536, 1.4172073526193958, 1.6022122533307945,
                               -0.538397243543875255, 0.12]
GgolfPositionJointAnglesR2 = [1.4835298641951802, 0.061086523819801536, 1.1, 1.6022122533307945, 0.038397243543875255,
                              0.12]
GgolfPositionJointAnglesR3 = [1.4835298641951802, 0.061086523819801536, 1.4172073526193958, 1.6022122533307945,
                              0.767944870877505, 0.12]
GgolfPositionJointAnglesR4 = [1.03549, 0.314159, 1.66742, 0.971064, -0.980268, 0.12]
GgolfPositionJointAnglesR5 = [1.4835298641951802, 0.061086523819801536, 1.4172073526193958, 1.6022122533307945,
                              0.038397243543875255, 0.6]  # 松杆参数
GgolfPositionJointAnglesR6 = [1.4835298641951802, 0.061086523819801536, 1.4172073526193958, 1.6022122533307945,
                              0.038397243543875255, 0.04]  # 抓杆参数

# 身体姿式------------------------------------------------#
def GHitBall():
    if (alpha < 0):
        # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR0,0.2)
        motionPrx.angleInterpolationWithSpeed(GPositionJointNamesR, GgolfPositionJointAnglesR1, 0.2)
        time.sleep(1)
        motionPrx.angleInterpolationWithSpeed(GPositionJointNamesR, GgolfPositionJointAnglesR2, 0.2)
        time.sleep(0.5)
        motionPrx.angleInterpolationWithSpeed(GPositionJointNamesR, GgolfPositionJointAnglesR3, 0.2)
        time.sleep(1)
        motionPrx.angleInterpolationWithSpeed(GPositionJointNamesR, GgolfPositionJointAnglesR11, 0.6)

def StiffnessOn(proxy):
    proxy.stiffnessInterpolation("Body", 1, 1)
    print("stiff_set finished")
    tts.say("初始化完成")

def headtouch():
    while True:
        headTouchedButtonFlag = memoryProxy.getData("FrontTactilTouched")
        if headTouchedButtonFlag == 1.0:
            print "front head touched"
            tts.say("开始击球")
            break
    print("头部前传感器收到信号")

def zhuagan():
    while True:
        # RighthandTouchedFlag = memoryProxy.getData("HandRightRightTouched")
        headTouchedmidlleFlag = memoryProxy.getData("MiddleTactilTouched")
        if headTouchedmidlleFlag == 1.0:
            print "right hand touched"
            tts.say("请给我一个球杆，好吗")
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR5, 0.4)
            time.sleep(5)
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR6, 0.1)
            time.sleep(3)
            # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.4)
            """  motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.4)
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR1,0.4)"""
            break
            print("抓杆完成")  # 待修改

def shougang():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch") 
    times.append([1, 2, 3, 4])  
    keys.append([0, 0, 0, 0])

    names.append("HeadYaw")  # Z轴动   
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LAnklePitch")  # 脚踝Z轴
    times.append([1, 2, 3, 4])
    keys.append([-0.349794, -0.349794, -0.349794, -0.349794])

    names.append("LAnkleRoll")  # 脚踝X轴
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LElbowRoll")  # 肘Z轴
    times.append([1, 2, 3, 4])
    keys.append([-0.321141, -0.321141, -1.1, -1.1])

    names.append("LElbowYaw")  # X轴
    times.append([1, 2, 3, 4])
    keys.append([-1.37757, -1.37757, -1.466076, -1.466076])

    names.append("LHand")  # 左掌
    times.append([1, 2, 3, 4, 5.2])
    keys.append([0.9800, 0.9800, 0.9800, 0.9800, 0.1800])

    names.append("LHipPitch")  # 腿Y轴
    times.append([1, 2, 3, 4])
    keys.append([-0.450955, -0.450955, -0.450955, -0.450955])

    names.append("LHipRoll")  # 腿X轴
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LHipYawPitch")  # 啥关节
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LKneePitch")  # 膝盖Y轴
    times.append([1, 2, 3, 4])
    keys.append([0.699462, 0.699462, 0.699462, 0.699462])

    names.append("LShoulderPitch")  # 左肩轴       
    times.append([1, 2, 3, 4, 5.2])
    ##------------------------------------------------------------
    keys.append([1.53885, 1.43885, 1.3, 1.3, 1.3])

    names.append("LShoulderRoll")  # 肩Z轴
    times.append([1, 2, 3, 4, 5.2])
    keys.append([0.268407, 0.268407, -0.04014, -0.04014, -0.04014])

    names.append("LWristYaw")  # 手腕X轴
    times.append([1, 2, 3, 4])
    keys.append([-0.016916, -0.016916, -1.632374, -1.632374])

    names.append("RAnklePitch")  # 脚踝Y轴
    times.append([1, 2, 3, 4])
    keys.append([-0.354312, -0.354312, -0.354312, -0.354312])

    names.append("RAnkleRoll")  # 脚踝X轴
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RElbowRoll")  # 肘Z轴
    times.append([1, 2, 3, 4])
    keys.append([0.958791, 0.958791, 0.958791, 0.958791])

    names.append("RElbowYaw")  # 肘X轴
    times.append([1, 2, 3, 4])
    keys.append([1.466076, 1.466076, 1.466076, 1.466076])

    names.append("RHand")
    times.append([1, 2, 3, 4])
    keys.append([0.0900, 0.0900, 0.0900, 0.0900])

    names.append("RHipPitch")  # 腿Y轴
    times.append([1, 2, 3, 4])
    keys.append([-0.451038, -0.451038, -0.451038, -0.451038])

    names.append("RHipRoll")  # 腿X轴
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RHipYawPitch")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RKneePitch")  # 膝盖Y轴
    times.append([1, 2, 3, 4])
    keys.append([0.699545, 0.699545, 0.699545, 0.699545])

    names.append("RShoulderPitch")  # 肩Y轴
    times.append([0.5,1, 2, 3, 4, 5.2])
    # keys.append([1.03856, 1.03856, 1.03856, 1.03856, 1.03856])
    keys.append([0.9, 1.03856, 1.03856,1.03856, 1.03856, 1.03856])

    names.append("RShoulderRoll")  # 肩Z轴
    times.append([1, 2, 3, 4, 5.2])
    keys.append([0.04014, 0.04014, 0.04014, 0.04014, 0.04014])

    names.append("RWristYaw")  # 腕X轴
    times.append([1, 2, 3, 4])
    keys.append([1.632374, 1.632374, 1.632374, 1.632374])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)  # 如果为true，则以绝对角度描述运动，否则角度相对于当前角度why？

def LShoulderpitchAmend():
    names = list()
    keys = list()
    times = list()
    names.append("LShoulderPitch")  #
    times.append([1])
    keys.append([1.03856])
    names.append("LHand")
    times.append([1, 2, 3, 4])
    keys.append([0.0200, 0.0200, 0.0200, 0.0200])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)

def ShoulderpitchAmend():
    names = list()
    keys = list()
    Reduction()
    Reduction1()
    Reduction2()
    time.sleep(0.5)
    # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.4)

    times = list()
    names.append("LShoulderPitch")  #
    times.append([0.5, 1])
    keys.append([1.13856, 1.43856])
    names.append("RShoulderPitch")  #
    times.append([0.5, 1])
    keys.append([1.13856, 1.43856])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)

def ShoulderpitchAmend2():
    names = list()
    keys = list()
    times = list()
    names.append("LShoulderPitch")  #
    times.append([0.5, 1])
    keys.append([1.43856, 1.88495559])
    names.append("RShoulderPitch")  #
    times.append([0.5, 1])
    keys.append([1.43856, 1.88495559])
    names.append("LElbowRoll")  #
    times.append([0.5, 1])
    keys.append([-1.23490659, -1.51843645])
    names.append("RElbowRoll")  #
    times.append([0.5, 1])
    keys.append([1.23490659, 1.51843645])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)

def Reduction1():
    names = list()
    keys = list()
    times = list()
    names.append('LHand')
    keys.append([0.2, 0.4, 0.6])
    times.append([0.25, 0.5, 0.75])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)

def Reduction():
    names = list()
    keys = list()
    times = list()
    names.append("LShoulderPitch")  #
    times.append([0.25, 0.5, 1.5, 2, 2.5])
    keys.append([1.88495559, 1.42244334, 1.33856, 1.23856, 1.13856])
    names.append("RShoulderPitch")  #
    times.append([0.25, 0.5, 1.5, 2, 2.5])
    keys.append([1.88495559, 1.42244334, 1.33856, 1.23856, 1.13856])
    names.append("LElbowRoll")  #
    times.append([0.25, 0.5, 1.5, 2])
    keys.append([-1.51843645, -1.23490659, -1, -0.958791])
    names.append("RElbowRoll")  #
    times.append([0.25, 0.5, 1.5, 2])
    keys.append([1.51843645, 1.23490659, 1, 0.958791])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)

def Reduction2():
    names = list()
    keys = list()
    times = list()
    names.append("LShoulderPitch")  #
    times.append([0.25, 0.5])
    keys.append([1.42244334, 1.8])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)

def Reduction3():
    names = list()
    keys = list()
    times = list()
    names.append("RHand")
    times.append([0.5, 1])
    keys.append([0.0900, 0.0900])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)

# 击球 ------------------------------------------------#
def kaiBall():
    motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR7, 0.2)
    time.sleep(1)
    # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.2)
    time.sleep(0.5)
    motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR31, 0.1)
    time.sleep(1)
    # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR11,0.15)

    # 添加程序
    effectornamelist = ["RWristYaw"]
    timelist = [0.7]
    targetlist = [-65 * math.pi / 180.0]
    motionPrx.angleInterpolation(effectornamelist, targetlist, timelist, True)

def HitBall():
    if (alpha < 0):
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR7, 0.2)
        time.sleep(1)
        # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.2)
        time.sleep(0.5)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR31, 0.1)
        time.sleep(1)
        # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR11,0.15)
        effectornamelist = ["RWristYaw"]
        timelist = [1.2]
        targetlist = [-65 * math.pi / 180.0]
        motionPrx.angleInterpolation(effectornamelist, targetlist, timelist, True)

    if (alpha > 0):
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR7, 0.2)
        time.sleep(1)
        # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.2)
        time.sleep(0.5)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR32, 0.1)  # 手腕逆弯曲
        time.sleep(2)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR12, 0.25)

def HitBall1():
    if (alpha < 0):
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR7, 0.2)
        time.sleep(1)
        # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.2)
        time.sleep(0.5)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR31, 0.1)
        time.sleep(1)

        # 添加程序
        effectornamelist = ["RWristYaw"]
        timelist = [0.5]
        targetlist = [-45 * math.pi / 180.0]
        motionPrx.angleInterpolation(effectornamelist, targetlist, timelist, True)

        # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR111,0.01)
    if (alpha > 0):
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR7, 0.2)
        time.sleep(1)
        # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.2)
        time.sleep(0.5)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR32, 0.1)  # 手腕逆弯曲
        time.sleep(2)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR12, 0.25)

def HitBall22():
    motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR7, 0.2)
    time.sleep(1)
    # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.2)
    time.sleep(0.5)
    motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR101, 0.1)
    time.sleep(1)
    # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR11, 0.2)
    names = list()
    times = list()
    keys = list()
    names.append("RWristYaw")
    times.append([0.65])
    keys.append([-0.738397])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)


def HitBallCouldAjust(step = 1):
    if (alpha < 0):
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR7, 0.2)
        time.sleep(1)
        # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.2)
        time.sleep(0.5)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR101, 0.1)
        time.sleep(1)
        # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR11, 0.2)
        motionPrx.setMoveArmsEnabled(False, False)
        roundTime = 0.6
        if step == 4:
            roundTime = 0.6
        if step == 3:
            roundTime = 0.8
        if step == 2:
            roundTime = 1.0
        if step == 1:
            roundTime = 1.2
        motionPrx.angleInterpolation(["RWristYaw"], [-0.738397], [roundTime], True)
    if (alpha > 0):
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR7, 0.2)
        time.sleep(1)
        # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.2)
        time.sleep(0.5)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR32, 0.1)
        time.sleep(2)
        # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR12, 0.2)
        motionPrx.setMoveArmsEnabled(False, False)
        roundTime = 0.6
        if step == 4:
            roundTime = 0.6
        if step == 3:
            roundTime = 0.8
        if step == 2:
            roundTime = 1.0
        if step == 1:
            roundTime = 1.2
        motionPrx.angleInterpolation(["RWristYaw"], [0.738397], [roundTime], True)

# 黄杆有关 ------------------------------------------------#
def findYellowStick(contourRange = [75, 850]):
    TIME = time.strftime('%m-%d_%H-%M-%S', time.localtime(time.time()))
    print("Finding Yellow Stick")
    # 15-20 contains orange
    low = np.array([15, 80, 80])
    up = np.array([36, 255, 255])
    camProxy.setActiveCamera(0)
    videoClient = camProxy.subscribe("python_client", 2, 11, 5)  # 640*480，RGB, FPS
    rowImgData = camProxy.getImageRemote(videoClient)
    camProxy.unsubscribe(videoClient)
    imgWidth = rowImgData[0]
    imgHeight = rowImgData[1]
    image = np.zeros((imgHeight, imgWidth, 3), dtype='uint8')
    image.data = rowImgData[6]
    b, g, r = cv2.split(image)
    img = cv2.merge([r, g, b])
    # deal img
    frameHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    frameBin = cv2.inRange(frameHSV, low, up)
    kernelErosion = np.ones((5, 5), np.uint8)
    kernelDilation = np.ones((5, 5), np.uint8)
    frameBin = cv2.erode(frameBin, kernelErosion, iterations=1)
    frameBin = cv2.dilate(frameBin, kernelDilation, iterations=1)
    frameBin = cv2.GaussianBlur(frameBin, (3, 3), 0)
    # frameBin debug
    cv2.imwrite("bin.jpg", frameBin)
    _, contours, _ = cv2.findContours(frameBin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) == 0:
        cv2.imwrite("not_find_stick%s.jpg" % TIME, img)
        print("no Yellow in img")
        return []
    # abandon some yello shape because of width
    good_contour = []
    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= contourRange[0] or perimeter >= contourRange[1]:
            print "findYellowStick(): to large or to small"
            continue
        ratio = 4
        if perimeter >= 150:
            ratio = 5
        box2D = cv2.minAreaRect(contour)
        w = min(box2D[1][0], box2D[1][1])
        h = max(box2D[1][0], box2D[1][1])
        print "findYellowStick(): a yellow", w, h
        if w * ratio > h:
            continue
        good_contour.append(contour)
    if len(good_contour) == 0:
        cv2.imwrite("not_find_stick%s.jpg" % TIME, img)
        print("Yellow range width is to large")
        return []
    # find big contour
    maxContour = good_contour[0]
    maxPerimeter = contourRange[0]
    for contour in good_contour:
        perimeter = cv2.arcLength(contour, True)
        if perimeter > maxPerimeter:
            x, y, w, h = cv2.boundingRect(contour)
            print "w: ", w, h
            print perimeter
            if 2.5 * w > h:
                print "findYellowStick(): find a hotizontal yellow stick"
                continue
            maxPerimeter = perimeter
            maxContour = contour
    # find alpha
    box2D = cv2.minAreaRect(maxContour)
    if box2D[1][0] > box2D[1][1]:
        w = box2D[1][1]
        h = box2D[1][0]
        center = box2D[0]
        theta = -box2D[2] * math.pi / 180
        alpha = theta + math.atan(1.0 * w / h)
        bevel = math.sqrt(w * w + h * h) / 2

        x1 = int(center[0] - bevel * math.cos(alpha))
        x2 = int(x1 - w * math.sin(theta))
        x = int((x1 + x2) / 2)
    else:
        w = box2D[1][0]
        h = box2D[1][1]
        center = box2D[0]
        theta = -box2D[2] * math.pi / 180
        alpha = theta + math.atan(1.0 * h / w)
        bevel = math.sqrt(w * w + h * h) / 2
        x1 = int(center[0] - bevel * math.cos(alpha))
        x2 = int(x1 + w * math.cos(theta))
        x = int((x1 + x2) / 2)
    cv2.drawContours(img, maxContour, -1, (0, 0, 255), 1)
    cv2.imwrite("ok.jpg", img)
    # 这个值是水平方向角
    return (320.0 - x) / 640.0 * 60.97 * math.pi / 180

def findYellowStick31():
    """
    :param minPerimeter: yellow area min perimeter
    """
    TIME = time.strftime('%m-%d_%H-%M-%S', time.localtime(time.time()))
    print("Finding Yellow Stick")
    # 15-20 contains orange
    low = np.array([15, 80, 80])
    up = np.array([36, 255, 255])
    camProxy.setActiveCamera(0)
    videoClient = camProxy.subscribe("python_client", 2, 11, 5) # 640*480，RGB, FPS
    rowImgData = camProxy.getImageRemote(videoClient)
    camProxy.unsubscribe(videoClient)
    imgWidth = rowImgData[0]
    imgHeight = rowImgData[1]
    image = np.zeros((imgHeight, imgWidth, 3), dtype='uint8')
    image.data = rowImgData[6]
    b, g, r = cv2.split(image)
    img = cv2.merge([r, g, b])
    # deal img
    frameHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    frameBin = cv2.inRange(frameHSV, low, up)
    kernelErosion = np.ones((5, 5), np.uint8)
    kernelDilation = np.ones((5, 5), np.uint8)
    frameBin = cv2.erode(frameBin, kernelErosion, iterations=1)
    frameBin = cv2.dilate(frameBin, kernelDilation, iterations=1)
    frameBin = cv2.GaussianBlur(frameBin, (3, 3), 0)
    # frameBin debug
    cv2.imwrite("bin.jpg", frameBin)
    _, contours, _ = cv2.findContours(frameBin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) == 0:
        cv2.imwrite("not_find_stick%s.jpg" %TIME, img)
        print("no Yellow in img")
        return []
    # abandon some yello shape because of width
    good_contour = []
    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 100 or perimeter >= 300:
            continue
        ratio = 4
        if perimeter >= 150:
            ratio = 5
        box2D = cv2.minAreaRect(contour)
        w = min(box2D[1][0], box2D[1][1])
        h = max(box2D[1][0], box2D[1][1])
        if w * ratio > h:
            continue
        good_contour.append(contour)
    if len(good_contour) == 0:
        cv2.imwrite("not_find_stick%s.jpg" %TIME, img)
        print("Yellow range width is to large")
        return []
    # find big contour
    maxContour = good_contour[0]
    maxPerimeter = 100
    for contour in good_contour:
        perimeter = cv2.arcLength(contour, True)
        if perimeter > maxPerimeter:
            maxPerimeter = perimeter
            maxContour = contour
    # find alpha
    box2D = cv2.minAreaRect(maxContour)
    if box2D[1][0] > box2D[1][1]:
        w = box2D[1][1]
        h = box2D[1][0]
        center = box2D[0]
        theta = -box2D[2] * math.pi / 180
        alpha = theta + math.atan(1.0 * w / h)
        bevel = math.sqrt(w * w + h * h) / 2
       
        x1 = int(center[0] - bevel * math.cos(alpha))
        x2 = int(x1 - w * math.sin(theta))
        x = int((x1 + x2) / 2)
    else:
        w = box2D[1][0]
        h = box2D[1][1]
        center = box2D[0]
        theta = -box2D[2] * math.pi / 180
        alpha = theta + math.atan(1.0 * h / w)
        bevel = math.sqrt(w * w + h * h) / 2
        x1 = int( center[0] - bevel * math.cos(alpha) )
        x2 = int( x1 + w * math.cos(theta) )
        x = int((x1 + x2) / 2)
    cv2.drawContours(img, maxContour, -1, (0, 0, 255), 1)
    cv2.imwrite("ok.jpg", img)
    # 这个值是水平方向角
    return (320.0 - x)/640.0 * 60.97 * math.pi / 180

def findYellowStick32():
    """
    :param minPerimeter: yellow area min perimeter
    """
    TIME = time.strftime('%m-%d_%H-%M-%S', time.localtime(time.time()))
    print("Finding Yellow Stick")
    # 15-20 contains orange
    low = np.array([15, 80, 80])
    up = np.array([36, 255, 255])
    camProxy.setActiveCamera(0)
    videoClient = camProxy.subscribe("python_client", 2, 11, 5) # 640*480，RGB, FPS
    rowImgData = camProxy.getImageRemote(videoClient)
    camProxy.unsubscribe(videoClient)
    imgWidth = rowImgData[0]
    imgHeight = rowImgData[1]
    image = np.zeros((imgHeight, imgWidth, 3), dtype='uint8')
    image.data = rowImgData[6]
    b, g, r = cv2.split(image)
    img = cv2.merge([r, g, b])
    # deal img
    cv2.imwrite("scr.jpg", img)
    frameHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    frameBin = cv2.inRange(frameHSV, low, up)
    kernelErosion = np.ones((5, 5), np.uint8)
    kernelDilation = np.ones((5, 5), np.uint8)
    frameBin = cv2.erode(frameBin, kernelErosion, iterations=1)
    frameBin = cv2.dilate(frameBin, kernelDilation, iterations=1)
    frameBin = cv2.GaussianBlur(frameBin, (3, 3), 0)
    # frameBin debug
    cv2.imwrite("bin" + TIME + ".jpg", frameBin)
    _, contours, _ = cv2.findContours(frameBin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) == 0:
        cv2.imwrite("not_find_stick%s.jpg" %TIME, img)
        print("no Yellow in img")
        return []
    # abandon some yello shape because of width
    good_contour = []
    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        print "findYellowStick32(): perimeter-", perimeter
        if perimeter <= 200 or perimeter >= 400:
            continue
        ratio = 3
        if perimeter >= 250:
            ratio = 4
        box2D = cv2.minAreaRect(contour)
        w = min(box2D[1][0], box2D[1][1])
        h = max(box2D[1][0], box2D[1][1])
        if w * ratio > h:
            continue
        good_contour.append(contour)
    if len(good_contour) == 0:
        cv2.imwrite("not_find_stick%s.jpg" %TIME, img)
        print("Yellow range width is to large")
        return []
    # find big contour
    maxContour = good_contour[0]
    maxPerimeter = 100
    for contour in good_contour:
        perimeter = cv2.arcLength(contour, True)
        if perimeter > maxPerimeter:
            maxPerimeter = perimeter
            maxContour = contour
    # find alpha
    box2D = cv2.minAreaRect(maxContour)
    if box2D[1][0] > box2D[1][1]:
        w = box2D[1][1]
        h = box2D[1][0]
        center = box2D[0]
        theta = -box2D[2] * math.pi / 180
        alpha = theta + math.atan(1.0 * w / h)
        bevel = math.sqrt(w * w + h * h) / 2
        x1 = int(center[0] - bevel * math.cos(alpha))
        x2 = int(x1 - w * math.sin(theta))
        x = int((x1 + x2) / 2)
    else:
        w = box2D[1][0]
        h = box2D[1][1]
        center = box2D[0]
        theta = -box2D[2] * math.pi / 180
        alpha = theta + math.atan(1.0 * h / w)
        bevel = math.sqrt(w * w + h * h) / 2
        x1 = int( center[0] - bevel * math.cos(alpha) )
        x2 = int( x1 + w * math.cos(theta) )
        x = int((x1 + x2) / 2)
    cv2.drawContours(img, maxContour, -1, (0, 0, 255), 1)
    cv2.imwrite("ok.jpg", img)
    return (320.0 - x)/640.0 * 60.97 * math.pi / 180

def findYellowStick_33():
    """
    :param minPerimeter: yellow area min perimeter
    """
    TIME = time.strftime('%m-%d_%H-%M-%S', time.localtime(time.time()))
    print("Finding Yellow Stick")
    # 15-20 contains orange
    low = np.array([15, 80, 80])
    up = np.array([36, 255, 255])
    camProxy.setActiveCamera(0)
    videoClient = camProxy.subscribe("python_client", 2, 11, 5) # 640*480，RGB, FPS
    rowImgData = camProxy.getImageRemote(videoClient)
    camProxy.unsubscribe(videoClient)
    imgWidth = rowImgData[0]
    imgHeight = rowImgData[1]
    image = np.zeros((imgHeight, imgWidth, 3), dtype='uint8')
    image.data = rowImgData[6]
    b, g, r = cv2.split(image)
    img = cv2.merge([r, g, b])
    # deal img
    cv2.imwrite("scr.jpg", img)
    frameHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    frameBin = cv2.inRange(frameHSV, low, up)
    kernelErosion = np.ones((5, 5), np.uint8)
    kernelDilation = np.ones((5, 5), np.uint8)
    frameBin = cv2.erode(frameBin, kernelErosion, iterations=1)
    frameBin = cv2.dilate(frameBin, kernelDilation, iterations=1)
    frameBin = cv2.GaussianBlur(frameBin, (3, 3), 0)
    # frameBin debug
    cv2.imwrite("bin.jpg", frameBin)
    _, contours, _ = cv2.findContours(frameBin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) == 0:
        cv2.imwrite("not_find_stick%s.jpg" %TIME, img)
        print("no Yellow in img")
        return []
    # abandon some yello shape because of width
    good_contour = []
    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        print perimeter
        if perimeter <= 400 or perimeter >= 1300:
            continue
        ratio = 4
        if perimeter >= 250:
            ratio = 5
        box2D = cv2.minAreaRect(contour)
        w = min(box2D[1][0], box2D[1][1])
        h = max(box2D[1][0], box2D[1][1])
        if w * ratio > h:
            continue
        good_contour.append(contour)
    if len(good_contour) == 0:
        cv2.imwrite("not_find_stick%s.jpg" %TIME, img)
        print("Yellow range width is to large")
        return []
    # find big contour
    maxContour = good_contour[0]
    maxPerimeter = 100
    for contour in good_contour:
        perimeter = cv2.arcLength(contour, True)
        if perimeter > maxPerimeter:
            maxPerimeter = perimeter
            maxContour = contour
    # find alpha
    box2D = cv2.minAreaRect(maxContour)
    if box2D[1][0] > box2D[1][1]:
        w = box2D[1][1]
        h = box2D[1][0]
        center = box2D[0]
        theta = -box2D[2] * math.pi / 180
        alpha = theta + math.atan(1.0 * w / h)
        bevel = math.sqrt(w * w + h * h) / 2
        x1 = int(center[0] - bevel * math.cos(alpha))
        x2 = int(x1 - w * math.sin(theta))
        x = int((x1 + x2) / 2)
    else:
        w = box2D[1][0]
        h = box2D[1][1]
        center = box2D[0]
        theta = -box2D[2] * math.pi / 180
        alpha = theta + math.atan(1.0 * h / w)
        bevel = math.sqrt(w * w + h * h) / 2
        x1 = int( center[0] - bevel * math.cos(alpha) )
        x2 = int( x1 + w * math.cos(theta) )
        x = int((x1 + x2) / 2)
    cv2.drawContours(img, maxContour, -1, (0, 0, 255), 1)
    cv2.imwrite("ok.jpg", img)
    return (320.0 - x)/640.0 * 60.97 * math.pi / 180

def findYellowStick_with_mark():
    """
    :param minPerimeter: yellow area min perimeter
    """
    TIME = time.strftime('%m-%d_%H-%M-%S', time.localtime(time.time()))
    print("Finding Yellow Stick")
    # 15-20 contains orange
    low = np.array([15, 80, 80])
    up = np.array([36, 255, 255])
    camProxy.setActiveCamera(0)
    videoClient = camProxy.subscribe("python_client", 2, 11, 5) # 640*480，RGB, FPS
    rowImgData = camProxy.getImageRemote(videoClient)
    camProxy.unsubscribe(videoClient)
    imgWidth = rowImgData[0]
    imgHeight = rowImgData[1]
    image = np.zeros((imgHeight, imgWidth, 3), dtype='uint8')
    image.data = rowImgData[6]
    b, g, r = cv2.split(image)
    img = cv2.merge([r, g, b])
    # deal img
    cv2.imwrite("scr.jpg", img)
    frameHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    frameBin = cv2.inRange(frameHSV, low, up)
    kernelErosion = np.ones((5, 5), np.uint8)
    kernelDilation = np.ones((5, 5), np.uint8)
    frameBin = cv2.erode(frameBin, kernelErosion, iterations=1)
    frameBin = cv2.dilate(frameBin, kernelDilation, iterations=1)
    frameBin = cv2.GaussianBlur(frameBin, (3, 3), 0)
    # frameBin debug
    cv2.imwrite("bin.jpg", frameBin)
    _, contours, _ = cv2.findContours(frameBin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) == 0:
        cv2.imwrite("not_find_stick%s.jpg" %TIME, img)
        print("no Yellow in img")
        return []
    # abandon some yello shape because of width
    good_contour = []
    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        print perimeter
        if perimeter <= 400 or perimeter >= 1200:
            continue
        ratio = 3
        if perimeter >= 250:
            ratio = 4
        box2D = cv2.minAreaRect(contour)
        w = min(box2D[1][0], box2D[1][1])
        h = max(box2D[1][0], box2D[1][1])
        if w * ratio > h:
            continue
        good_contour.append(contour)
    if len(good_contour) == 0:
        cv2.imwrite("not_find_stick%s.jpg" %TIME, img)
        print("Yellow range width is to large")
        return []
    # find big contour
    maxContour = good_contour[0]
    maxPerimeter = 100
    for contour in good_contour:
        perimeter = cv2.arcLength(contour, True)
        if perimeter > maxPerimeter:
            maxPerimeter = perimeter
            maxContour = contour
    # find alpha
    box2D = cv2.minAreaRect(maxContour)
    if box2D[1][0] > box2D[1][1]:
        w = box2D[1][1]
        h = box2D[1][0]
        center = box2D[0]
        theta = -box2D[2] * math.pi / 180
        alpha = theta + math.atan(1.0 * w / h)
        bevel = math.sqrt(w * w + h * h) / 2
        x1 = int(center[0] - bevel * math.cos(alpha))
        x2 = int(x1 - w * math.sin(theta))
        x = int((x1 + x2) / 2)
    else:
        w = box2D[1][0]
        h = box2D[1][1]
        center = box2D[0]
        theta = -box2D[2] * math.pi / 180
        alpha = theta + math.atan(1.0 * h / w)
        bevel = math.sqrt(w * w + h * h) / 2
        x1 = int( center[0] - bevel * math.cos(alpha) )
        x2 = int( x1 + w * math.cos(theta) )
        x = int((x1 + x2) / 2)
    cv2.drawContours(img, maxContour, -1, (0, 0, 255), 1)
    cv2.imwrite("ok.jpg", img)
    return (320.0 - x)/640.0 * 60.97 * math.pi / 180

def verticalToYellowStick():
    headAngle = - 2.0 / 3 * math.pi
    maxAngle = 0
    motionPrx.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
    motionPrx.angleInterpolationWithSpeed("HeadYaw", headAngle, 0.5)
    time.sleep(0.5)
    alpha = findYellowStick([75, 200])
    while alpha == []:
        headAngle += 1.0 / 3 * math.pi
        if headAngle > maxAngle:
            return -100
        motionPrx.angleInterpolationWithSpeed("HeadYaw", headAngle, 0.5)
        motionPrx.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
        time.sleep(0.5)
        alpha = findYellowStick([75, 200])
    final_headangle = motionPrx.getAngles("HeadYaw", True)
    # 算出来以后再向左转90度
    d_angle = final_headangle[0] + alpha + 0.5 * math.pi
    refixRound(d_angle)
    return d_angle

def towordYellowStick32():
    headAngle = - 1.0 / 3 * math.pi
    maxAngle =  1.0 / 3 * math.pi
    motionPrx.angleInterpolationWithSpeed("HeadYaw", headAngle, 0.5)
    motionPrx.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
    time.sleep(0.5)
    alpha = findYellowStick([150, 360])
    while alpha == []:
        headAngle += 1.0 / 3 * math.pi
        if headAngle > maxAngle:
            return -100
        motionPrx.angleInterpolationWithSpeed("HeadYaw", headAngle, 0.5)
        motionPrx.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
        time.sleep(0.5)
        alpha = findYellowStick([150, 360])
    final_headangle = motionPrx.getAngles("HeadYaw", True)
    d_angle = final_headangle[0] + alpha
    refixRound(d_angle)
    return d_angle

def verticalToYellowStick_withoutMark():
    headAngle = - 2.0 / 3 * math.pi
    maxAngle = 0
    motionPrx.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
    motionPrx.angleInterpolationWithSpeed("HeadYaw", headAngle, 0.5)
    time.sleep(0.5)
    alpha = findYellowStick([300, 850])
    while alpha == []:
        headAngle += 1.0 / 3 * math.pi
        if headAngle > maxAngle:
            return -100
        motionPrx.angleInterpolationWithSpeed("HeadYaw", headAngle, 0.5)
        motionPrx.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
        time.sleep(0.5)
        alpha = findYellowStick([300, 850])
    final_headangle = motionPrx.getAngles("HeadYaw", True)
    # 算出来以后再向左转90度
    d_angle = final_headangle[0] + alpha + 0.5 * math.pi
    refixRound(d_angle)
    return d_angle

# 红球 ------------------------------------------------#

def findball(angle, sub_angle=0):
    camProxy.setActiveCamera(1)
    motionPrx.angleInterpolationWithSpeed("HeadYaw", angle * math.pi / 180, 0.5)
    motionPrx.angleInterpolationWithSpeed("HeadPitch", sub_angle * math.pi / 180, 0.3)
    redballProxy.subscribe("redBallDetected")
    memoryProxy.insertData("redBallDetected", [])
    # 增加红球识别和取消红球识别
    time.sleep(2)
    ballData = memoryProxy.getData("redBallDetected")
    #    redballProxy.unsubscribe("redBallDetected")
    if (ballData != []):
        tts.say("红球大约在" + str(angle) + "度")
        headangle = motionPrx.getAngles("HeadYaw", True)
        allballData = [headangle, ballData, 0]
        return allballData
    else:
        return []

def firstSearchredball():
    global _searchBallTimes, _stepStatue
    _searchBallTimes += 1
    camProxy.setActiveCamera(1)

    if _searchBallTimes <= 3:
        searchRange = -60
    else:
        searchRange = -120

    search = searchRange
    while search <= -searchRange:
        findballinfo = findball(search)
        if findballinfo != []:
            return findballinfo
        else:
            search += 60

    tts.say("暂时没有找到红球")  # 没有球
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.moveTo(0.3, 0.0, 0.0, _stepStatue)  # 寻找距离修正
    allballDatatest = [0, 0, 1]
    return allballDatatest

def CalculateRobotToRedball(allballData):
    global  _stepStatue, _stepStatue2
    h = 0.478
    x = None
    theta1 = allballData[0][0] + allballData[1][1][0]
    refixRound(theta1)
    tts.post.say("正对红球")
    # 走到三十厘米左右
    val1 = findball(0)
    if val1 != []:
        # 顺便修正一下角度
        if abs(val1[1][1][0]) >= 0.08:
            motionPrx.moveTo(0, 0, val1[1][1][0], _stepStatue)
        thetav = val1[1][1][1] + (39.7 * math.pi / 180.0)
        time.sleep(0.5)
        x = h / (math.tan(thetav)) - 0.35
    else:
        # 没有找到球,那么就用第一次找到球的计算数据
        thetav = allballData[1][1][1] + (39.7 * math.pi / 180.0)
        x = h / (math.tan(thetav)) * math.cos(theta1) - 0.35
    if x >= 0:
        motionPrx.moveTo(x, 0, 0, _stepStatue)

    # 我开始了
    tts.post.say("我开始了")
    val3 = findball(0, 30)
    if val3 == []:
        motionPrx.moveTo(-0.05, 0, 0, _stepStatue2)
        val3 = findball(0, 30)
    if val3 == []:
        motionPrx.moveTo(0.15, 0, 0, _stepStatue2)
        val3 = findball(0, 30)
    # 即使通过上面的方法, 可能还是找不到红球, 我们放在后面
    if val3 != []:
        theta = val3[1][1][0]
        emsl = val3[1][3][0]
        thetav = val3[1][1][1] + (69.7 * math.pi / 180.0)
        x = (h - 0.03) / (math.tan(thetav)) - 0.10  # 三点一线最终修改关键点
        motionPrx.moveTo(x, 0, theta, _stepStatue2)
        tts.say("我应该到10厘米了")

        # v4用来调节角度
        val4 = findball(0, 30)
        if val4 != []:
            if abs(val4[1][0][1]) >= 0.08:
                motionPrx.moveTo(0, 0, val4[1][1][1], _stepStatue2)
            elif val4[1][0][1] > 0:
                    motionPrx.moveTo(0, 0.02, 0)
            elif val4[1][0][1] == 0:
                pass
            else:
                motionPrx.moveTo(0, 0.02, 0)
        else:
            motionPrx.moveTo(-0.04, 0, 0)
        val5 = findball(0, 30)
        fai = 0
        if val5 !=[]:
            thetav = val5[1][1][1] + (69.7 * math.pi / 180.0)
            dx = (h - 0.03) / (math.tan(thetav))
            dy = ((val5[1][3][2] - 0.03) / math.sin(thetav)) * math.tan(val5[1][1][1])
            motionPrx.moveTo(0, 0, dy, _stepStatue2)
            print "very, very important:    ", dy
            fai = math.atan(dy / dx)
        elif val4 != []:
            thetav = val4[1][1][1] + (69.7 * math.pi / 180.0)
            dx = (h - 0.03) / (math.tan(thetav))
        else:
            dx = 0.14
        dx += emsl
        print "final: ", dx, fai
        return dx, fai
    else:
        tts.post.say("凉凉, 重新开始吧")
        motionPrx.moveTo(-0.3, 0, 0)
        while True:
            allballData = firstSearchredball()
            if (allballData[2] == 0):
                break
        return CalculateRobotToRedball(allballData)

def CalculateRobotToRedball1(allballData):
    h = 0.478
    # 机器人行走参数
    maxstepx = 0.04
    maxstepy = 0.14
    maxsteptheta = 0.3
    maxstepfrequency = 0.6
    stepheight = 0.02
    torsowx = 0.0
    torsowy = 0.0

    headangle = allballData[0]  # 头偏转角
    wzCamera = allballData[1][1][0]  # alpha角
    wyCamera = allballData[1][1][1]  # beta角
    isenabled = False
    x = 0.0
    y = 0.0
    if (headangle[0] + wzCamera < 0.0):
        theta = headangle[0] + wzCamera + 0.2  # 修改一
    else:
        theta = headangle[0] + wzCamera

    motionPrx.setMoveArmsEnabled(False, False)
    # 接下来，第一次，机器人转到正对红球的方向
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    motionPrx.moveTo(x, y, theta,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])  # x=y=0

    time.sleep(1.5)
    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (39.7 * math.pi / 180.0)
    x = h / (math.tan(thetav)) - 0.5  # 最少为40厘米
    if (x >= 0):
        theta = 0.0
        motionPrx.setMoveArmsEnabled(False, False)
        print("正对红球方向完成")
        # 接下来，第二次，机器人走到距离红球20厘米的位置,程序为40厘米，因为机器人在实际行走过程中误差过大，会踢到球
        motionPrx.moveTo(x, y, theta,
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])

        motionPrx.waitUntilMoveIsFinished()
    # 向下低头30度
    effectornamelist = ["HeadPitch"]
    timelist = [0.5]
    targetlist = [30 * math.pi / 180.0]
    motionPrx.angleInterpolation(effectornamelist, targetlist, timelist, isenabled)
    time.sleep(1.5)
    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
    x = 0.0
    y = 0.0
    theta = thetah
    motionPrx.setMoveArmsEnabled(False, False)
    print("走到距离红球20cm处")
    # 接下来，第三次，机器人修正角度对准红球
    motionPrx.moveTo(x, y, theta,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    time.sleep(1.5)

    maxstepx = 0.02
    maxstepy = 0.14
    maxsteptheta = 0.15
    maxstepfrequency = 0.6
    stepheight = 0.02
    torsowx = 0.0
    torsowy = 0.0

    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
    x = (h - 0.03) / (math.tan(thetav)) - 0.15  # 三点一线最终修改关键点
    theta = thetah
    motionPrx.setMoveArmsEnabled(False, False)
    print("角度修正完成")
    # 接下来，第四次，机器人走到距离红球10厘米的位置
    motionPrx.moveTo(x, y, theta,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    time.sleep(1.5)
    print("到达10cm处")
    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    print(thetah)
    thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
    x = 0.0
    y = 0.0
    theta = thetah
    motionPrx.setMoveArmsEnabled(False, False)
    print("走到距离红球20cm处")
    # 接下来，第五次，机器人最后修正角度对准红球
    motionPrx.moveTo(x, y, theta,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    time.sleep(1.5)
    # 接下来，第六次，最后一次检测球和机器人的距离dx
    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
    dx = (h - 0.03) / (math.tan(thetav))  # dx作为了三角形的一条边
    print dx
    return dx
    # print("dx="+dx)

# NAOmark ---------------------------------#

def findMark():
    markData = memoryProxy.getData("LandmarkDetected")
    if markData == []:
        time.sleep(0.5)
        markData = memoryProxy.getData("LandmarkDetected")
    if markData == []:
        time.sleep(0.5)
        markData = memoryProxy.getData("LandmarkDetected")
    return markData


def robotTolandmark(allmarkdata):
    currentCamera = "CameraTop"
    landmarkTheoreticalSize = 0.1  # in meters
    wzCamera = allmarkdata[0]
    wyCamera = allmarkdata[1]
    angularSize = allmarkdata[2]
    angle = allmarkdata[3]  # 机器人与红球xdistance ，机器人与mark之间sdistance ， 两条线的夹角为angle
    # 计算距离
    distanceFromCameraToLandmark = landmarkTheoreticalSize / (2 * math.tan(angularSize / 2))

    # 获取当前机器人到摄像头的距离的变换矩阵
    transform = motionPrx.getTransform(currentCamera, 2, True)
    transformList = almath.vectorFloat(transform)
    robotToCamera = almath.Transform(transformList)

    # 计算指向NAOmark的旋转矩阵
    cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, wyCamera, wzCamera)

    # 摄像头到NAOmark的矩阵
    cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)

    # 机器人到NAOmark的矩阵
    robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform
    x = robotToLandmark.r1_c4
    y = robotToLandmark.r2_c4
    z = robotToLandmark.r3_c4
    sdistance = math.sqrt(x * x + y * y)  # 机器人与mark的距离sdistance
    robotTolandmarkdata = [angle, sdistance, x, y, z]
    print "x " + str(robotToLandmark.r1_c4) + " (in meters)"
    print "y " + str(robotToLandmark.r2_c4) + " (in meters)"
    print "z " + str(robotToLandmark.r3_c4) + " (in meters)"
    return robotTolandmarkdata

def firstSearchNAOmark():
    headYawAngle = -2.09
    camProxy.setActiveCamera(0)
    motionPrx.angleInterpolationWithSpeed("HeadPitch", -0.1, 0.3)
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3)
    landmarkProxy.subscribe("landmarkTest")

    while (headYawAngle < 2.09):
        motionPrx.angleInterpolationWithSpeed("HeadYaw", headYawAngle, 0.1)
        # 拿三次mark数据
        markData = findMark()

        if (markData != []):
            tts.say("i saw landmark!")
            yellow_data = findYellowStick_with_mark()
            landmarkFlag = 0  # landmark识别符0代表识别到，1代表未识别到。
            markwzCamera = markData[1][0][0][1]
            markwyCamera = markData[1][0][0][2]
            markangularSize = markData[1][0][0][3]
            headangle = motionPrx.getAngles("HeadYaw", True);
            markheadangle = markwzCamera + headangle[0]
            allmarkdata = [markwzCamera, markwyCamera, markangularSize, markheadangle, landmarkFlag]
            if yellow_data != [] and abs(yellow_data - markwzCamera) <= 0.5:
                markwzCamera = yellow_data
                print yellow_data
                print markwzCamera
            landmarkProxy.unsubscribe("landmarkTest")
            return allmarkdata  # 因为已经返回值了，所以不会执行while循环之后的程序
        else:
            tts.say("马克去哪了?")
        headYawAngle = headYawAngle + 0.5

    allmarkdata = [0, 0, 0, 0, 1]
    landmarkProxy.unsubscribe("landmarkTest")
    return allmarkdata

def secondSearchNAOmark(robotTolandmarkdata):
    camProxy.setActiveCamera(0)
    motionPrx.angleInterpolationWithSpeed("HeadPitch", -0.1, 0.3)
    landmarkProxy.subscribe("landmarkTest")
    alpha = robotTolandmarkdata[0]

    if alpha < 0:
        headYawAngle = (-90 * math.pi / 180.0)
    else:
        headYawAngle = (90 * math.pi / 180.0)

    headYawAngle1 = headYawAngle
    motionPrx.angleInterpolationWithSpeed("HeadYaw", headYawAngle1, 0.1)

    markData = findMark()

    if markData != []:
        tts.say("i saw landmark!")
        yellow_data = findYellowStick_with_mark()
        landmarkFlag = 0  # landmark识别符0代表识别到，1代表未识别到。
        markwzCamera = markData[1][0][0][1]
        markwyCamera = markData[1][0][0][2]
        if yellow_data!=[] and abs(yellow_data - markwzCamera) <= 0.5:
            markwzCamera = yellow_data
        markangularSize = markData[1][0][0][3]
        headangle = motionPrx.getAngles("HeadYaw", True)
        markheadangle = markwzCamera + headangle[0]
        allmarkdata1 = [markwzCamera, markwyCamera, markangularSize, markheadangle, landmarkFlag]


    headYawAngle1 = headYawAngle + 0.6
    motionPrx.angleInterpolationWithSpeed("HeadYaw", headYawAngle1, 0.1)
    time.sleep(1)
    markData = findMark()

    if markData != []:
        tts.say("i saw landmark!")
        landmarkFlag = 0  # landmark识别符0代表识别到，1代表未识别到。
        # Retrieve landmark center position in radians.
        markwzCamera = markData[1][0][0][1]
        markwyCamera = markData[1][0][0][2]
        # Retrieve landmark angular size in radians.
        markangularSize = markData[1][0][0][3]
        headangle = motionPrx.getAngles("HeadYaw", True);
        markheadangle = markwzCamera + headangle[0]
        allmarkdata1 = [markwzCamera, markwyCamera, markangularSize, markheadangle, landmarkFlag]
        return allmarkdata1  # 因为已经返回值了，所以不会执行while循环之后的程序

    headYawAngle1 = headYawAngle - 0.6
    motionPrx.angleInterpolationWithSpeed("HeadYaw", headYawAngle1, 0.1)
    time.sleep(1)
    markData = findMark()
    if (markData != []):
        tts.say("i saw landmark!")
        landmarkFlag = 0  # landmark识别符0代表识别到，1代表未识别到。
        # Retrieve landmark center position in radians.
        markwzCamera = markData[1][0][0][1]
        markwyCamera = markData[1][0][0][2]
        # Retrieve landmark angular size in radians.
        markangularSize = markData[1][0][0][3]
        print markwzCamera
        print markwyCamera
        print markangularSize
        headangle = motionPrx.getAngles("HeadYaw", True);
        print headangle
        markheadangle = markwzCamera + headangle[0]

        print(markheadangle)
        allmarkdata1 = [markwzCamera, markwyCamera, markangularSize, markheadangle, landmarkFlag]
        print allmarkdata1
        return allmarkdata1  # 因为已经返回值了，所以不会执行while循环之后的程序

    print "no mark, no mark!"
    landmarkFlag = 1  # landmark识别符0代表识别到，1代表未识别到。
    allmarkdata1 = [0, 0, 0, 0, landmarkFlag]
    landmarkProxy.unsubscribe("landmarkTest")
    return allmarkdata1

def TriangleCalculation(x, s, alpha):
    s = s + 0.055 + 0.075
    if (alpha < 0.0):
        print "alpha" + str(alpha)  # str函数，将任意值转化为字符串
        alpha = abs(alpha)
        l2 = x * x + s * s - 2 * x * s * math.cos(alpha)  # 凭什么认为球和NAOmark的距离就为根号12？
        l = math.sqrt(l2)  # 球和NAOmark的距离
        costheta = (x * x + l2 - s * s) / (2 * x * l)
        theta = math.acos(costheta)
        print "theta" + str(theta)
        if theta >= math.pi / 2:
            theta2 = theta - math.pi / 2
            turnAngle1 = theta2  # 完全可以
            dis1 = 0 - (x * math.sin(theta2) + 0.10)  # 修正值0.0
            turnAngle2 = 0
            dis2 = x * math.cos(theta2)
            print dis1
            print dis2
            print("alpha<0.theta>pi/2")  # 2

        if theta < math.pi / 2:
            theta2 = math.pi / 2 - theta
            turnAngle1 = 0 - theta2  # 可以
            dis1 = x * math.cos(theta) + 0.20  # 修正值
            turnAngle2 = 0
            dis2 = x * math.sin(theta) + 0.03
            print dis1
            print dis2
            print("alpha<0.theta<pi/2")  # 3

        turndata = [dis1, dis2, turnAngle1, turnAngle2]
        return turndata
    if (alpha > 0.0):
        print "alpha" + str(alpha)
        l2 = x * x + s * s - 2 * x * s * math.cos(alpha)
        l = math.sqrt(l2)
        costheta = (x * x + l2 - s * s) / (2 * x * l)
        theta = math.acos(costheta)
        print "theta" + str(theta)
        if theta >= math.pi / 2:
            theta2 = theta - math.pi / 2
            turnAngle1 = 0 - theta2  # 可以
            dis1 = x * math.sin(theta2)  # 修正值
            turnAngle2 = 0
            dis2 = x * math.cos(theta2)
            print dis1
            print dis2
            print("alpha>0.theta>pi/2")  # 1

        if theta < math.pi / 2:
            theta2 = math.pi / 2 - theta
            turnAngle1 = theta2
            dis1 = -x * math.cos(theta)
            turnAngle2 = 0
            dis2 = x * math.sin(theta)
            print dis1
            print dis2
            print("alpha>0.theta<pi/2")  # 4
        turndata = [dis1, dis2, turnAngle1, turnAngle2]  # turnangle1一定是个绝对值0-90度的值，turnangle2一定是个0
        return turndata

        # ------------------------------------------- 调整击球位置 -----------------------------------------#

# 调节身体---------------------------------#
def refixRound(angle):
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0, 0.5)
    motionPrx.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
    motionPrx.setMoveArmsEnabled(False, False)
    limit = 0.4
    if angle >= 0:
        while angle > limit:
            angle -= limit
            motionPrx.moveTo(0, 0, limit)
            time.sleep(1)
    else:
        while angle < -limit:
            angle += limit
            motionPrx.moveTo(0, 0, -limit)
    motionPrx.moveTo(0, 0, angle)

def AdjustPosition(turndata):
    h = 0.478
    dis1 = turndata[0]
    dis2 = turndata[1]
    turnAngle1 = turndata[2]
    print turnAngle1
    print("keyi")
    turnAngle2 = turndata[3]
    # 机器人行走参数
    maxstepx = 0.02
    maxstepy = 0.14
    maxsteptheta = 0.15
    maxstepfrequency = 0.6
    stepheight = 0.02
    torsowx = 0.0
    torsowy = 0.0
    motionPrx.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.3)
    motionPrx.setMoveArmsEnabled(False, False)

    if (turnAngle1<0):
        turnAngle1=turnAngle1+0.2

    motionPrx.moveTo(0.0, 0.0, turnAngle1,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    motionPrx.moveTo(-0.21, 0.0, 0.0,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    motionPrx.setMoveArmsEnabled(False, False)
    # 加一段程序
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3)
    motionPrx.angleInterpolationWithSpeed("HeadPitch", 0.5236, 0.3)
    currentCamera = "CameraBottom"
    camProxy.setActiveCamera(1)
    time.sleep(2)
    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
    # 这个x决定了长度
    x = (h - 0.03) / (math.tan(thetav)) - 0.11
    # 改正位置
    # y = ((h - 0.03) / math.tan(thetav)) * math.sin(thetah)
    ##修改：
    y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah)
    print "改后y:", y
    print y
    if (y > 0):
        # y = ((h - 0.03) / math.tan(thetav)) * math.sin(thetah) + 0.05
        ##修改：
        y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah)+0.05
    else:
        # y = ((h - 0.03) / math.tan(thetav)) * math.sin(thetah) + 0.03
        ##修改：
        y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah)+0.03
    print(y)
    # 改正结束
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.moveTo(x, 0.0, 0.0,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])

    motionPrx.moveTo(0.0, y, 0.0,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])

    # 添加结束

def AdjustPosition2():
    global _stepStatue2
    h = 0.478
    # 机器人行走参数

    # motionPrx.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.3)
    motionPrx.setMoveArmsEnabled(False, False)
    # 加一段程序
    val = findball(0, 30)
    while val == []:
        motionPrx.moveTo(-0.05, 0, 0, _stepStatue2)
        val = findball(0, 30)
    ballinfo = val[1][1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (69.7 * math.pi / 180.0)

    x = (h - 0.03) / (math.tan(thetav)) - 0.11
    y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah)
    if (y > 0):
        y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah)+0.05
    else:
        y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah)+0.03
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.moveTo(x, 0.0, 0.0,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])

    motionPrx.moveTo(0.0, y, 0.0,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])

def TurnAfterHitball():
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    # 机器人行走参数
    maxstepx = 0.04
    maxstepy = 0.14
    maxsteptheta = 0.3
    maxstepfrequency = 0.6
    stepheight = 0.02
    torsowx = 0.0
    torsowy = 0.0
    if (alpha < 0.0):
        motionPrx.setMoveArmsEnabled(False, False)
        motionPrx.moveTo(0.18, 0.0, 0.0,
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
        motionPrx.setMoveArmsEnabled(False, False)
        motionPrx.moveTo(0.0, 0.0, -1.33,
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
        motionPrx.setMoveArmsEnabled(False, False)
        """
        motionPrx.moveTo(1, 0.0, 0.0,  # 此处参数可调，设置搜寻距离
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
        motionPrx.moveTo(0.0, 0.0, -0.24,
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
        motionPrx.setMoveArmsEnabled(False, False)
        """

    if (alpha > 0.0):
        motionPrx.setMoveArmsEnabled(False, False)
        motionPrx.moveTo(0.18, 0.0, 0.0,
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
        motionPrx.setMoveArmsEnabled(False, False)
        motionPrx.moveTo(0.0, 0.0, 1.45,
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
        motionPrx.setMoveArmsEnabled(False, False)

def TurnAfterHitball1():
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    # 机器人行走参数
    global _stepStatue
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.moveTo(0.0, 0.0, -1.33, _stepStatue)
    motionPrx.setMoveArmsEnabled(False, False)
    for i in range(4):
        motionPrx.moveTo(0.25, 0.0, 0.0, _stepStatue)
        time.sleep(0.5)

def TurnAfterHitball2():
    global _stepStatue
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    # 机器人行走参数

    motionPrx.setMoveArmsEnabled(False, False)

    motionPrx.moveTo(0.15, 0.0, 0.0, _stepStatue)
    time.sleep(0.5)

    motionPrx.moveTo(0.0, 0.0, -0.5 * math.pi, _stepStatue)
    time.sleep(0.5)

    # 直走0.6找次黄杆
    motionPrx.moveTo(0.3, 0.0, 0.0, _stepStatue)
    time.sleep(0.5)
    motionPrx.moveTo(0.3, 0.0, 0.0, _stepStatue)
    flag = towordYellowStick32()
    motionPrx.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0, 0.5)
    # 如果第一个0.6没有找到黄杆, 0.9时对一次黄杆
    motionPrx.moveTo(0.3, 0.0, 0.0, _stepStatue)
    time.sleep(0.5)
    if flag == -100:
        flag = towordYellowStick32()
        motionPrx.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
        motionPrx.angleInterpolationWithSpeed("HeadYaw", 0, 0.5)
    for i in range(3):
        motionPrx.moveTo(0.3, 0.0, 0.0, _stepStatue)
        time.sleep(0.5)
    towordYellowStick32()

memoryProxy = ALProxy("ALMemory", robotIP, PORT)  # memory  object
motionPrx = ALProxy("ALMotion", robotIP, PORT)  # motion  object
tts = ALProxy("ALTextToSpeech", robotIP, PORT)
postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

Lifestop = ALProxy("ALAutonomousLife", robotIP, PORT)
Lifestop.setState("disabled")

redballProxy = ALProxy("ALRedBallDetection", robotIP, PORT)
camProxy = ALProxy("ALVideoDevice", robotIP, PORT)
landmarkProxy = ALProxy("ALLandMarkDetection", robotIP, PORT)
StiffnessOn(motionPrx)
postureProxy.goToPosture("StandInit", 1.0)

zhuagan()  # 抓杆"""

headtouch()  # 触摸前额开始
time.sleep(1)
alpha = -math.pi / 2  # 初始值
time.sleep(1)
kaiBall()  # 击球第一杆
time.sleep(1.5)

shougang()
LShoulderpitchAmend()
ShoulderpitchAmend2()
time.sleep(2)

TurnAfterHitball1()# 旋转找球

while True:
    allballData = firstSearchredball()  # 红球定位 返回红球位置信息
    if (allballData[2] == 0):
        break

CalculateRobotToRedball1(allballData)
AdjustPosition2()
verticalToYellowStick()
# 可能没有看到球
AdjustPosition2()
verticalToYellowStick()
Reduction()
Reduction1()
Reduction2()
time.sleep(0.5)
HitBall22()

time.sleep(1.5)
shougang()
LShoulderpitchAmend()
ShoulderpitchAmend2()

# motionPrx.moveTo(0, 0, -1 / 12.0 * math.pi)
TurnAfterHitball2()

while True:
    while True:
        allballData = firstSearchredball()
        if (allballData[2] == 0):
            break
    x, fai = CalculateRobotToRedball(allballData)
    # 我们直接在这里进行优化
    # x = 26
    # 第一次找MARK

    allmarkdata = firstSearchNAOmark()  # NAOmark定位
    if (allmarkdata[4] == 0):
        robotTolandmarkdata = robotTolandmark(allmarkdata)  # 返回夹角angle和机器人与mark距离
        s = robotTolandmarkdata[1]  # 获取机器人到naomark距离
        alpha = robotTolandmarkdata[0]   # 获取 三角形夹角
        tts.say("快测")
        print "mmmmmmmmmmmmmmm", s, x, alpha
        headtouch()
        turndata = TriangleCalculation(x, s, alpha)  # 计算旋转数据
        AdjustPosition(turndata)  # 调整机器人位置
        markangle1 = allmarkdata[3]

        # 第二次找MARK(如果第一次算错,第二次只会让机器人更错)
        h = 0.478
        allmarkdata = secondSearchNAOmark(robotTolandmarkdata)
        if (allmarkdata[4] == 0):
            markangle2 = allmarkdata[3]
            markangle = markangle1 - markangle2
            turnAngle2 = turndata[2] - markangle
            print turnAngle2

            # if (abs(turnAngle2) < 0.1):
            #     turnAngle2 = turnAngle2 * 2

            maxstepx = 0.02
            maxstepy = 0.14
            maxsteptheta = 0.15
            maxstepfrequency = 0.6
            stepheight = 0.02
            torsowx = 0.0
            torsowy = 0.0
            motionPrx.setMoveArmsEnabled(False, False)
            motionPrx.moveTo(0.0, 0.0, turnAngle2,
                             [["MaxStepX", maxstepx],
                              ["MaxStepY", maxstepy],
                              ["MaxStepTheta", maxsteptheta],
                              ["MaxStepFrequency", maxstepfrequency],
                              ["StepHeight", stepheight],
                              ["TorsoWx", torsowx],
                              ["TorsoWy", torsowy]])
            motionPrx.setMoveArmsEnabled(False, False)

            # 加一段程序
            motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3)
            motionPrx.angleInterpolationWithSpeed("HeadPitch", 0.5236, 0.3)
            currentCamera = "CameraBottom"
            camProxy.setActiveCamera(1)
            time.sleep(2)
            val = memoryProxy.getData("redBallDetected")
            ballinfo = val[1]
            thetah = ballinfo[0]
            thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
            x = abs((h - 0.03) / (math.tan(thetav))) * math.cos(thetah) - 0.11
            y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah)
            if (y > 0):
                y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah) + 0.05
            else:
                y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah) + 0.03
            print(y)
            motionPrx.setMoveArmsEnabled(False, False)
            motionPrx.moveTo(x, 0.0, 0.0,
                             [["MaxStepX", maxstepx],
                              ["MaxStepY", maxstepy],
                              ["MaxStepTheta", maxsteptheta],
                              ["MaxStepFrequency", maxstepfrequency],
                              ["StepHeight", stepheight],
                              ["TorsoWx", torsowx],
                              ["TorsoWy", torsowy]])

            motionPrx.moveTo(0.0, y, 0.0,
                             [["MaxStepX", maxstepx],
                              ["MaxStepY", maxstepy],
                              ["MaxStepTheta", maxsteptheta],
                              ["MaxStepFrequency", maxstepfrequency],
                              ["StepHeight", stepheight],
                              ["TorsoWx", torsowx],
                              ["TorsoWy", torsowy]])

            # 添加结束

            allmarkdata = secondSearchNAOmark(robotTolandmarkdata)
            # 如果第一次错了, 第三次也是保证更错
            if (allmarkdata[4] == 0):
                markangle3 = allmarkdata[3]
                markangle = markangle1 - markangle3
                turnAngle3 = turndata[2] - markangle
                maxstepx = 0.02
                maxstepy = 0.14
                maxsteptheta = 0.15
                maxstepfrequency = 0.6
                stepheight = 0.02
                torsowx = 0.0
                torsowy = 0.0
                motionPrx.setMoveArmsEnabled(False, False)
                motionPrx.moveTo(0.0, 0.0, turnAngle3,
                                 [["MaxStepX", maxstepx],
                                  ["MaxStepY", maxstepy],
                                  ["MaxStepTheta", maxsteptheta],
                                  ["MaxStepFrequency", maxstepfrequency],
                                  ["StepHeight", stepheight],
                                  ["TorsoWx", torsowx],
                                  ["TorsoWy", torsowy]])
                motionPrx.setMoveArmsEnabled(False, False)
            # 如果α>0让机器人第四次搜寻红球, 应该也是更错
            if alpha > 0:
                allmarkdata = secondSearchNAOmark(robotTolandmarkdata)
                if (allmarkdata[4] == 0):
                    markangle3 = allmarkdata[3]
                    markangle = markangle1 - markangle3
                    turnAngle3 = turndata[2] - markangle
                    print turnAngle3
                    print ("keyi3")
                    
                    maxstepx = 0.02
                    maxstepy = 0.14
                    maxsteptheta = 0.15
                    maxstepfrequency = 0.6
                    stepheight = 0.02
                    torsowx = 0.0
                    torsowy = 0.0
                    motionPrx.setMoveArmsEnabled(False, False)
                    motionPrx.moveTo(0.0, 0.0, turnAngle3,
                                     [["MaxStepX", maxstepx],
                                      ["MaxStepY", maxstepy],
                                      ["MaxStepTheta", maxsteptheta],
                                      ["MaxStepFrequency", maxstepfrequency],
                                      ["StepHeight", stepheight],
                                      ["TorsoWx", torsowx],
                                      ["TorsoWy", torsowy]])
                    motionPrx.setMoveArmsEnabled(False, False)

        # taishou()

        Reduction()
        Reduction1()
        Reduction2()
        time.sleep(0.5)
        # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.4)
        time.sleep(1)
        HitBall()

        time.sleep(1.5)
        shougang()
        LShoulderpitchAmend()
        ShoulderpitchAmend2()
        time.sleep(2)

        # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR7,0.2)
        # shougang()
        TurnAfterHitball()  # 击球后旋转
    else:
        verticalToYellowStick_withoutMark()
        AdjustPosition2()
        alpha = verticalToYellowStick_withoutMark() - 0.5 * math.pi
        if alpha > 0:
            motionPrx.moveTo(0, 0, 5 / 180.0 * math.pi)
        else:
            motionPrx.moveTo(0, 0, -5 / 180.0 * math.pi)
        AdjustPosition2()
        Reduction()
        Reduction1()
        Reduction2()
        # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.4)
        time.sleep(1)
        HitBall()  # 击球

        time.sleep(1.5)
        shougang()
        LShoulderpitchAmend()
        ShoulderpitchAmend2()
        time.sleep(2)

        TurnAfterHitball()
