# encoding: utf-8
import os
import re
from naoqi import ALProxy
import time

PositionJointNamesR = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
PositionJointNamesL = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"]
golfPositionJointAnglesR1 = [1.4172073526193958, 0.061086523819801536, 1.6022122533307945, 1.4835298641951802,
                             0.038397243543875255, 0.12]
golfPositionJointAnglesR2 = [1.4172073526193958, 0.061086523819801536, 1.6022122533307945, 1.4835298641951802,
                             -0.538397243543875255, 0.12]
golfPositionJointAnglesR31 = [1.35787, 0.05760, 1.50098, 1.50971, 0.767945, 0.12]
golfPositionJointAnglesR111 = [1.35787, 0.05760, 1.50098, 1.50971, -0.738397, 0.12]
golfPositionJointAnglesR11 = [1.35787, 0.05760, 1.50098, 1.50971, -0.538397, 0.12]
golfPositionJointAnglesR32 = [1.35787, 0.05760, 1.50098, 1.50971, -0.767944870877505, 0.12]
golfPositionJointAnglesR12 = [1.35787, 0.05760, 1.50098, 1.50971, 0.767944870877505, 0.12]
golfPositionJointAnglesR42 = [1.02974, 0.24958, 1.61094, 1.10828, -0.43633, 0.12]
golfPositionJointAnglesR5 = [1.3508848410436112, -0.36302848441482055, 1.4974924982111346, 1.5446163880149815,
                             0.30019663134302466, 0.6]  # 松杆参数
golfPositionJointAnglesR6 = [1.3508848410436112, -0.36302848441482055, 1.4974924982111346, 1.5446163880149815,
                             0.30019663134302466, 0.12]  # 抓杆参数

golfPositionJointAnglesR7 = [0.9508848410436112, -0.36302848441482055, 1.4974924982111346, 1.5446163880149815,
                             0.30019663134302466, 0.12]
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



def stop_process(process_ids):
    for process in process_ids:
        print process_ids
        os.system("kill -STOP " + process)

def continue_process(process_ids):
	for process in process_ids:
		os.system("kill -CONT " + process)

def headtouch():
    while True:
        print "try to fix"
        headTouchedButtonFlag = memoryProxy.getData("FrontTactilTouched")
        if headTouchedButtonFlag == 1.0:
            break
    print("头部前传感器收到信号")
 
def zhuagan():
    while True:
        # RighthandTouchedFlag = memoryProxy.getData("HandRightRightTouched")
        headTouchedmidlleFlag = memoryProxy.getData("MiddleTactilTouched")
        if headTouchedmidlleFlag == 1.0:
            print "right hand touched"
            tts.say("请给我一个球杆，好吗")
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR5, 0.2)
            time.sleep(4)
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR6, 0.2)
            print("抓杆完成")
            break
     
def shougang():  # 收杆
    names = list()
    keys = list()
    times = list()
    names.append("RWristYaw")  #
    times.append([1, 2])
    keys.append([1.632374, 1.632374])
    names.append("LElbowRoll")  #
    times.append([0.5, 1])
    keys.append([-0.6, -0.17453292519943295])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("HeadYaw")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LAnklePitch")
    times.append([1, 2, 3, 4])
    keys.append([-0.349794, -0.349794, -0.349794, -0.349794])

    names.append("LAnkleRoll")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LElbowRoll")  #
    times.append([1, 2, 3, 4])
    keys.append([-0.321141, -0.321141, -0.958791, -0.958791])

    names.append("LElbowYaw")  #
    times.append([1, 2, 3, 4])
    keys.append([-1.37757, -1.37757, -1.466076, -1.466076])

    names.append("LHand")  #
    times.append([1, 2, 3, 4, 5.2])
    keys.append([0.6000, 0.6000, 0.6000, 0.6000, 0.1800])

    names.append("LHipPitch")
    times.append([1, 2, 3, 4])
    keys.append([-0.450955, -0.450955, -0.450955, -0.450955])

    names.append("LHipRoll")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LHipYawPitch")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LKneePitch")
    times.append([1, 2, 3, 4])
    keys.append([0.699462, 0.699462, 0.699462, 0.699462])

    names.append("LShoulderPitch")  #
    times.append([1, 2, 3, 4, 5.2])
    keys.append([1.53885, 1.43885, 1.24856, 1.24856, 1.24856])

    names.append("LShoulderRoll")  #
    times.append([1, 2, 3, 4, 5.2])
    keys.append([0.268407, 0.268407, -0.04014, -0.04014, -0.04014])

    names.append("LWristYaw")  #
    times.append([1, 2, 3, 4])
    keys.append([-0.016916, -0.016916, -1.632374, -1.632374])

    names.append("RAnklePitch")
    times.append([1, 2, 3, 4])
    keys.append([-0.354312, -0.354312, -0.354312, -0.354312])

    names.append("RAnkleRoll")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RElbowRoll")  #
    times.append([1, 2, 3, 4])
    keys.append([0.958791, 0.958791, 0.958791, 0.958791])

    names.append("RElbowYaw")  #
    times.append([1, 2, 3, 4])
    keys.append([1.466076, 1.466076, 1.466076, 1.466076])

    names.append("RHand")
    times.append([1, 2, 3, 4])
    keys.append([0.0900, 0.0900, 0.0900, 0.0900])

    names.append("RHipPitch")
    times.append([1, 2, 3, 4])
    keys.append([-0.451038, -0.451038, -0.451038, -0.451038])

    names.append("RHipRoll")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RHipYawPitch")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RKneePitch")
    times.append([1, 2, 3, 4])
    keys.append([0.699545, 0.699545, 0.699545, 0.699545])

    names.append("RShoulderPitch")  #
    times.append([1, 2, 3, 4, 5.2])
    keys.append([1.03856, 1.03856, 1.03856, 1.03856, 1.03856])

    names.append("RShoulderRoll")  #
    times.append([1, 2, 3, 4, 5.2])
    keys.append([0.04014, 0.04014, 0.04014, 0.04014, 0.04014])

    names.append("RWristYaw")  #
    times.append([1, 2, 3, 4])
    keys.append([1.632374, 1.632374, 1.632374, 1.632374])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)
  
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
     


if __name__ == '__main__':
     
    robotIP = "192.168.1.105"
    postureProxy = ALProxy("ALRobotPosture", robotIP, 9559) 
    Lifestop = ALProxy("ALAutonomousLife", robotIP, 9559)
    memoryProxy = ALProxy("ALMemory", robotIP, 9559)
    tts = ALProxy("ALTextToSpeech", robotIP, 9559)
    tts = ALProxy("ALTextToSpeech", robotIP, 9559)
    motionPrx = ALProxy("ALMotion", robotIP, 9559)

    print "good luck"
    os.system("ps -aux|grep '/usr/bin/python2.7 r125_[1, 2, 3].py' > 1")
    f = open("1")
    txt = f.read()
    reg = re.compile("nao.*?(\d.*?) ", re.S)
    process_ids = re.findall(reg, txt)

    while True:
        while True:
            posture = postureProxy.getPostureFamily()
            if posture == "LyingBelly" or posture == "LyingBack":
                if len(process_ids) != 0:
                    stop_process(process_ids)
                break
            time.sleep(0.5)

        motionPrx.post.angleInterpolation('RHand', 0.6, 0.1, True)
        time.sleep(3)
        Lifestop.setState("solitary")
        headtouch()
        Lifestop.setState("disabled")
        postureProxy.goToPosture("StandInit", 1.0)
        zhuagan()
        headtouch()
        shougang()
        LShoulderpitchAmend()
        ShoulderpitchAmend2()
        tts.say("我将再次运行")
        if len(process_ids) != 0:
            continue_process(process_ids)



