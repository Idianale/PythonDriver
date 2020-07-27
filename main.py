# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Example of Python controller for Nao robot.
   This demonstrates how to access sensors and actuators"""

from random import uniform
from controller import Robot, Keyboard, Motion
import time

class Nao (Robot):
    PHALANX_MAX = 8
    poses = {
        # First Pose
        'NeutralHandsDown':
            [
                #Head Pitch | Head Yaw
                0,0,
                #RElbow Roll | RElbow Yaw | RShoulderPitch | RShoulderRoll
                0, 0, 1.49797, 0,
                #RAnklePitch | AnklRoll | KneePitch
                0,0,0,
                #RHip Pitch | RHip Roll | RHipYawPitch
                0, 0, 0,
                #LElbow Roll | LElbow Yaw | LShoulderPitch | LShoulderRoll
                0, 0, 1.49797, 0,
                #LAnklePitch | LAnklRoll | LKneePitch
                0, 0, 0,
                #LHip Pitch | LHip Roll | LHipYawPitch
                0, 0, 0,
                #Lights
                0x000000
            ],
        'NeutralHandsUp':
            [
                # Head Pitch | Head Yaw
                0, 0,
                # RElbow Roll | RElbow Yaw | RShoulderPitch | RShoulderRoll
                0, 0, 0, 0,
                # RAnklePitch | AnklRoll | KneePitch
                0, 0, 0,
                # RHip Pitch | RHip Roll | RHipYawPitch
                0, 0, 0,
                # LElbow Roll | LElbow Yaw | LShoulderPitch | LShoulderRoll
                0, 0, 0, 0,
                # LAnklePitch | LAnklRoll | LKneePitch
                0, 0, 0,
                # LHip Pitch | LHip Roll | LHipYawPitch
                0, 0, 0,
                # Lights
                0x000000
            ],
        'SemiDab':
            [
                # Head Pitch | Head Yaw
                -0.164347,1,
                # RElbow Roll | RElbow Yaw | RShoulderPitch | RShoulderRoll
                0, 0.214419, 0.25, -1.33,
                # RAnklePitch | AnklRoll | KneePitch
                0,0,0,
                # RHip Pitch | RHip Roll | RHipYawPitch
                0,0,0,
                # LElbow Roll | LElbow Yaw | LShoulderPitch | LShoulderRoll
                -1, 0.203155, -0.531762, 0,
                # LAnklePitch | LAnklRoll | LKneePitch
                0,0,0,
                # LHip Pitch | LHip Roll | LHipYawPitch
                0,0,0,
                # Lights
                0x00ff00 # green
            ],
        "HandsUpHeadAway":
            [
                # Head Pitch | Head Yaw
                0.394981, 1.496369,
                # RElbow Roll | RElbow Yaw | RShoulderPitch | RShoulderRoll
                0.664048, -1.003231, -1.735979, 0,
                # RAnklePitch | AnklRoll | KneePitch
                0,0,0,
                # RHip Pitch | RHip Roll | RHipYawPitch
                0,0,0
                # LElbow Roll | LElbow Yaw | LShoulderPitch | LShoulderRoll
                -1.021001,-0.129176,-1.04238,0,
                # LAnklePitch | LAnklRoll | LKneePitch
                0,0,0,
                # LHip Pitch | LHip Roll | LHipYawPitch
                0,0,0,
                # Lights
                0x0000ff
            ],
        "KneeBentHeadUp":
            [
                # Head Pitch | Head Yaw
                -0.442392, 1.191726,
                # RElbow Roll | RElbow Yaw | RShoulderPitch | RShoulderRoll
                0,1.498829, 1.965338, 0,
                # RAnklePitch | AnkleRoll | KneePitch
                0, 0, 0,
                # RHip Pitch | RHip Roll | RHipYawPitch
                0, 0, -0,
                # LElbow Roll | LElbow Yaw | LShoulderPitch | LShoulderRoll
                -1.021001, -0.129176, -1.04238, 0,
                # LAnklePitch | LAnklRoll | LKneePitch
                0,0,0,
                # LHip Pitch | LHip Roll | LHipYawPitch
                0,0,0,
                # Lights
                0xff0000
            ],
        "Other":
            [
                # Head Pitch | Head Yaw
                -0.442392, 1.191726,
                # RElbow Roll | RElbow Yaw | RShoulderPitch | RShoulderRoll
                0, 1.498829, 1.965338, 0,
                # RAnklePitch | AnkleRoll | KneePitch
                0, 0, 0,
                # RHip Pitch | RHip Roll | RHipYawPitch
                0, 0, -0,
                # LElbow Roll | LElbow Yaw | LShoulderPitch | LShoulderRoll
                -1.021001, -0.129176, -1.04238, 0,
                # LAnklePitch | LAnklRoll | LKneePitch
                0, 0, 0,
                # LHip Pitch | LHip Roll | LHipYawPitch
                0, 0, 0,
                # Lights
                0xff0000
        ],
        "ArmsAwaySideView":
            [
                # Head Pitch | Head Yaw
                -0.442392, 1.191726,
                # RElbow Roll | RElbow Yaw | RShoulderPitch | RShoulderRoll
                0, 0, 2.09, 0,
                # RAnklePitch | AnkleRoll | KneePitch
                0, 0, 0,
                # RHip Pitch | RHip Roll | RHipYawPitch
                0, 0, -0,
                # LElbow Roll | LElbow Yaw | LShoulderPitch | LShoulderRoll
                0, 0, 2.09, 0,
                # LAnklePitch | LAnklRoll | LKneePitch
                0, 0, 0,
                # LHip Pitch | LHip Roll | LHipYawPitch
                0, 0, 0,
                # Lights
                0xff0000
            ],
        "ArmsFlex":
            [
            # Head Pitch | Head Yaw
            0,0,
            # RElbow Roll | RElbow Yaw | RShoulderPitch | RShoulderRoll
            1.5, 0.214419,-1.5,-1.5,
            # RAnklePitch | AnklRoll | KneePitch
            0, 0, 0,
            # RHip Pitch | RHip Roll | RHipYawPitch
            0, 0, 0,
            # LElbow Roll | LElbow Yaw | LShoulderPitch | LShoulderRoll
            #-1, 0.203155, -1.75, -1.75,
            -1.5, -0.25, -1.5, 1.5,
            # LAnklePitch | LAnklRoll | LKneePitch
            0, 0, 0,
            # LHip Pitch | LHip Roll | LHipYawPitch
            0, 0, 0,
            # Lights
            0xff0000  # Red
        ],
        "HandBehind":
            [
            # Head Pitch | Head Yaw
            0.330384,-0.315484,
            # RElbow Roll | RElbow Yaw | RShoulderPitch | RShoulderRoll
            1.134315,1.698359,-1.996279, 0,
            # RAnklePitch | AnklRoll | KneePitch
            0,0,0,
            # RHip Pitch | RHip Roll | RHipYawPitch
            0,0,0,
            # LElbow Roll | LElbow Yaw | LShoulderPitch | LShoulderRoll
            -1, 0.23, 0.442072, 0.442072,
            # LAnklePitch | LAnklRoll | LKneePitch
            0,0,0,
            # LHip Pitch | LHip Roll | LHipYawPitch
            0,0,0,
            # Lights
            0xffff00
        ],
        "LookAtHand":
            [
            # Head Pitch | Head Yaw
            0.419583, 0.874915,
            # RElbow Roll | RElbow Yaw | RShoulderPitch | RShoulderRoll
            0.705741, 1.497226, 1.533920, 1.533920,
            # RAnklePitch | AnklRoll | KneePitch
            0, 0, 0,
            # RHip Pitch | RHip Roll | RHipYawPitch
            0, 0, 0,
            # LElbow Roll | LElbow Yaw | LShoulderPitch | LShoulderRoll
            -1 , 0.23 , 0.404123 , 0.404123,
            # LAnklePitch | LAnklRoll | LKneePitch
            0, 0, 0,
            # LHip Pitch | LHip Roll | LHipYawPitch
            0, 0, 0,
            # Lights
            0x0000ff
        ],
        "SpeechMode":
        [
            # Head Pitch | Head Yaw
            -0.606911, 0,
            # RElbow Roll | RElbow Yaw | RShoulderPitch | RShoulderRoll
            0, 1.887531, -2.04, -2.04,
            # RAnklePitch | AnklRoll | KneePitch
            0, 0, 0,
            # RHip Pitch | RHip Roll | RHipYawPitch
            0, 0, 0,
            # LElbow Roll | LElbow Yaw | LShoulderPitch | LShoulderRoll
            0, -0.812530, 0.324120, 0.324120,
            # LAnklePitch | LAnklRoll | LKneePitch
            0, 0, 0,
            # LHip Pitch | LHip Roll | LHipYawPitch
            0, 0, 0,
            # Lights
            0x00ff00
        ],
        "Attention":
        [
            # Head Pitch | Head Yaw
            -0.148474, 0.1,
            # RElbow Roll | RElbow Yaw | RShoulderPitch | RShoulderRoll
            0.988020, -0.082400, 0.922309, 0.922309,
            # RAnklePitch | AnklRoll | KneePitch
            0, 0, 0,
            # RHip Pitch | RHip Roll | RHipYawPitch
            0, 0, 0,
            # LElbow Roll | LElbow Yaw | LShoulderPitch | LShoulderRoll
            0, 0, 1.49797, 0,
            # LAnklePitch | LAnklRoll | LKneePitch
            0, 0, 0,
            # LHip Pitch | LHip Roll | LHipYawPitch
            0, 0, 0,
            # Lights
            0x00ff00
        ]

    }

    # load motion files
    def loadMotionFiles(self):
        self.handWave = Motion('softbank/nao/motions/HandWave.motion')
        self.forwards = Motion('softbank/nao/motions/Forwards50.motion')
        self.backwards = Motion('softbank/nao/motions/Backwards.motion')
        self.sideStepLeft = Motion('softbank/nao/motions/SideStepLeft.motion')
        self.sideStepRight = Motion('softbank/nao/motions/SideStepRight.motion')
        self.turnLeft60 = Motion('softbank/nao/motions/TurnLeft60.motion')
        self.turnRight60 = Motion('softbank/nao/motions/TurnRight60.motion')

    def startMotion(self, motion):
        # interrupt current motion
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()

        # start new motion
        motion.play()
        self.currentlyPlaying = motion

    # the accelerometer axes are oriented as on the real robot
    # however the sign of the returned values may be opposite
    def printAcceleration(self):
        acc = self.accelerometer.getValues()
        print('----------accelerometer----------')
        print('acceleration: [ x y z ] = [%f %f %f]' % (acc[0], acc[1], acc[2]))

    # the gyro axes are oriented as on the real robot
    # however the sign of the returned values may be opposite
    def printGyro(self):
        vel = self.gyro.getValues()
        print('----------gyro----------')
        # z value is meaningless due to the orientation of the Gyro
        print('angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1]))

    def printGps(self):
        p = self.gps.getValues()
        print('----------gps----------')
        print('position: [ x y z ] = [%f %f %f]' % (p[0], p[1], p[2]))

    # the InertialUnit roll/pitch angles are equal to naoqi's AngleX/AngleY
    def printInertialUnit(self):
        rpy = self.inertialUnit.getRollPitchYaw()
        print('----------inertial unit----------')
        print('roll/pitch/yaw: [%f %f %f]' % (rpy[0], rpy[1], rpy[2]))

    def printFootSensors(self):
        fsv = []  # force sensor values

        fsv.append(self.fsr[0].getValues())
        fsv.append(self.fsr[1].getValues())

        l = []
        r = []

        newtonsLeft = 0
        newtonsRight = 0

        # The coefficients were calibrated against the real
        # robot so as to obtain realistic sensor values.
        l.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Front Left
        l.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Front Right
        l.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Rear Right
        l.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Rear Left

        r.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Front Left
        r.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Front Right
        r.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Rear Right
        r.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Rear Left

        for i in range(0, len(l)):
            l[i] = max(min(l[i], 25), 0)
            r[i] = max(min(r[i], 25), 0)
            newtonsLeft += l[i]
            newtonsRight += r[i]

        print('----------foot sensors----------')
        print('+ left ---- right +')
        print('+-------+ +-------+')
        print('|' + str(round(l[0], 1)) +
              '  ' + str(round(l[1], 1)) +
              '| |' + str(round(r[0], 1)) +
              '  ' + str(round(r[1], 1)) +
              '|  front')
        print('| ----- | | ----- |')
        print('|' + str(round(l[3], 1)) +
              '  ' + str(round(l[2], 1)) +
              '| |' + str(round(r[3], 1)) +
              '  ' + str(round(r[2], 1)) +
              '|  back')
        print('+-------+ +-------+')
        print('total: %f Newtons, %f kilograms'
              % ((newtonsLeft + newtonsRight), ((newtonsLeft + newtonsRight) / 9.81)))

    def printFootBumpers(self):
        ll = self.lfootlbumper.getValue()
        lr = self.lfootrbumper.getValue()
        rl = self.rfootlbumper.getValue()
        rr = self.rfootrbumper.getValue()
        print('----------foot bumpers----------')
        print('+ left ------ right +')
        print('+--------+ +--------+')
        print('|' + str(ll) + '  ' + str(lr) + '| |' + str(rl) + '  ' + str(rr) + '|')
        print('|        | |        |')
        print('|        | |        |')
        print('+--------+ +--------+')

    def printUltrasoundSensors(self):
        dist = []
        for i in range(0, len(self.us)):
            dist.append(self.us[i].getValue())

        print('-----ultrasound sensors-----')
        print('left: %f m, right %f m' % (dist[0], dist[1]))

    def printCameraImage(self, camera):
        scaled = 2  # defines by which factor the image is subsampled
        width = camera.getWidth()
        height = camera.getHeight()

        # read rgb pixel values from the camera
        image = camera.getImage()

        print('----------camera image (gray levels)---------')
        print('original resolution: %d x %d, scaled to %d x %f'
              % (width, height, width / scaled, height / scaled))

        for y in range(0, height // scaled):
            line = ''
            for x in range(0, width // scaled):
                gray = camera.imageGetGray(image, width, x * scaled, y * scaled) * 9 / 255  # rescale between 0 and 9
                line = line + str(int(gray))
            print(line)

    def setAllLedsColor(self, rgb):
        # these leds take RGB values
        for i in range(0, len(self.leds)):
            self.leds[i].set(rgb)

        # ear leds are single color (blue)
        # and take values between 0 - 255
        self.leds[5].set(rgb & 0xFF)
        self.leds[6].set(rgb & 0xFF)

    def setHandsAngle(self, angle):
        for i in range(0, self.PHALANX_MAX):
            clampedAngle = angle
            if clampedAngle > self.maxPhalanxMotorPosition[i]:
                clampedAngle = self.maxPhalanxMotorPosition[i]
            elif clampedAngle < self.minPhalanxMotorPosition[i]:
                clampedAngle = self.minPhalanxMotorPosition[i]

            if len(self.rphalanx) > i and self.rphalanx[i] is not None:
                self.rphalanx[i].setPosition(clampedAngle)
            if len(self.lphalanx) > i and self.lphalanx[i] is not None:
                self.lphalanx[i].setPosition(clampedAngle)

    def setRShoulderPitch(self):
        angle = float(input("Enter angle"))
        self.RShoulderPitch.setPosition(angle)

    """
    Set rotational motor position 
    inputs: motor name, angle 
    """
    def setMotorPosition(self):
        angle = float(input("Input the angle:   \n"))
        motorName = input("Input motor name:   \n")

        if motorName =="HeadPitch":
            self.HeadPitch.setPosition(angle)
        elif motorName == "HeadYaw":
            self.HeadYaw.setPosition(angle)
        elif motorName == "RElbowRoll":
            self.RElbowRoll.setPosition(angle)
        elif motorName == "RElbowYaw":
            self.RElbowYaw.setPosition(angle)
        elif motorName == "RWristYaw":
            self.RWristYaw.setPosition(angle)
        elif motorName == "RShoulderPitch":
            self.RShoulderPitch.setPosition(angle)
        elif motorName == "RShoulderRoll":
            self.RShoulderRoll.setPosition(angle)
        elif motorName == "RAnklePitch":
            self.RAnklePitch.setPosition(angle)
        elif motorName == "RAnkleRoll":
            self.RAnkleRoll.setPosition(angle)
        elif motorName == "RHipPitch":
            self.RHipPitch.setPosition(angle)
        elif motorName == "RHipRoll":
            self.RHipRoll.setPosition(angle)
        elif motorName == "RHipYawPitch":
            self.RHipYawPitch.setPosition(angle)
        elif motorName == "RKneePitch":
            self.RKneePitch.setPosition(angle)
        elif motorName == "LElbowRoll":
            self.LElbowRoll.setPosition(angle)
        elif motorName == "LElbowYaw":
            self.LElbowYaw.setPosition(angle)
        elif motorName == "LWristYaw":
            self.LWristYaw.setPosition(angle)
        elif motorName == "LShoulderPitch":
            self.LShoulderPitch.setPosition(angle)
        elif motorName == "LShoulderRoll":
            self.LShoulderRoll.setPosition(angle)
        elif motorName == "LAnklePitch":
            self.LAnklePitch.setPosition(angle)
        elif motorName == "LAnkleRoll":
            self.LAnkleRoll.setPosition(angle)
        elif motorName == "LHipPitch":
            self.LHipPitch.setPosition(angle)
        elif motorName == "LHipRoll":
            self.LHipRoll.setPosition(angle)
        elif motorName == "LHipYawPitch":
            self.LHipYawPitch.setPosition(angle)
        elif motorName == "LKneePitch":
            self.LKneePitch.setPosition(angle)
        else:
            print("No motor with that name\n")
            print("Stopping")

    def setMotorPositions(self):
        while True:
            self.setMotorPosition()
            answer = input("continue?: y or n")
            if answer == "n":
                break




    """
    Get position of motor
    inputs: motor name 
    """

    def getMotorPosition(self):
        motorName = input("Input Motor Name: \n")

        if motorName == "HeadPitch":
            print("Head Pitch: %f\n" % self.HeadPitchS.getValue())
        elif motorName == "HeadYaw":
            print("Head Yaw: %f\n" % self.HeadYawS.getValue())
        elif motorName == "RElbowRoll":
            print("RElbowRoll: %f\n" % self.RElbowRollS.getValue())
        elif motorName == "RElbowYaw":
            print("RElbowYaw: %f\n" % self.RElbowYawS.getValue())
        elif motorName == "RWristYaw":
            print("RWristYaw: %f\n" % self.RWristYawS.getValue())
        elif motorName == "RShoulderPitch":
            print("RShoulderPitch: %f\n" % self.RShoulderPitchS.getValue())
        elif motorName == "RShoulderRoll":
            print("RShoulderPitch: %f\n" % self.RShoulderRollS.getValue())
        elif motorName == "RAnklePitch":
            print("RAnklePitch: %f\n" % self.RAnklePitchS.getValue())
        elif motorName == "RAnkleRoll":
            print("RAnkleRoll: %f\n" % self.RAnkleRollS.getValue())
        elif motorName == "RHipPitch":
            print("RHipPitch: %f\n" % self.RHipPitchS.getValue())
        elif motorName == "RHipRoll":
            print("RHipRoll: %f\n" % self.RHipRollS.getValue())
        elif motorName == "RHipYawPitch":
            print("RHipYawPitch: %f\n" % self.RHipYawPitchS.getValue())
        elif motorName == "RKneePitch":
            print("RKneePitch: %f\n" % self.RKneePitchS.getValue())
        elif motorName == "LElbowRoll":
            print("LElbowRoll: %f\n" % self.LElbowRollS.getValue())
        elif motorName == "LElbowYaw":
            print("LElbowYaw: %f\n" % self.LElbowYawS.getValue())
        elif motorName == "LWristYaw":
            print("LWristYaw: %f\n" % self.LWristYawS.getValue())
        elif motorName == "LShoulderPitch":
            print("LShoulderPitch: %f\n" % self.LShoulderPitchS.getValue())
        elif motorName == "LShoulderRoll":
            print("LShoulderRoll: %f\n" % self.LShoulderRollS.getValue())
        elif motorName == "LAnklePitch":
            print("LAnklePitch: %f\n" % self.LAnklePitchS.getValue())
        elif motorName == "LHipPitch":
            print("LHipPitch: %f\n" % self.LHipPitchS.getValue())
        elif motorName == "LHipRoll":
            print("LHipRoll: %f\n" % self.LHipRollS.getValue())
        elif motorName == "LHipYawPitch":
            print("LHipYawPitch: %f\n" % self.LHipYawPitchS.getValue())
        elif motorName == "LKneePitch":
            print("LKneePitch: %f\n" % self.LKneePitchS.getValue())
        else:
            print("No motor with that name")

    def printAll(self):
        print('----------Current Motor Positions----------')
        print("HeadPitch: %f\n" % self.HeadPitchS.getValue())
        print("HeadYaw: %f\n" % self.HeadYawS.getValue())
        print("RElbowRoll: %f\n" % self.RElbowRollS.getValue())
        print("RElbowYaw: %f\n" % self.RElbowYawS.getValue())
        print("RShoulderPitch: %f\n" % self.RShoulderPitchS.getValue())
        print("RShoulderRoll: %f\n" % self.RShoulderRollS.getValue())
        print("RAnklePitch: %f\n" % self.RAnklePitchS.getValue())
        print("RAnkleRoll: %f\n" % self.RAnkleRollS.getValue())
        print("RHipPitch: %f\n" % self.RHipPitchS.getValue())
        print("RHipRoll: %f\n" % self.RHipRollS.getValue())
        print("RHipYawPitch: %f\n" % self.RHipYawPitchS.getValue())
        print("RKneePitch: %f\n" % self.RKneePitchS.getValue())
        print("LElbowRoll: %f\n" % self.LElbowRollS.getValue())
        print("LElbowYaw: %f\n" % self.LElbowYawS.getValue())
        print("LShoulderPitch: %f\n" % self.LShoulderPitchS.getValue())
        print("LShoulderRoll: %f\n" % self.LShoulderRollS.getValue())
        print("LAnklePitch: %f\n" % self.LAnklePitchS.getValue())
        print("LAnkleRoll: %f\n" % self.LAnkleRollS.getValue())
        print("LHipPitch: %f\n" % self.LHipPitchS.getValue())
        print("LHipRoll: %f\n" % self.LHipRollS.getValue())
        print("LHipYawPitch: %f\n" % self.LHipYawPitchS.getValue())
        print("LKneePitch: %f\n" % self.LKneePitchS.getValue())

    """
    Set the position of all joints to a legal random value
    """
    def randomPositions(self):
        self.HeadPitch.setPosition(uniform(-0.671952,0.514872))
        self.HeadYaw.setPosition(uniform(-2.08567,2.08567))
        self.RElbowRoll.setPosition(uniform(-1.54462,1.54462))
        self.RElbowYaw.setPosition(uniform(-2.08567,2.08567))
        self.RShoulderPitch.setPosition(uniform(-2.08567,2.08567))
        self.RShoulderRoll.setPosition(uniform(-1.32645,0.314159))
        """
        self.RAnklePitch.setPosition(uniform(-1.1863,0.932006))
        self.RAnkleRoll.setPosition(uniform(-0.768992,0.397935))
        self.RHipPitch.setPosition(uniform(-0.379435,0.79046))
        self.RHipRoll.setPosition(uniform(-0.379435,0.79046))
        self.RHipYawPitch.setPosition(uniform(-1.14529,0.740718))
        self.RKneePitch.setPosition(uniform(-0.0923279,2.11255))
        """
        self.LElbowRoll.setPosition(uniform(-1.54462,1.54462))
        self.LElbowYaw.setPosition(uniform(-2.08567,2.08567))
        self.LShoulderPitch.setPosition(uniform(-0.314159,1.32645))
        self.LShoulderRoll.setPosition(uniform(-1.32645,0.314159))
        """
        self.LAnklePitch.setPosition(uniform(-1.1863,0.932006))
        self.LAnkleRoll.setPosition(uniform(-0.768992,0.397935))
        self.LHipPitch.setPosition(uniform(-0.379435,0.79046))
        self.LHipRoll.setPosition(uniform(-0.379435,0.79046))
        self.LHipYawPitch.setPosition(uniform(-1.14529,0.740718))
        self.LKneePitch.setPosition(uniform(-0.0923279,2.11255))
        """

    """
    Input: Array of motor Positions
    Output: None 
    Change in State to the robots positions 
    """
    def setAll(self, posValues):
        self.HeadPitch.setPosition(posValues[0])
        self.HeadYaw.setPosition(posValues[1])
        self.RElbowRoll.setPosition(posValues[2])
        self.RElbowYaw.setPosition(posValues[3])
        self.RShoulderPitch.setPosition(posValues[4])
        self.RShoulderRoll.setPosition(posValues[5])
        self.RAnklePitch.setPosition(posValues[6])
        self.RAnkleRoll.setPosition(posValues[7])
        self.RKneePitch.setPosition(posValues[8])
        self.RHipPitch.setPosition(posValues[9])
        self.RHipRoll.setPosition(posValues[10])
        self.RHipYawPitch.setPosition(posValues[11])
        self.LElbowRoll.setPosition(posValues[12])
        self.LElbowYaw.setPosition(posValues[13])
        self.LShoulderPitch.setPosition(posValues[14])
        self.LShoulderRoll.setPosition(posValues[15])
        self.LAnklePitch.setPosition(posValues[16])
        self.LAnkleRoll.setPosition(posValues[17])
        self.LKneePitch.setPosition(posValues[18])
        self.LHipPitch.setPosition(posValues[19])
        self.LHipRoll.setPosition(posValues[20])
        self.LHipYawPitch.setPosition(posValues[21])
        # set color
        self.setAllLedsColor(posValues[22])

    def poseSelector(self):
        print("select pose")
        print("q: NeutralHandsDown")
        print("w: NeutraLHandsUp")
        print("e: HandsUpHeadAway")
        print("r: KneeBentHeadUp")
        print("t: SemiDab")
        print("y: ArmsAwaySideView")
        print("a: ArmsFlex")
        print("s: HandBehind")
        print("d: LookAtHand")
        print("f: SpeechMode")
        print("g: Attention")

        select = input()

        if select == 'q':
            self.setAll(self.poses.get('NeutralHandsDown'))
        elif select == 'w':
            self.setAll(self.poses.get('NeutralHandsUp'))
        elif select == 'e':
            self.setAll(self.poses.get('Other'))
        elif select == 'r':
            self.setAll(self.poses.get('KneeBentHeadUp'))
        elif select == 't':
            self.setAll(self.poses.get('SemiDab'))
        elif select == 'y':
            self.setAll(self.poses.get('ArmsAwaySideView'))
        elif select == 'a':
            self.setAll(self.poses.get('ArmsFlex'))
        elif select == 's':
            self.setAll(self.poses.get('HandBehind'))
        elif select == 'd':
            self.setAll(self.poses.get("LookAtHand"))
        elif select == 'f':
            self.setAll(self.poses.get("SpeechMode"))
        elif select == 'g':
            self.setAll(self.poses.get("Attention"))
        else:
            print("exiting selector")

    def printHelp(self):
        print('----------nao_demo_python----------')
        print('Use the keyboard to control the robots (one at a time)')
        print('(The 3D window need to be focused)')
        print('[Up][Down]: move one step forward/backwards')
        print('[<-][->]: side step left/right')
        print('[Shift] + [<-][->]: turn left/right')
        print('[U]: print ultrasound sensors')
        print('[A]: print accelerometers')
        print('[G]: print gyros')
        print('[S]: print gps')
        print('[I]: print inertial unit (roll/pitch/yaw)')
        print('[F]: print foot sensors')
        print('[B]: print foot bumpers')
        print('[Home][End]: print scaled top/bottom camera image')
        print('[PageUp][PageDown]: open/close hands')
        print('[7][8][9]: change all leds RGB color')
        print('[0]: turn all leds off')
        print('[H]: print this help message')

    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())

        # camera
        self.cameraTop = self.getCamera("CameraTop")
        self.cameraBottom = self.getCamera("CameraBottom")
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom.enable(4 * self.timeStep)

        # accelerometer
        self.accelerometer = self.getAccelerometer('accelerometer')
        self.accelerometer.enable(4 * self.timeStep)

        # gyro
        self.gyro = self.getGyro('gyro')
        self.gyro.enable(4 * self.timeStep)

        # gps
        self.gps = self.getGPS('gps')
        self.gps.enable(4 * self.timeStep)

        # inertial unit
        self.inertialUnit = self.getInertialUnit('inertial unit')
        self.inertialUnit.enable(self.timeStep)

        # ultrasound sensors
        self.us = []
        usNames = ['Sonar/Left', 'Sonar/Right']
        for i in range(0, len(usNames)):
            self.us.append(self.getDistanceSensor(usNames[i]))
            self.us[i].enable(self.timeStep)

        # foot sensors
        self.fsr = []
        fsrNames = ['LFsr', 'RFsr']
        for i in range(0, len(fsrNames)):
            self.fsr.append(self.getTouchSensor(fsrNames[i]))
            self.fsr[i].enable(self.timeStep)

        # foot bumpers
        self.lfootlbumper = self.getTouchSensor('LFoot/Bumper/Left')
        self.lfootrbumper = self.getTouchSensor('LFoot/Bumper/Right')
        self.rfootlbumper = self.getTouchSensor('RFoot/Bumper/Left')
        self.rfootrbumper = self.getTouchSensor('RFoot/Bumper/Right')
        self.lfootlbumper.enable(self.timeStep)
        self.lfootrbumper.enable(self.timeStep)
        self.rfootlbumper.enable(self.timeStep)
        self.rfootrbumper.enable(self.timeStep)

        # there are 7 controlable LED groups in Webots
        self.leds = []
        self.leds.append(self.getLED('ChestBoard/Led'))
        self.leds.append(self.getLED('RFoot/Led'))
        self.leds.append(self.getLED('LFoot/Led'))
        self.leds.append(self.getLED('Face/Led/Right'))
        self.leds.append(self.getLED('Face/Led/Left'))
        self.leds.append(self.getLED('Ears/Led/Right'))
        self.leds.append(self.getLED('Ears/Led/Left'))

        # get phalanx motor tags
        # the real Nao has only 2 motors for RHand/LHand
        # but in Webots we must implement RHand/LHand with 2x8 motors
        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        for i in range(0, self.PHALANX_MAX):
            self.lphalanx.append(self.getMotor("LPhalanx%d" % (i + 1)))
            self.rphalanx.append(self.getMotor("RPhalanx%d" % (i + 1)))

            # assume right and left hands have the same motor position bounds
            self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
            self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())

        #Rotational Motors
        self.HeadPitch = self.getMotor("HeadPitch")
        self.HeadYaw = self.getMotor("HeadYaw")

        self.RElbowRoll = self.getMotor("RElbowRoll")
        self.RElbowYaw = self.getMotor("RElbowYaw")
        self.RShoulderPitch = self.getMotor("RShoulderPitch")
        self.RShoulderRoll = self.getMotor("RShoulderRoll")

        self.RAnklePitch = self.getMotor("RAnklePitch")
        self.RAnkleRoll = self.getMotor("RAnkleRoll")
        self.RHipPitch = self.getMotor("RHipPitch")
        self.RHipRoll = self.getMotor("RHipRoll")
        self.RHipYawPitch = self.getMotor("RHipYawPitch")
        self.RKneePitch = self.getMotor("RKneePitch")


        self.LElbowRoll = self.getMotor("LElbowRoll")
        self.LElbowYaw = self.getMotor("LElbowYaw")
        self.LShoulderPitch = self.getMotor("LShoulderPitch")
        self.LShoulderRoll = self.getMotor("LShoulderRoll")

        self.LAnklePitch = self.getMotor("LAnklePitch")
        self.LAnkleRoll = self.getMotor("LAnkleRoll")
        self.LHipPitch = self.getMotor("LHipPitch")
        self.LHipRoll = self.getMotor("LHipRoll")
        self.LHipYawPitch = self.getMotor("LHipYawPitch")
        self.LKneePitch = self.getMotor("LKneePitch")

        # Enable Position sensors.
        self.RShoulderPitchS = self.getPositionSensor("RShoulderPitchS")
        self.RShoulderPitchS.enable(1000)
        self.LShoulderPitchS = self.getPositionSensor("LShoulderPitchS")
        self.LShoulderPitchS.enable(1000)

        self.RShoulderRollS = self.getPositionSensor("RShoulderPitchS")
        self.RShoulderPitchS.enable(1000)
        self.LShoulderRollS = self.getPositionSensor("LShoulderPitchS")
        self.LShoulderPitchS.enable(1000)

        self.HeadPitchS = self.getPositionSensor("HeadPitchS")
        self.HeadPitchS.enable(1000)
        self.HeadYawS = self.getPositionSensor("HeadYawS")
        self.HeadYawS.enable(1000)

        self.RElbowRollS = self.getPositionSensor("RElbowRollS")
        self.RElbowRollS.enable(1000)
        self.RElbowYawS = self.getPositionSensor("RElbowYawS")
        self.RElbowYawS.enable(1000)
        self.RElbowYawS = self.getPositionSensor("RElbowYawS")
        self.RElbowYawS.enable(1000)

        self.RAnklePitchS = self.getPositionSensor("RAnklePitchS")
        self.RAnklePitchS.enable(1000)
        self.RAnkleRollS = self.getPositionSensor("RAnkleRollS")
        self.RAnkleRollS.enable(1000)
        self.RHipPitchS = self.getPositionSensor("RHipPitchS")
        self.RHipPitchS.enable(1000)
        self.RHipRollS = self.getPositionSensor("RHipRollS")
        self.RHipRollS.enable(1000)

        self.RHipYawPitchS = self.getPositionSensor("RHipYawPitchS")
        self.RHipYawPitchS.enable(1000)

        self.RKneePitchS = self.getPositionSensor("RKneePitchS")
        self.RKneePitchS.enable(1000)

        self.LElbowRollS = self.getPositionSensor("LElbowRollS")
        self.LElbowRollS.enable(1000)
        self.LElbowRollS = self.getPositionSensor("LElbowRollS")
        self.LElbowRollS.enable(1000)
        self.LElbowYawS = self.getPositionSensor("LElbowYawS")
        self.LElbowYawS.enable(1000)

        self.LAnklePitchS = self.getPositionSensor("LAnklePitchS")
        self.LAnklePitchS.enable(1000)
        self.LAnkleRollS = self.getPositionSensor("LAnkleRollS")
        self.LAnkleRollS.enable(1000)
        self.LHipPitchS = self.getPositionSensor("LHipPitchS")
        self.LHipPitchS.enable(1000)
        self.LHipRollS = self.getPositionSensor("LHipRollS")
        self.LHipRollS.enable(1000)
        self.LHipYawPitchS = self.getPositionSensor("LHipYawPitchS")
        self.LHipYawPitchS.enable(1000)
        self.LKneePitchS = self.getPositionSensor("LKneePitchS")
        self.LKneePitchS.enable(1000)


        # keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)

    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False

        # initialize stuff
        self.findAndEnableDevices()
        self.loadMotionFiles()
        self.printHelp()

    def run(self):
        self.handWave.setLoop(True)
        self.handWave.play()

        # until a key is pressed
        key = -1
        while robot.step(self.timeStep) != -1:
            key = self.keyboard.getKey()
            if key > 0:
                break

        self.handWave.setLoop(False)

        while True:
            key = self.keyboard.getKey()

            if key == Keyboard.LEFT:
                self.startMotion(self.sideStepLeft)
            elif key == Keyboard.RIGHT:
                self.startMotion(self.sideStepRight)
            elif key == Keyboard.UP:
                self.startMotion(self.forwards)
            elif key == Keyboard.DOWN:
                self.startMotion(self.backwards)
            elif key == Keyboard.LEFT | Keyboard.SHIFT:
                self.startMotion(self.turnLeft60)
            elif key == Keyboard.RIGHT | Keyboard.SHIFT:
                self.startMotion(self.turnRight60)
            elif key == Keyboard.PAGEUP:
                self.setHandsAngle(0.96)
            elif key == Keyboard.PAGEDOWN:
                self.setHandsAngle(0.0)
            elif key == ord('1'):
                self.getMotorPosition()
            elif key == ord('2'):
                self.setMotorPositions()
            elif key == ord('3'):
                #while robot.step(32) !=1:
                self.randomPositions()
                    #time.sleep(.500)
            elif key == ord('4'):
                self.poseSelector()
            elif key == ord('5'):
                self.printAll()
            elif key == ord('7'):
                self.setAllLedsColor(0xff0000)  # red
            elif key == ord('8'):
                self.setAllLedsColor(0x00ff00)  # green
            elif key == ord('9'):
                self.setAllLedsColor(0x0000ff)  # blue
            elif key == ord('0'):
                self.setAllLedsColor(0xffff00)  # off
            elif key == ord('H'):
                self.printHelp()
            elif key == Keyboard.CONTROL+ord('B'):
                print("Ctrl+B is pressed")
            """
            elif key == ord('Z'):
                self.getRightShoulderValue()
            elif key == ord('Q'):
                self.setRShoulderPitch()
            elif key == ord('A'):
                self.printAcceleration()
            elif key == ord('G'):
                self.printGyro()
            elif key == ord('S'):
                self.printGps()
            elif key == ord('I'):
                self.printInertialUnit()
            elif key == ord('F'):
                self.printFootSensors()
            elif key == ord('B'):
                self.printFootBumpers()
            elif key == ord('U'):
                self.printUltrasoundSensors()
            elif key == Keyboard.HOME:
                self.printCameraImage(self.cameraTop)
            elif key == Keyboard.END:
                self.printCameraImage(self.cameraBottom)
            """


            if robot.step(self.timeStep) == -1:
                break


# create the Robot instance and run main loop
robot = Nao()
robot.run()
