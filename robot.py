from sr import *
import time



class CRobot(object):

    def __init__(self,leftWheel,rightWheel,beamPin,leftPin,rightPin,armBoard,armNumber):
        self.R = Robot()
        self.motion = Motion(leftWheel,rightWheel,self.R)
        self.eyes = Vision(self.R)
        self.lightbeam = Digital(beamPin,self.R)
        self.leftSensor = Digital(leftPin,self.R)
        self.rightSensor = Digital(rightPin,self.R)
        self.arm = Arm(armBoard,armNumber,self.R)


class Wheels(object):
    def __init__(self,left,right,robot):
        self.R = robot
        self.left = left
        self.right = right

    def _forward(self,secs,power):
        self.R.motors[self.left].target = power
        self.R.motors[self.right].target = power
        time.sleep(secs)
        self.R.motors[self.left].target = 0
        self.R.motors[self.right].target = 0

    def _clockwise(self,power,secs,both = True):
        self.R.motors[self.left].target = power
        if both:
            self.R.motors[self.left].target = -power
        time.sleep(secs)
        self.R.motors[self.left].target = 0
        if both:
            self.R.motors[self.left].target = 0

    def _anticlockwise(self,power,secs,both = True):
        self.clockwise(-power,secs,both)


class Motion(Wheels):

    driveConstant = 0.6
    turnConstant = 0.0075

    def forward(self,metres):
        self._forward(metres * self.driveConstant,100)

    def reverse(self,metres):
        self._forward(metres * self.driveConstant, -100)

    def clockwise(self,degrees):
        self._clockwise(100,degrees * self.turnConstant)

    def anticlockwise(self,degrees):
        self._anticlockwise(100,degrees * self.turnConstant)


class Vision(object):

    def __init__(self,robot):
        self.R = robot
        self.update()

    def update(self):
        self.markers = self.R.see()

    def get_wall_markers(self):
        

    def count(self):
        return len(self.markers)


class BaseIO(object):

    def __init__(self,pin,robot,input = False):
        self.pin = pin
        self.R = robot
        self.input = input

    def input(self):
        if type(self) is "Analogue":
            return self.R.io[0].input[self.pin].a
        else:
            return self.R.io[0].input[self.pin].d

Analogue = BaseIO


class Digital(BaseIO):
    def on(self):
        self.R.io[0].output[self.pin].d = 1

    def off(self):
        self.R.io[0].output[self.pin].d = 0


class Servo(object):

    def __init__(self,board,number,R):
        self.board = board
        self.number = number
        self.R = R

    def setpos(self,position):
        self.R.servos[self.board][self.number] = position


class Arm(Servo):

    def open(self):
        self.setPosition(0)  # Needs changing

    def close(self):
        self.setPosition(100)  # Needs changing


R = CRobot(1, 0, 0, 1, 2, 3, 4)

while True:

    R.motion.forward(2)
    R.motion.clockwise(90)







