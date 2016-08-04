from sr import *
import time, math, random

class CRobot(object):

    def __init__(self, leftWheel, rightWheel, beamPin, leftPin, rightPin, armBoard, armNumber):
        self.R = Robot()
        self.motion = Motion(leftWheel,rightWheel,self.R)
        self.eyes = Vision(self.R)
        self.lightbeam = Digital(beamPin,self.R)
        self.leftSensor = Digital(leftPin,self.R)
        self.rightSensor = Digital(rightPin,self.R)
        self.arm = Arm(armBoard,armNumber,self.R)
        self.c = Control(self.R)

    


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
    drivePower = 25
    turnConstant = 0.015

    def forward(self,metres):
        log("Forward " + str(metres) + " metres")
        self._forward(metres * self.driveConstant,self.drivePower)

    def reverse(self,metres):
        log("Reverse " + str(metres) + " metres")
        self._forward(metres * self.driveConstant, -1 * self.drivePower)

    def clockwise(self,degrees):
        log("Clockwise " + str(degrees) + " degrees")
        self._clockwise(self.drivePower,degrees * self.turnConstant)

    def anticlockwise(self,degrees):
        log("Anti-clockwise " + str(degrees) + " degrees")
        self._anticlockwise(self.drivePower,degrees * self.turnConstant)


class Vision(object):

    def __init__(self,robot):
        self.R = robot
        self.update()

    def update(self):
        log("Checking Vision")
        self.markers = self.R.see()

    def get_wall_markers(self):
        log("Fetching last seen wall markers")
        wall_markers = []
        for marker in self.markers:
            if marker.info.marker_type == MARKER_ARENA:
                wall_markers.append(marker)

        return wall_markers


    def check_location(self):
        log("Getting location")
        self.update()
        wall_markers = self.get_wall_markers()
        marker_positions = [[0,1],[0,2],[0,3],[0,4],[0,5],[0,6],[0,7],\
                            [1,0],[2,0],[3,0],[4,0],[5,0],[6,0],[7,0],\
                            [8,1],[8,2],[8,3],[8,4],[8,5],[8,6],[8,7],
                            [1,8],[2,8],[3,8],[4,8],[5,8],[6,8],[7,8],]
        total_cartesian = [0,0]
        for marker in wall_markers:
            horizontal_distance = math.cos(math.radians(marker.centre.polar.rot_x))* marker.dist
            if marker.info.offset in range(8):
                angle = marker.orientation.rot_z + 0
            elif marker.info.offset in range(8,15):
                angle = marker.orientation.rot_z + 90
            elif marker.info.offset in range(15,22):
                angle = marker.orientation.rot_z + 180
            elif marker.info.offset in range(22,28):
                angle = marker.orientation.rot_z + 270

            total_cartesian[0] += marker_positions[marker.info.offset][0]+\
                                  (math.sin(math.radians(angle))*horizontal_distance)
            total_cartesian[1] += marker_positions[marker.info.offset][1]+\
                                  (math.cos(math.radians(angle))*horizontal_distance)
        number_of_visible_wall_markers = len(wall_markers)

        return total_cartesian[0]/number_of_visible_wall_markers, total_cartesian[1]/number_of_visible_wall_markers


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
        log("Output On")
        self.R.io[0].output[self.pin].d = 1

    def off(self):
        log("Output Off")
        self.R.io[0].output[self.pin].d = 0


class Servo(object):

    def __init__(self, board, number, R):
        self.board = board
        self.number = number
        self.R = R

    def setpos(self,position):
        log("Setting Servo " + str(self.number) + " to " + str(position))
        self.R.servos[self.board][self.number] = position


class Arm(Servo):

    def open(self):
        log("Opening Arm")
        self.setpos(50)  # Needs changing

    def close(self):
        log("Closing Arm")
        self.setpos(90)  # Needs changing


class Control(object):
    """ Controls the timing of the loop and the loop. Also does events """

    total_time = 180

    def __init__(self,robot):
        log("Control Initiated")
        self.R = robot
        self.start_time = time.time()
        self.end_time = time.time() + self.total_time

    def sleep(self,secs):
        log("Wait: " + str(secs) + " secs")
        start = time.time()
        end = start + secs
        while end > time.time():
            self.loop()

    def time_left(self):
        return self.end_time - time.time()

    def loop(self):
        self.check_sensors()
        self.time_check()


    def check_sensors(self):
        pass

    def time_check(self):
        if self.time_left() <= 0:
            self.victory()

    def victory(self):
        print "FINISH"

def log(message):
        if verbose:
            print "     " + message


verbose = True

print "The Bigpeice"
print "Designed & Built by Bigpeice"
print "Code Copyright Bigpeice 2016. All rights reserved. Do not reuse without explicit permission"

print "Initialising..."
R = CRobot(0, 1, 10, 10, 10, 0, 0)
print "Zone: " + str(R.R.zone)
#print "Mode: " + str(R.R.mode) #Doesn't work on simulator
print "Started"

while True:
    R.arm.close()
    R.c.sleep(random.randint(1,10))
    R.arm.open()
    R.c.sleep(random.randint(1,10))


# Actual Code, not yet tested. Do not execute!
exit() # Prevent execution

verbose = True


print "The Bigpeice"
print "Designed & Built by Bigpeice"
#print "Code Copyright Bigpeice 2016. All rights reserved. Do not reuse without explicit permission"

print "Initialising..."
R = CRobot(0, 1, 10, 10, 10, 0, 0)
print "Zone: " + str(R.R.zone)
#print "Mode: " + str(R.R.mode) #Doesn't work on simulator
log("Starting Algorithm")
#Start Algorithm

R.arm.open()
R.motion.clockwise(90)
R.motion.forward(2)  #Get initial tokens
R.arm.close()
R.motion.anticlockwise(90)

R.c.sleep(0.5) # Wait to reduce motion blur

R.eyes.update() # Get new visionary data

