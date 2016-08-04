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
        self.cubecount = 0

    


class Wheels(object):
    def __init__(self,left,right,robot):
        self.R = robot
        self.left = left
        self.right = right

    def _forward(self,secs,power):
        self.R.motors[self.left].target = power * 1.13
        self.R.motors[self.right].target = power
        sleep(secs)
        self.R.motors[self.left].target = 0
        self.R.motors[self.right].target = 0

    def _clockwise(self,power,secs,both = True):
        self.R.motors[self.left].target = power
        if both:
            self.R.motors[self.left].target = -power
        sleep(secs)
        self.R.motors[self.left].target = 0
        if both:
            self.R.motors[self.left].target = 0

    def _anticlockwise(self,power,secs,both = True):
        self.clockwise(-power,secs,both)

    def stop(self):
        self.R.motors[self.left].target = 0
        self.R.motors[self.right].target = 0


class Motion(Wheels):

    driveConstant = 1.8
    drivePower = 25
    turnConstant = 0.145

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

    def move_till_beam_hit(self,extra,max):
        raise Exception('Not Implemented: Move till beam hit')
        return bool

    def go_to_score_zone(self):
        raise Exception('Not Implemented yet: Go to zone')



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

    def get_tokens(self):
        log("Fetching last seen tokens")
        tokens = []
        for marker in self.markers:
            if marker.info.marker_type == MARKER_TOKEN:
                tokens.append(marker)

        return tokens

    def closest_token(self):
        tokens = self.get_tokens()
        if len(tokens) > 0:
            closest = tokens[0]
            for token in tokens:
                if closest.dist > token.dist:
                    closest = token

            return closest

    def get_token_by_offset(self,number):
        raise Exception('Not Implemented: Get Token by offset')

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

    def check_angle(self):
        log("Getting angle")
        self.update()
        wall_markers = self.get_wall_markers()

        marker = wall_marker[0]
        if marker.info.offset in range(8):
            angle = marker.orientation.rot_z + 180
        elif marker.info.offset in range(8,15):
            angle = marker.orientation.rot_z + 270
        elif marker.info.offset in range(15,22):
            angle = marker.orientation.rot_z + 360
        elif marker.info.offset in range(22,28):
            angle = marker.orientation.rot_z + 450
        angle -= marker.rot_y
        if angle > 360:
            angle -= 360
        return angle
        

class BaseIO(object):

    def __init__(self,pin,robot):
        self.pin = pin
        self.R = robot

    def input(self):
        return self.R.io[0].input[self.pin].d

    def analogue(self):
        return self.R.io[0].input[self.pin].a

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
        #print "WARNING! Bigpeice.Control.check_sensors is disabled!"
        #self.check_sensors()
        self.time_check()


    def check_sensors(self):

        raise Exception('Not Implemented: Check Sensors')

    def time_check(self):
        if self.time_left() <= 0:
            self.victory()

    def victory(self):
        print "FINISH"
        exit()

def log(message):
        if verbose:
            print "     " + message
            
            
def sleep(secs):
    C.sleep(secs)

verbose = False

print "The Bigpeice"
print "Designed & Built by Bigpeice"
print "Code Copyright Bigpeice 2016. All rights reserved. Do not reuse without explicit permission"

print "Initialising..."
R = CRobot(0, 1, 2, 0, 1, 0, 0)
C = Control(R)
print "Zone: " + str(R.R.zone)
#print "Mode: " + str(R.R.mode) #Doesn't work on simulator
print "Started"

R.arm.open()
sleep(3)
R.arm.close()
sleep(1)
R.arm.open()

R.motion.clockwise(45)
R.motion.forward(2)

R.eyes.update()
print "WALL MARKERS"
markers = R.eyes.get_wall_markers()
print len(markers)
for marker in markers:
    print str(marker.dist) + " @ " + str(marker.rot_y)

print "TOKENS"
markers  = R.eyes.get_tokens()
print len(markers)
for marker in markers:
    print str(marker.dist) + " @ " + str(marker.rot_y)

prevValue = 0
count = 0
while True:
    value = R.lightbeam.analogue()
    if value == prevValue:
        count += 1
    else:
        prevValue = value
        count = 0

    if count >= 100:
        print "100 consecutive values found"
        sleep(5)



    sleep(0.01)


# Actual Code, not yet tested. Do not execute! Turn on check_sensors first!
exit() # Prevent execution

verbose = TRue


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
R.motion.clockwise(45)
R.motion.forward(2)  #Get initial tokens
R.arm.close()
R.cubecount += 4
R.motion.anticlockwise(90)

sleep(0.5) # Wait to reduce motion blur

R.eyes.update() # Get new visionary data

while R.cubecount <= 6 or C.time_left() > 30:
    target = R.eyes.closest_token()
    log("Targeting: " + str(target.info.offset))
    R.motion.clockwise(target.rot_y)
    R.motion.forward(target.dist/2)
    sleep(0.5)
    R.eyes.update()
    target = R.eyes.get_token_by_offset(target.info.offset)
    if target == None:
        raise Exception('No target found, not implemented')
    else:
        #Target found, adjust
        R.motion.clockwise(target.rot_y)
        R.arm.open()
        hit = R.motion.move_till_beam_hit(0.4,target.dist * 2)
        if not hit:
            raise Exception('Not Implemented: Cube not caught')
        R.cubecount += 1
        R.arm.close()

R.arm.close()
R.motion.go_to_score_zone()
R.motion.reverse(2)
R.arm.close()

#Scan for poison
print "FUCK WE FORGOT HOW TO HUNT THE POISON!"

