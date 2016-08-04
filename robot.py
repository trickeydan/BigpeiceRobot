from sr import *
import time, math, random

class CRobot(object):

    def __init__(self, leftWheel, rightWheel, beamPin, leftPin, rightPin, armBoard, armNumber):
        self.R = Robot()
        self.motion = Navigation(leftWheel,rightWheel,self.R)
        self.eyes = Vision(self.R)
        #self.lightbeam = Digital(beamPin,self.R)
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
        self.R.motors[self.left].target = power * 1.5
        self.R.motors[self.right].target = power
        sleep(secs)
        self.R.motors[self.left].target = 0
        self.R.motors[self.right].target = 0

    def _clockwise(self,power,secs):
        self.R.motors[self.left].target = -power
        self.R.motors[self.right].target = power
        sleep(secs)
        self.R.motors[self.left].target = 0
        self.R.motors[self.right].target = 0

    def _anticlockwise(self,power,secs):
        self._clockwise(-power,secs)

    def stop(self):
        self.R.motors[self.left].target = 0
        self.R.motors[self.right].target = 0


class Motion(Wheels):

    driveConstant = 1.8
    drivePower = 25
    turnConstant = 0.0070714285714286

    def forward(self,metres):
        log("Forward " + str(metres) + " metres")
        self._forward(metres * self.driveConstant,self.drivePower)

    def reverse(self,metres):
        log("Reverse " + str(metres) + " metres")
        self._forward(metres * self.driveConstant, -1 * self.drivePower)

    def clockwise(self,degrees):
        log("Clockwise " + str(degrees) + " degrees")
        self._clockwise(self.drivePower ,degrees * self.turnConstant)

    def anticlockwise(self,degrees):
        log("Anti-clockwise " + str(degrees) + " degrees")
        self._anticlockwise(self.drivePower,degrees * self.turnConstant)




class Vision(object):

    def __init__(self,robot):
        self.R = robot
        self.update()

    def update(self):
        log("Checking Vision")
        sleep(0.5) # Wait to reduce motion blur
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
        else:
            return None

    def get_token_by_offset(self,number):
        tokens = self.get_tokens()
        if len(tokens) > 0:
            target = None
            for token in tokens:
                if target.info.offset = number:
                    target = token

        return target

    def get_poison(self):
        for marker in self.markers:
            if marker.info.marker_type == MARKER_POISON: #CHECK ME!
                return marker



        
class Navigation(Motion):

    
    def getAngleToPoint(self,x_landmark, y_landmark):
        x_orig,y_orig = self.check_location()
        deltaY = y_landmark - y_orig
        deltaX = x_landmark - x_orig
        
        if deltaY == 0:
            if deltaX > 0:
                angle = 270
            elif deltaX < 0:
                angle = 90
            else:
                angle = 0

        elif deltaX == 0:
            if deltaY > 0:
                angle = 180
            else:
                angle = 0
        else:
            angle = math.degrees(math.atan(deltaX/deltaY))
            if deltaX > 0:
                angle = angle +270
            elif deltaX < 0:
                angle = angle +90


        return angle

    def check_location(self):
        log("Getting location")
        wall_markers = self.R.eyes.get_wall_markers()
        marker_positions = [[[0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6], [0, 7], \
                             [1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, 0], [7, 0], \
                             [8, 1], [8, 2], [8, 3], [8, 4], [8, 5], [8, 6], [8, 7], \
                             [1, 8], [2, 8], [3, 8], [4, 8], [5, 8], [6, 8], [7, 8]], \
    \
                            [[1, 8], [2, 8], [3, 8], [4, 8], [5, 8], [6, 8], [7, 8] \
                                [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6], [0, 7], \
                             [1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, 0], [7, 0], \
                             [8, 1], [8, 2], [8, 3], [8, 4], [8, 5], [8, 6], [8, 7]], \
    \
                            [[8, 1], [8, 2], [8, 3], [8, 4], [8, 5], [8, 6], [8, 7], \
                             [1, 8], [2, 8], [3, 8], [4, 8], [5, 8], [6, 8], [7, 8], \
                             [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6], [0, 7], \
                             [1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, 0], [7, 0]], \
    \
                            [[1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, 0], [7, 0], \
                             [8, 1], [8, 2], [8, 3], [8, 4], [8, 5], [8, 6], [8, 7], \
                             [1, 8], [2, 8], [3, 8], [4, 8], [5, 8], [6, 8], [7, 8], \
                             [0, 1], [0, 2], [0, 3], [0, 4], [0, 5], [0, 6], [0, 7]], \
    \
                            ]
        total_cartesian = [0, 0]

        for marker in wall_markers:
            horizontal_distance = math.cos(math.radians(marker.centre.polar.rot_x)) * marker.dist
            if marker.info.offset in range(7):
                angle = marker.orientation.rot_z + 0
            elif marker.info.offset in range(7, 14):
                angle = marker.orientation.rot_z + 90
            elif marker.info.offset in range(14, 21):
                angle = marker.orientation.rot_z + 180
            elif marker.info.offset in range(21, 28):
                angle = marker.orientation.rot_z + 270
            angle = angle + (90 * zone)

            total_cartesian[0] += marker_positions[marker.info.offset][self.R.zone][0] + \
                                  (math.sin(math.radians(angle)) * horizontal_distance)
            total_cartesian[1] += marker_positions[marker.info.offset][self.R.zone][1] + \
                                  (math.cos(math.radians(angle)) * horizontal_distance)
        number_of_visible_wall_markers = len(wall_markers)

        return total_cartesian[0] / number_of_visible_wall_markers, total_cartesian[
            1] / number_of_visible_wall_markers
    
    def check_angle(self):
        log("Getting angle")
        
        wall_markers = self.R.eyes.get_wall_markers()

        marker = wall_marker[0]
        if marker.info.offset in range(7):
            angle = marker.orientation.rot_z + 180
        elif marker.info.offset in range(7, 14):
            angle = marker.orientation.rot_z + 270
        elif marker.info.offset in range(16, 21):
            angle = marker.orientation.rot_z + 360
        elif marker.info.offset in range(21, 28):
            angle = marker.orientation.rot_z + 450
        angle -= marker.rot_y
        angle = angle + (90 * self.R.zone)

        while angle > 360:
            angle -= 360
        return angle

    def in_score_zone(self):
        x,y = self.check_location()
        return (y > 6-x) and (x<4) and (y < 4))

    def navigate_to_zone(self):
        self.R.eyes.update()
        x,y = self.check_location()
        angle = self.check_angle()

        if x > y and y > 8-x:#if the bot is almost directly opposite scoring zone, but more towards the anticlockwise
            target = [6,2]
            
        elif y >= x and y > 8-x:#same as above, but closer to clockwise
            target = [2,6]

        elif y <= 8-x and (y > 4 or x > 4):#if not in top-left quadrant or bottom-right half
            target = [2,2]

        elif y <= 4 and x <= 4:#if in top left, point to the centre
            target = [3,3]

            
        angle_to_turn = getAngleToPoint(target[0],target[1]) - angle
        distance_to_go = math.sqrt(((x-target[0])^2)+((y-target[1])^2))+0.1
        while angle_to_turn < 0:
            angle_to_turn + 360

        if angle_to_turn < 180:
            self.clockwise(angle_to_turn)
        else:
            self.anticlockwise(360-angle_to_turn)

        self.forward(distance_to_go)


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
        self.loop = True

    def sleep(self,secs):
        log("Wait: " + str(secs) + " secs")
        start = time.time()
        end = start + secs
        while end > time.time():
            if self.loop:
                self.loop()

    def time_left(self):
        return self.end_time - time.time()

    def loop(self):
        self.check_sensors()
        self.time_check()


    def check_sensors(self):
        if self.R.leftSensor.input() or self.R.rightSensor.input(): # Might not be returning bool, check
            log("Sensors Hit")
            self.R.motion.stop()
            self.loop = False
            sleep(0.2)
            self.R.motion.reverse(0.8)
            
            self.R.motion.clockwise(180)
            self.R.eyes.update()
            self.loop = True
            
            
        raise Exception('Not Implemented: Check Sensors')

    def time_check(self):
        if self.time_left() <= 0:
            self.victory()

    def victory(self):
        print "FINISH - Performing Victory Dance"
        self.R.motion.clockwise(5)
        time.sleep(0.1)
        self.R.motion.anticlockwise(5)
        exit()

def log(message):
        if verbose:
            print "     " + message
            
            
def sleep(secs):
    C.sleep(secs)

verbose = True


print "The Bigpeice"
print "Designed & Built by Bigpeice"
print "Code Copyright Bigpeice 2016. All rights reserved. Do not reuse without explicit permission"

print "Initialising..."
R = CRobot(0, 1, 10, 10, 10, 0, 0)
print "Zone: " + str(R.R.zone)
print "Mode: " + str(R.R.mode) #Doesn't work on simulator
log("Starting Algorithm")
#Start Algorithm

R.arm.open()
#R.motion.clockwise(45)
R.motion.forward(2)  #Get initial tokens
R.arm.close()
R.cubecount += 4
R.motion.anticlockwise(90)



while R.cubecount <= 6 and C.time_left() > 60:
    R.eyes.update()
    target = R.eyes.closest_token()
    log("Targeting: " + str(target.info.offset))
    if target != None and isinstance(target,Marker):
        R.motion.clockwise(target.rot_y)
        R.motion.forward(target.dist/2)
        R.eyes.update()
        new_target = get_token_by_offset(target.info.offset)
        if new_target != None:
            R.motion.clockwise(new_target.rot_y)
            R.arm.open()
            R.motion.forward(new_target.dist* 2)
            R.cubecount += 1
            R.arm.close()
    else:
        R.motion.clockwise(90)




R.arm.close()
R.motion.navigate_to_zone()
R.arm.open()
R.motion.reverse(2)
R.arm.close()

#Scan for poison

R.eyes.update()
poison_token = R.eyes.get_poison()
if poison_token != None:
    x,y = R.motion.check_location()
    angle = R.motion.check_angle()
    poison_angle = angle+poison_token.rot_y
    horizontal_distance = math.cos(math.radians(poison_token.centre.polar.rot_x)) * poison_token.dist # calculate position of poison
    poison_x = x-math.sin(math.radians(poison_angle))*horizontal_distance
    poison_y = y+math.cos(math.radians(poison_angle))*horizontal_distance
    if (poison_y > 6-poison_x) and (poison_x<4) and (poison_y < 4)):#if poison token is in zone
        R.motion.clockwise(poison_token.rot_y)
        R.motion.forward(poison_token.dist /2)
        halfway_distance = poison_token.dist /2
        R.eyes.update()
        poison_token = R.eyes.get_poison()
        if poison_token != None:
            R.motion.clockwise(poison_token.rot_y)
            R.motion.forward(poison_token.dist + 2)
        else:
            print "Oh sh*t, the poison is in our zone and we can't see it"
            R.motion.forward(halfway_distance +2)

    else:
        pass
    
    
else:
    pass

C.victory()







