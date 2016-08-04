
import math
#class Navigation():


def getAngleToPoint(x_landmark, y_landmark):
    #x_orig,y_orig = self.check_location()
    x_orig = 0
    y_orig = 0
    deltaY = y_landmark - y_orig
    deltaX = x_landmark - x_orig
    print(deltaX,deltaY)
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



    def navigate_to_zone(self):
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
            clockwise(angle_to_turn)
        else:
            anticlockwise(360-angle_to_turn)

        forward(distance_to_go)


def in_score_zone():
    x,y = self.check_location()
    if y > 6-x:
        return True
    else:
        return False


def check_location(self):
    log("Getting location")
    self.update()
    wall_markers = self.get_wall_markers()
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

        total_cartesian[0] += marker_positions[marker.info.offset][zone][0] + \
                              (math.sin(math.radians(angle)) * horizontal_distance)
        total_cartesian[1] += marker_positions[marker.info.offset][zone][1] + \
                              (math.cos(math.radians(angle)) * horizontal_distance)
    number_of_visible_wall_markers = len(wall_markers)

    return total_cartesian[0] / number_of_visible_wall_markers, total_cartesian[
        1] / number_of_visible_wall_markers

def check_angle(self):
    log("Getting angle")
    self.update()
    wall_markers = self.get_wall_markers()

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
    angle = angle + (90 * zone)

    while angle > 360:
        angle -= 360
    return angle

print("dicks")
print (getAngleToPoint(2,3))
print (getAngleToPoint(2,2))
print (getAngleToPoint(-2,2))
print (getAngleToPoint(-2,-2))
print (getAngleToPoint(2,-2))

print (getAngleToPoint(2,0))
print (getAngleToPoint(0,2))
print (getAngleToPoint(-2,0))
print (getAngleToPoint(0,-2))

