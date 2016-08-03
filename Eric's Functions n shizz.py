def check_location(R):
    R.eyes.update()
    wall_markers = R.eyes.get_wall_markers()
    marker_positions = [[0,1],[0,2],[0,3],[0,4],[0,5],[0,6],[0,7],\
                        [1,0],[2,0],[3,0],[4,0],[5,0],[6,0],[7,0],\
                        [8,1],[8,2],[8,3],[8,4],[8,5],[8,6],[8,7],
                        [1,8],[2,8],[3,8],[4,8],[5,8],[6,8],[7,8],]
    total_cartesian = [0,0]
    for marker in wall_markers:
        horizontal_distance = math.cos(math.radians(marker.centre.polar.rot_x))*marker.dist
        if marker.info.offset in range(8):
            angle = marker.orientation.rot_z + 0
        elif marker.info.offset in range(8,15):
            angle = marker.orientation.rot_z + 90
        elif marker.info.offset in range(15,22):
            angle = marker.orientation.rot_z + 180
        elif marker.info.offset in range(22,28):
            angle = marker.orientation.rot_z + 270

        total_cartesian[0] += marker_positions[marker.info.offset][0]+\
                              (math.sin(math.radians(angle))*horizontal_distance
        total_cartesian[1] += marker_positions[marker.info.offset][1]+\
                              (math.cos(math.radians(angle))*horizontal_distance
    number_of_visible_wall_markers = len(wall_markers)
                               
    return total_cartesian[0]/number_of_visible_wall_markers, total_cartesian[1]/number_of_visible_wall_markers



def find_wall_markers(markers)

