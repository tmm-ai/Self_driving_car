import picar_4wd as fc
import math
import A_star
from speed import Speed
import time
import obj_det2


def get_obstacles_on_map(dim_x, dim_y, angle_library, grid_condenser, start_row, start_col):
    """""
    This function creates a map/grid for the car and places all the obstacles found from the ultrasonic
    scan. If obstacle readings are found in a row, we fill in the cells on the maps between the coordinates
    of the obstacle reading to prevent the car from trying to drive through the obstacle but 
    appears to be a gap between the readings on the map.
    
    The padding function is then called to add padding to the obstacles on the map to prevent the car
    from bumping into obstalces. The map without, and with padding is then printed out.
    
    Inputs: starting destination, end destination, ultrasonic angle readings, grid condenser (#centimeters per unit)
    Outputs: The car successfully moves through the obstacle course to the final destination
    """""
    
    the_map = [[0 for x in range(dim_x)] for y in range(dim_y)]
    distance_reads = find_object_distances(angle_library)
    print("DISTANCE READS")
    print(distance_reads)

    prev_1 = False
    prev_x = 0
    prev_y = 0
    the_obstacles_set = set()
    for idx in range(len(distance_reads)):
        if 70 > distance_reads[idx] > 0:
            # I am adding in 90 to the angles so that we get a map that orients the car
            # perspective from the bottom looking up.
            radians = (angle_library[idx] + 90) / (180 / math.pi)
            x = int(distance_reads[idx] * math.cos(radians)) // grid_condenser
            y = int(distance_reads[idx] * math.sin(radians)) // grid_condenser

            # this puts the car perspective in the middle of the x axis
            x += start_col
            if x >= dim_x: continue
            # this orients the car at the bottom looking up, so that objects in the deepest row
            # are the closest to the car.
            y = start_row - y
            if 0 > y > dim_y: continue
            # getting rid of noise that constantly shows up on far left due to sensor issues.
            if y > dim_y - 5 and x < 25: continue

            # the ultrasonic is only taking reading every 3 degress and we do not want the car to try
            # to drive between continiously read obstacle on the map that may show up with multiple
            # blank cells on the map, so we fill in a line of continious obstalces on the map once
            # we have repeated, continious readings for an obstacle.
            if prev_1 == True:
                the_map[y][x] = 3
                # this is to change the first point in a continuous object to 3,
                # retroactively, after the second continuous point was found
                if prev_x != 0 and prev_y != 0:
                    the_map[prev_y][prev_x] = 3
                    the_obstacles_set.add((prev_y, prev_x))
            else:
                # this is to mark a object found, but we are not sure if it the next read
                # out will also find an object. If object is found on the next ultrasonic
                # read, then this will remain 2.
                the_map[y][x] = 2
                the_obstacles_set.add((y, x))

            if prev_1:
                # determining the amount of difference between x and y points
                # and what the max amount of change is.
                x_dif = x - prev_x
                y_dif = y - prev_y
                x_sign = y_sign = 1
                # the pos or neg differences(x_sign, y_sign) are used in filling in the gaps
                # between continuous objects in the for loop below.
                if x_dif < 0:
                    x_sign = -1
                if y_dif < 0:
                    y_sign = -1
                max_change = max(abs(x_dif), abs(y_dif))
                for new in range(1, max_change):
                    if abs(x_dif) > abs(y_dif):
                        fill_x = x + new * x_sign * -1
                        # the x axis has the most amount of difference / distance between points
                        # multi is a fraction in this case so that the y axis changes at
                        # a proportional rate the the x axis change.
                        multi = abs(y_dif) / abs(x_dif)
                        fill_y = y + int(new * multi) * y_sign * -1
                    # repeating the same process if the y points are furthest apart.
                    if abs(x_dif) <= abs(y_dif):
                        multi = abs(x_dif) / abs(y_dif)
                        fill_x = x + int(new * multi) * x_sign * -1
                        fill_y = y + new * y_sign * -1
                    the_map[int(fill_y)][int(fill_x)] = 1
                    the_obstacles_set.add((int(fill_y), int(fill_x)))
            prev_1 = True
            # setting the new previous points for the next ultrasonic read.
            prev_x = x
            prev_y = y
        else:
            prev_1 = False
    obstacle_list = list(the_obstacles_set)
    print("MAP WITHOUT PADDING\n\n")
    for row in the_map:
        print(row)
    map_w_padding = add_padding(padding, obstacle_list, the_map, dim_x, dim_y)
    print("MAP -- - -- - W - - - - --PADDING\n\n")
    for row in map_w_padding:
        print(row)
    return map_w_padding


def add_padding(padding, obstacle_list, the_map, dim_x, dim_y):
    """""
    For each object found in the ultrasonic scan and placed on the map, an amount of padding is added
    to the map to prevent the car from crashing into the obstacle
    padding is the number of cells around the obstacle that will be filled in with padding. 
    On the map, padding is noted by the number 4. 
    
    Inputs: the map with obstalces, starting destination, coordinates of obstacles, amount of padding
    Output: a map with obstacles and padding around the obstacles
    """""
    pad_list = []
    for pad in range(-padding, padding):
        pad_list.append([padding, pad])
        pad_list.append([pad, -padding])
        pad_list.append([-padding, pad])
        pad_list.append([pad, padding])

    for obstacle in obstacle_list:
        obstacle_y = obstacle[0]
        obstacle_x = obstacle[1]
        for pad in pad_list:
            y = pad[0]
            x = pad[1]
            # print("obs_y", obstacle_y, " y:", y, "obs_x", obstacle_x, " x:", x)
            if (obstacle_y + y) < 0: continue
            if (obstacle_y + y) >= dim_y: continue
            if (obstacle_x + x) < 0: continue
            if (obstacle_x + x) >= dim_x: continue
            # print("Y:",obstacle_y + y, " X: ", obstacle_x + x )
            the_map[obstacle_y + y][obstacle_x + x] = 4

    return the_map


def find_object_distances(angle_library):
    """""
    This function calls the ultrsonic sensor every 3 degrees across a 180 degree span to map
    obstacles in front of the car
    
    Input: an array of all the angles from 0 to 180, every 3 degrees.
    Output: an array of distances to the obstacles is returned.
    """""
    distance_reads = []
    for current_angle in angle_library:
        distance = fc.get_distance_at(current_angle)
        # print("got distance:", distance)
        distance_reads.append(distance)
    return distance_reads


def full_left(cell_jump):
    """""
    This function and the next 4 steer the car in the direction of the function name. 
    the sleep time differences address the pecularitites of the motor being stronger or weaker
    in different turning directions. 
    
    Input: cell_jump - which equates to how much time the car should drive forward after the turn 
    is complete
    Output: a precise turn is made and the car then drives forward for a certain amount of time.
    
    """""
    fc.turn_left(10)
    time.sleep(1.21)
    fc.forward(10)
    time.sleep(0.16 * cell_jump)


def diag_left(cell_jump):
    fc.turn_left(10)
    time.sleep(0.66)
    fc.forward(10)
    time.sleep(0.16 * math.sqrt(2)* cell_jump * 1.1)


def diag_right(cell_jump):
    fc.turn_right(10)
    time.sleep(0.58)
    fc.forward(10)
    time.sleep(0.16 * math.sqrt(2) * cell_jump)


def full_right(cell_jump):
    fc.turn_right(10)
    time.sleep(1.08)
    fc.forward(10)
    time.sleep(0.16 * cell_jump)


def forward(cell_jump):
    fc.forward(10)
    time.sleep(0.17 * cell_jump)

    """
    End of direction specific driving functions
    """


def navigate(the_path, current_row, current_col, second_scan):
    """""
    This function takes drives the car along the path created from the A_star function through the obstacls step by 
    step through each cell. It also continiously scans for people and traffic signs and will stop appropriately for
    each.
    
    Input: the path, the starting point, and a boolean defining if we do a second scan for obstacles along the path
    Output: a car successfully driven to the final destination. 
    """""
    test_run = Speed(25)
    test_run.start()
    fc.forward(10)
    direction = 2
    # direction key: 0=full left, 1=left diag, 2=up,3=right diag, 4=full right
    cell_jump = 1.1
    # this loop goes though each step of the defined path, executing turns and contiiously looking for people and signs
    for next1 in range(1, len(the_path),1):

        pair = the_path[next1]
        next_row = pair[0]
        next_col = pair[1]
        print("Step:", next1, " at:", pair," currently at:", current_row, current_col)

        """Integrating PEOPLE and SIGN Detection Here """
        if next1%2==0:
            fc.forward(0)
            person, traffic_sign = obj_det2.run_obj_det('efficientdet_lite0.tflite', 0, 640, 480, 4, False)
            while person:
                print("Person ahead, Stop and check again in 2 seconds")
                fc.stop()
                time.sleep(2)
                person, traffic_sign = obj_det2.run_obj_det('efficientdet_lite0.tflite', 0, 640, 480, 4, False)
            if traffic_sign:
                fc.stop()
                print("Stopping for 5 seconds for stop sign")
                time.sleep(5)
            fc.forward(10)
        """ END - HUMAN - SIGN Detection"""

        """ DOING 2nd scan now"""
        if current_row < 34 and direction == 2 and second_scan is False:
            dim_x_new = 50
            dim_y_new = current_row+1 # you need this +1 so current row fits on next map.
            test_run.deinit()
            fc.stop()
            second_scan2 = True
            make_route_w_padding(dim_x_new, dim_y_new, angle_library, grid_condenser, current_row, current_col,
                                 end_row, end_col, second_scan2)
            break

        if next_row == current_row - 1 and next_col == current_col:  # NEED TO GO UP
            if direction == 0:  # currently LEFT
                print("taking RIGHT to go UP at:", pair)
                full_right(cell_jump)
            elif direction == 1:  # currently diag LEFT
                print("taking DIAG RIGHT to go UP at:", pair)
                diag_right(cell_jump)
            elif direction == 2:  # currently up
                print("continuing up at:", pair)
                forward(cell_jump)
            elif direction == 3:  # currently diag right
                print("taking DIAG LEFT to go UP at:", pair)
                diag_left(cell_jump)
            elif direction == 4:  # current RIGHT
                print("taking LEFT to go UP at:", pair)
                full_left(cell_jump)
            current_row = next_row
            direction = 2
        elif next_row == current_row - 1 and next_col == current_col - 1:  # NEED TO GO DIAG LEFT
            if direction == 0:  # currently LEFT
                print("taking diag RIGHT to diag left at:", pair)
                diag_right(cell_jump)
            elif direction == 1:  # currently diag LEFT
                print("continuing diag LEFT at:", pair)
                forward(cell_jump)
            elif direction == 2:  # currently up
                print("taking diag left at:", pair)
                diag_left(cell_jump)
            elif direction == 3:  # currently diag right
                print("taking full LEFT to go diag left at:", pair)
                full_left(cell_jump)
            # elif direction == 4: # current RIGHT
            #     print("taking LEFT to go UP at:", pair)
            #     full_left(cell_jump)
            current_row = next_row
            current_col = next_col
            direction = 1
        # this is is a full let turn, so I will only take this when going up or am currently left
        # other wise i would either be going down or path would move backwards/backtrack
        elif next_row == current_row and next_col == current_col - 1:  # NEED TO GO FULL LEFT
            if direction == 0:  # currently LEFT
                print("continuing left at:", pair)
                forward(cell_jump)
            elif direction == 1:  # currently diag LEFT
                print("taking diag LEFT to go full left at:", pair)
                diag_left(cell_jump)
            elif direction == 2:  # currently up
                print("taking FULL left at:", pair)
                full_left(cell_jump)
            current_row = next_row
            current_col = next_col
            direction = 0
        elif next_row == current_row - 1 and next_col == current_col + 1:  # NEED TO GO DIAG right)
            if direction == 1:  # currently diag LEFT
                print("taking full right from diag LEFT to go diag right at:", pair)
                full_right(cell_jump)
            elif direction == 2:  # currently up
                print("taking diag right to go diag right at:", pair)
                diag_right(cell_jump)
            elif direction == 3:  # currently diag right
                print("continuing diag right at:", pair)
                forward(cell_jump)
            elif direction == 4:  # current RIGHT
                print("taking diag LEFT to go diag right at:", pair)
                diag_left(cell_jump)
            current_row = next_row
            current_col = next_col
            direction = 3
        elif next_row == current_row and next_col == current_col + 1:  # NEED TO GO FULL right
            if direction == 2:  # currently up
                print("taking full right to go full right at:", pair)
                full_right(cell_jump)
            elif direction == 3:  # currently diag right
                print("taking diag right to go full right at:", pair)
                diag_right(cell_jump)
            elif direction == 4:  # current RIGHT
                print("taking diag LEFT to go diag right at:", pair)
                forward(cell_jump)
            current_row = next_row
            current_col = next_col
            direction = 4
        else:
            print("Outside conditionals - ELSE")
            continue
    test_run.deinit()
    fc.stop()


def make_route_w_padding(dim_x, dim_y, angle_library, grid_condenser, start_row, start_col, end_row,
                         end_col, second_scan):
    """""
    This function creates a map with obstacles and calculates the shortest path from start to end using the A* algorithm
    
    Input: dim_x and dim_y - dimensions of the map, angle_library and grid_condenser - used to get obstacles on the map, 
    start_row and start_col - starting point coordinates, end_row and end_col - ending point coordinates, 
    second_scan - flag for additional navigation
    Output: A map with the shortest path marked as 7, the defind driving path through the obstacles using the A.star 
    function, and call then s call to the navigate function with the path and start point
    """""
    map_w_padding = get_obstacles_on_map(dim_x, dim_y, angle_library, grid_condenser, start_row, start_col)
    the_path = A_star.aStarAlgorithm(start_row, start_col, end_row, end_col, map_w_padding)
    print("The A-star path\n")
    print(the_path)
    # this is to show the path on the map
    for step in the_path:
        row = step[0]
        col = step[1]
        map_w_padding[row][col] = 7
    print("Map with padding\n")
    for row in map_w_padding:
        print(row)
    navigate(the_path, start_row, start_col, second_scan)


""" Setting Basic parameters """
# the ultrasonic sensor will take a distance reading for every angle step(3), this means 60 per 180 degree scan
angle_step = 3
# creating a list of angles for every angle step
angle_library = [step for step in range(90, -91, -angle_step)]
# setting the size of our grid. Each unit is about 5 cm
# 1 unit in the grid equals the the number of centimentets in grid_condenser
dim_y = 50
dim_x = 50
grid_condenser = 5
# each unit of padding will be 1 unit on the grid
padding = 4
# start location of car
start_row = dim_y - 1
start_col = dim_x // 2
""" DESTINATIONS ***  """
end_row = 0
# endCol = 12 # first destination
end_col = 30 # second destination
speed = 10
# second scan indicates if we do a second ultrasnoic scan and mapping of route midway through the course.
second_scan = True
if "name" == "navi3":
    make_route_w_padding(dim_x, dim_y, angle_library, grid_condenser, start_row, start_col, end_row,
                         end_col, second_scan)


