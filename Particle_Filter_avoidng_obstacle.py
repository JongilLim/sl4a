
# Standard set for Ice
import sys, traceback#, Ice
import time

# 2014/07/28
import android
import time

from math import *
import random
import csv
import lineutils

# 2014/07/28
droid = android.Android()

# some top level parameters

eps = 1.0e-10

# 2014/08/28{
grid = [[0, 0, 1, 0, 1, 0, 1],
        [1, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 1, 1, 0, 0],
        [0, 1, 0, 1, 0, 0, 1]]

X = [0.335, 1.005, 1.675, 2.345, 3.015,  3.685, 4.355]
Y = [0.335, 1.005, 1.675, 2.345]


coordinate = [[(X[0], Y[3]), (X[1], Y[3]), (X[2], Y[3]), (X[3], Y[3]), (X[4], Y[3]), (X[5], Y[3]), (X[6], Y[3])],
              [(X[0], Y[2]), (X[1], Y[2]), (X[2], Y[2]), (X[3], Y[2]), (X[4], Y[2]), (X[5], Y[2]), (X[6], Y[2])],
              [(X[0], Y[1]), (X[1], Y[1]), (X[2], Y[1]), (X[3], Y[1]), (X[4], Y[1]), (X[5], Y[1]), (X[6], Y[1])],
              [(X[0], Y[0]), (X[1], Y[0]), (X[2], Y[0]), (X[3], Y[0]), (X[4], Y[0]), (X[5], Y[0]), (X[6], Y[0])]]

init = []
goal = []
#2014/09/09
path = []

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

cost = 1
# 2014/08/28}

# 2014/07/28
send_sonar = 'A'
send_movement = 'B'
#send_wait = 'C'
stop = 'C'
#2014/09/09
send_obstacle = 'D'

# Measurements could be inprecise

bearing_noise = 0.3 # Noise parameter: should be included in sense function.
steering_noise = 0.1 # Noise parameter: should be included in move function.
distance_noise = 0.1 # Noise parameter: should be included in move function.

# List of wall segments [((start_x,start_y),(end_x,end_y)), (...)]
room_plan = []

# Horizontal (x) and vertical (y) ranges of the room
world_x_range = [0.0, 0.0]
world_y_range = [0.0, 0.0]

# Robot dimensions in meters
robot_length = 0.320
#robot_width = 0.195
robot_width = 0.180

# 2014/07/28
def bt_out(value):
    
    droid.bluetoothWrite(value)

# 2014/08/28
def search(init, goal):
    
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1
    action = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0

    open_map = [[g, x, y]]

    found = False  # flag that is set when search is complet
    resign = False # flag set if we can't find expand

    while not found and not resign:
        if len(open_map) == 0:
            resign = True
            return 'fail'
        else:
            open_map.sort()
            open_map.reverse()
            next = open_map.pop()
            x = next[1]
            y = next[2]
            g = next[0]
            
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            open_map.append([g2, x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i
                                
    x = goal[0]
    y = goal[1]
    path = [goal]
    while x != init[0] or y != init[1]:    
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        x = x2
        y = y2
        path.append((x,y))
        
    path.reverse()

    #2014/09/09
    robot_coordinate = []
    
    for i in range(len(path)):
        x = path[i][0]
        y = path[i][1]
        #2014/09/09
        robot_coordinate.append(coordinate[x][y])

    #2014/09/09
    print 'Coordinate : ', robot_coordinate
    
    #2014/09/09{
    f = open('trajectory'+'.dat', 'w')
    for i in range(len(robot_coordinate)):
        f.write (str(robot_coordinate[i][0])+'\t'+str(robot_coordinate[i][1])+'\n')
    f.close()
    #2014/09/09}

    #2014/09/09
    return path


def write_meas(robot_pos, mes, f):

    #08/19
    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] + mes[0] * cos(robot_pos[2]))+'\t'+str(robot_pos[1] + mes[0] * sin(robot_pos[2]))+'\n\n')

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] - mes[6] * cos(robot_pos[2]))+'\t'+str(robot_pos[1] - mes[6] * sin(robot_pos[2]))+'\n\n')

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] + mes[1] * cos(robot_pos[2]+(3*pi)/18))+'\t'+str(robot_pos[1] + mes[1] * sin(robot_pos[2]+(3*pi)/18))+'\n\n')

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] + mes[7] * cos(robot_pos[2]-(3*pi)/18))+'\t'+str(robot_pos[1] + mes[7] * sin(robot_pos[2]-(3*pi)/18))+'\n\n')

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] + mes[2] * cos(robot_pos[2]+(6*pi)/18))+'\t'+str(robot_pos[1] + mes[2] * sin(robot_pos[2]+(6*pi)/18))+'\n\n')

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] + mes[8] * cos(robot_pos[2]-(6*pi)/18))+'\t'+str(robot_pos[1] + mes[8] * sin(robot_pos[2]-(6*pi)/18))+'\n\n')

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] + mes[3] * cos(robot_pos[2]+(9*pi)/18))+'\t'+str(robot_pos[1] + mes[3] * sin(robot_pos[2]+(9*pi)/18))+'\n\n')

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] + mes[9] * cos(robot_pos[2]-(9*pi)/18))+'\t'+str(robot_pos[1] + mes[9] * sin(robot_pos[2]-(9*pi)/18))+'\n\n')

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] + mes[4] * cos(robot_pos[2]+(12*pi)/18))+'\t'+str(robot_pos[1] + mes[4] * sin(robot_pos[2]+(12*pi)/18))+'\n\n')

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] + mes[10] * cos(robot_pos[2]-(12*pi)/18))+'\t'+str(robot_pos[1] + mes[10] * sin(robot_pos[2]-(12*pi)/18))+'\n\n')

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] + mes[5] * cos(robot_pos[2]+(15*pi)/18))+'\t'+str(robot_pos[1] + mes[5] * sin(robot_pos[2]+(15*pi)/18))+'\n\n')

    f.write(str(robot_pos[0])+'\t'+str(robot_pos[1])+'\n')
    f.write(str(robot_pos[0] + mes[11] * cos(robot_pos[2]-(15*pi)/18))+'\t'+str(robot_pos[1] + mes[11] * sin(robot_pos[2]-(15*pi)/18))+'\n')


def dump_measurements(plan_list, robot_pos, iter_count, Z):
    
    f = open('sonar_meas'+str(iter_count).zfill(3)+'.dat', 'w')
    print ("Sonars:", Z)
    write_meas(robot_pos, Z, f)
    f.close()
    mes = lineutils.measurements(plan_list, robot_pos)
    f = open('ideal_meas'+str(iter_count).zfill(3)+'.dat', 'w')
    write_meas(robot_pos, mes, f)
    f.close()


# Generates motions list based on the trajectory specified as a set of
# waypoints. We assume, that initial position of the robot has
# coordinates as specified in the first line of the trajectory file
# and bearing (angle) is 0, i.e. robot is parallel to X axis facing to
# the right.
def generate_motions(trajectory_file_name):
    
    motions = []
    f = open(trajectory_file_name, 'r')

    prev_angle = 0.0
    b = None
    for line in f:
        x, y = line.split('\t')
        if b is not None:
            a = b
            b = (float(x), float(y))
            (x1, y1) = a
            (x2, y2) = b
            #2014/09/03
            (x0, y0) = ((x2 - x1), (y2 - y1))

            #2014/09/03{
            if x0 > 0 and y0 >= 0:
                theta = atan(y0/x0)
            elif x0 > 0 and y0 < 0:
                theta = atan(y0/x0) + 2 * pi
            elif x0 < 0:
                theta = atan(y0/x0) + pi
            elif x0 == 0 and y0 > 0:
                theta = pi/2
            elif x0 == 0 and y0 <0:
                theta = (3*pi)/2
            #2014/09/03}

            distance = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

            #2014/09/03{
            rotate = prev_angle - theta 
            if 0 < rotate and rotate < pi:
                rotate = -abs(rotate)
            elif -pi < rotate and rotate < 0:
                rotate = abs(rotate)
            elif pi < rotate:
                rotate = (2*pi) - abs(rotate)
            elif rotate < -pi:
                rotate = -((2*pi) - abs(rotate))

            #2014/09/08
            motions.append([rotate,distance,[x2,y2,theta]])
            prev_angle = theta
            #2014/09/03}
        else:
            b = (float(x), float(y))

    f.close()
    
    return motions


# Request left and right wheel to travel sl and sr distance with
# defined speed. Speed is measured in encoder pulses per second.
# There are 333 pulses for complete wheel turn. So default speed 33
# corresponds to the 1/10 wheel revolution per second.
def make_motion_step(sl, sr, speed = 1):
    
    #print "Motion step:", sl, sr

    if abs(sl) < eps:
        actuator_cmd1_speed = 0
        actuator_cmd1_distance = 0
    else:
        actuator_cmd1_speed = speed if sl >= 0 else -speed
        actuator_cmd1_distance = abs(sl) / (3.14159 * 0.06)

    if abs(sr) < eps:
        actuator_cmd2_speed = 0
        actuator_cmd2_distance = 0
    else:
        actuator_cmd2_speed = speed if sr >= 0 else -speed
        actuator_cmd2_distance = abs(sr) / (3.14159 * 0.06)
    movement = [actuator_cmd1_speed, round(actuator_cmd1_distance,2), actuator_cmd2_speed, round(actuator_cmd2_distance,2)]
    move = ' '.join(str(n) for n in movement)+' '+'\n'
    print "Actual motion l/r:", move

    bt_out(send_movement)
    bt_out(move)
    ans = droid.bluetoothRead(10).result
        #time.sleep(0.2)
    #time.sleep(4.6)
    print 'done'


def sonar_measurment():
    
    #2014/07/21
    bt_out(send_sonar)
    time.sleep(2)
    measurement = droid.bluetoothRead(4096).result
    time.sleep(1.4)
    Z = measurement.split()
    #print 'Z', Z
    for i in range(0,12):
       Z[i] = float(Z[i])/100
       #2014/09/04
       if Z[i] != 0:
           Z[i] += 0.055
    #print a

    return Z


#2014/09/09
def check_obstacle(theta):
    
    if theta < 0:
        theta += 2*pi
    degrees = int(round((180 * theta / pi),0))
    
    if 0 <= degrees and degrees < 90:
        rotate = [1, degrees]
    elif 90 <= degrees and degrees < 180:
        degrees -= 90
        rotate = [2, degrees]
    elif 180 <= degrees and degrees < 270:
        degrees -= 180
        rotate = [3, degrees]
    elif 270 <= degrees and degrees < 360:
        degrees -= 270
        rotate = [4, degrees]

    rotation = ' '.join(str(n) for n in rotate)+' '+'\n'
    bt_out(send_obstacle)
    bt_out(rotation)
    time.sleep(2.5)
    distance = droid.bluetoothRead(10).result
    time.sleep(1.5)
    distance = float(distance)/100
    print "Check distance : ", distance

    return distance


def measurement_prob(plan_list, particle, measurements):
    
    # exclude particles outside the room
    if not lineutils.point_inside_polygon(plan_list, particle):
        return 0.0

    # calculate the correct measurement
    predicted_measurements = lineutils.measurements(room_plan, particle)

    # compute errors
    prob = 1.0
    count = 0
    for i in xrange(0, len(measurements)):
        if measurements[i] != 0:
            error_mes = abs(measurements[i] - predicted_measurements[i])
            # update Gaussian
            #error *= (exp(- (error_mes ** 2) / (bearing_noise ** 2) / 2.0) / sqrt(2.0 * pi * (bearing_noise ** 2)))

            #8/28
            prob += (exp(- (error_mes ** 2) / (bearing_noise ** 2) / 2.0) / sqrt(2.0 * pi * (bearing_noise ** 2)))
            count += 1
    prob /= count
    prob = prob ** 4

    return prob


# Here we are using equations for two-wheels differential
# steering system as presented here 
# http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html :
#
# S = (Sr+Sl)/2
# Theta = (Sr-Sl)/2b+theta0
# X = s*cos(theta)+x0
# Y = s*sin(theta)+y0
# Where Sr and Sl is the distance travelled by each wheel and b is the
# distance between wheels (vehicle width)
#
def move(particle, motion):
    
    (x,y,theta0) = particle
    (delta_theta, s, _) = motion
    delta_theta = random.gauss(delta_theta, steering_noise)
    s = random.gauss(s, distance_noise)
    
    theta = theta0 + delta_theta;
    x += s * cos(theta)
    y += s * sin(theta)

    return (x,y,theta)


# extract position from a particle set
def get_position(p):
    
    x = 0.0
    y = 0.0
    orientation = 0.0
    #2014/09/04
    X = []
    Y = []
    D = []
    (_,_,init_orientation) = p[0]
    for (px,py,theta) in p:
        x += px
        y += py

        #2014/09/04
        X.append(px)
        Y.append(py)
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((theta - init_orientation + pi) % (2.0 * pi)) 
                        + init_orientation - pi)
        #2014/09/04
        D.append((((theta - init_orientation + pi) % (2.0 * pi)) 
                        + init_orientation - pi))

    #2014/09/04
    average_x = x / len(p)
    variance_x = sum((average_x - value ) ** 2 for value in X) / len(X)
    average_y = y / len(p)
    variance_y = sum((average_y - value ) ** 2 for value in Y) / len(Y)
    average_d = orientation / len(p)
    variance_d = sum((average_d - value ) ** 2 for value in D) / len(D)

    #2014/09/08
    return (x / len(p), y / len(p), orientation / len(p), variance_x, variance_y, variance_d)


#2014/09/08
def calculate_rotate(rotate):
    
    if 0 < rotate and rotate < pi:
        rotate = -abs(rotate)
    elif -pi < rotate and rotate < 0:
        rotate = abs(rotate)
    elif pi < rotate:
        rotate = (2*pi) - abs(rotate)
    elif rotate < -pi:
        rotate = -((2*pi) - abs(rotate))

    return rotate


#2014/09/08
def move_position(x1, y1, prev_angle, x2, y2, original_d):

    (x0, y0) = ((x2 - x1), (y2 - y1))

    if x0 > 0 and y0 >= 0:
        theta = atan(y0/x0)
    elif x0 > 0 and y0 < 0:
        theta = atan(y0/x0) + 2 * pi
    elif x0 < 0:
        theta = atan(y0/x0) + pi
    elif x0 == 0 and y0 > 0:
        theta = pi/2
    elif x0 == 0 and y0 <0:
        theta = (3*pi)/2

    distance = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

    rotate = calculate_rotate(prev_angle - theta ) 
    adjust_direction = calculate_rotate(theta -  original_d)

    sr = rotate * robot_width
    sl = -sr

    make_motion_step(sl, sr) # Rotate on place
    make_motion_step(distance, distance) # Drive forward

    sr = adjust_direction * robot_width
    sl = -sr
    
    make_motion_step(sl, sr) # Rotate on place


#2014/09/09
def repath(path, path_count, goal):
    
    print "Obstacle : ", path[path_count+1]
    obstacle = path[path_count+1]
    x = obstacle[0]
    y = obstacle[1]
    grid[x][y] = 1
    for i in range(len(grid)):
        print grid[i]
    path = search(path[path_count], goal)
    grid[x][y] = 0

    return path


#2014/09/09
def prediction_and_measurement(motions, path, goal, N, p, iter_count, complete):

    path_count = 0
    
    for motion in motions:

        #2014/09/09{
        (delta_theta, s, _) = motion

        distance = check_obstacle(delta_theta)
        
        if 0.02 < distance and distance < s + 0.02:
            print "Path replanning"
            path = repath(path, path_count, goal)
            motions = generate_motions('trajectory.dat')
            # path, motions
            complete = 1

            return (motions, path, p, iter_count, complete)
            

        p = map(lambda particle: move(particle, motion), p)

        sr = (delta_theta) * robot_width
        sl = -sr

        make_motion_step(sl, sr) # Rotate on place
        make_motion_step(s, s) # Drive forward
        #2014/09/09}

        # Measurement update
        Z = sonar_measurment()

        w = map(lambda particle: measurement_prob(room_plan, particle, Z), p)

        # Resampling
        p2 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p2.append(p[index])
        p = p2

        

        # Dump current particle set to file for plotting
        (_,_,robot_ideal_pos) = motion
        dump_measurements(room_plan, robot_ideal_pos, iter_count, Z)        

        # Dump current particle set to file for plotting
        f = open('iteration'+str(iter_count).zfill(3)+'.dat', 'w')
        map(lambda (x,y,_): f.write(str(x)+'\t'+str(y)+'\n'), p)
        f.close()

        #2014/09/08
        estimation = get_position(p)
        print "position : ", "(x =", estimation[0], ", y =", estimation[1], ", theta =", estimation[2], ")"
        print "variance : ", "(x =", estimation[3], ", y =", estimation[4], ", theta =", estimation[5], ')\n'
        #2014/09/08}

        #2014/09/08{
        if (iter_count % 4 ) == 0:
            if estimation[3] < 0.1 and estimation[4] < 0.1 and estimation[5] < 0.1:
                print "Adjust location"
                particle_x = estimation[0]
                particle_y = estimation[1]
                particle_d = estimation[2]
                original_x = motions[path_count][2][0]
                original_y = motions[path_count][2][1]
                original_d = motions[path_count][2][2]
                
                if estimation[2] < 0:
                    particle_d += 2*pi
                elif estimation[2] > 2*pi:
                    particle_d -= 2*pi
                    
                adjust_position = move_position(round(particle_x, 3), round(particle_y, 3), particle_d, original_x, original_y, original_d)
        #2014/09/08}
                
        iter_count += 1

        #2014/09/09
        path_count += 1

    complete = 0

    return (motions, path, p, iter_count, complete)


#2014/09/09
def particle_filter(motions, path, goal, N=1500):

    # Make particles

    world_x_size = abs(world_x_range[1] - world_x_range[0])
    world_y_size = abs(world_y_range[1] - world_y_range[0])
    p = []
    while len(p) < N:
        # particle is a vector (x,y,bearing)
        particle = (random.random() * world_x_size + world_x_range[0],
                    random.random() * world_y_size + world_y_range[0],
                    random.random() * 2.0 * pi)
        # add only particles inside our room
        if lineutils.point_inside_polygon(room_plan, particle):
            p.append(particle)
    

    # Update particles
    iter_count = 0

    #2014/09/09
    complete = 1
    
    #2014/09/09{
    #complete : 0, not complete : 1
    while(complete == 1):
        information = prediction_and_measurement(motions, path, goal, N, p, iter_count, complete)
        motions = information[0]
        path = information[1]
        p = information[2]
        iter_count = information[3]
        complete = information[4]
    #2014/09/09}
        
    return get_position(p)


# Main application class with run() function as an entry point to the
# application
class Client():

    # 2014/08/28
    for i in range(len(grid)):
        print grid[i]
    
    init = input("start position(x,y) : ")
    goal = input("Final position(x,y) : ")

    #2014/09/09
    path = search(init,goal)
    
    time.sleep(1.5)

    # 2014/07/21
    droid.bluetoothConnect('00001101-0000-1000-8000-00805F9B34FB', '20:13:05:09:11:90')
    time.sleep(1)

    while True:
        
        # Read room plan and create list with wall coordinates
        with open('plan.dat', 'r') as planfile:
            planreader = csv.reader(planfile, delimiter='\t')
            b = None
            for (x, y) in planreader:
                # Calculate world boundaries
                if float(x) < world_x_range[0]:
                    world_x_range[0] = float(x)
                elif float(x) > world_x_range[1]:
                    world_x_range[1] = float(x)

                if float(y) < world_y_range[0]:
                    world_y_range[0] = float(y)
                elif float(y) > world_y_range[1]:
                    world_y_range[1] = float(y)

                # Construct wall segment and add to the room_plan
                if b is not None:
                    a = b
                    b = (float(x), float(y))
                    room_plan.append((a,b))
                else:
                    b = (float(x), float(y))

            # Create proxy interface to our robot's chassis (left and
            # right wheels) using parameters (host, port, etc.) specified
            # in the configuration file as Chassis.proxy property
 
        receiver = []
        receiver = sonar_measurment()

        # Make some motions and estimate resulting position using our particle filter
        motions = generate_motions('trajectory.dat')
        #2014/09/09
        estimated_position = particle_filter(motions, path, goal)
        print "Estimated final position after motion: ", estimated_position


if __name__ == "__main__":
    app = Client()
