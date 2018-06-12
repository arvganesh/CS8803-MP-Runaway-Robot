# ----------
# Part Four
#
# Again, you'll track down and recover the runaway Traxbot.
# But this time, your speed will be about the same as the runaway bot.
# This may require more careful planning than you used last time.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time.
#
# ----------
# GRADING
#
# Same as part 3. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random

##################
## From Problem Set 2
##################
def kfilter(x, P, measurements):
    dt = 0.1
    u = matrix([[0.], [0.], [0.], [0.]]) # external motion
    F = matrix([[1., 0., dt, 0.], [0., 1., 0., dt], [0., 0., 1., 0.], [0., 0., 0., 1.]])       # next state function: generalize the 2d version to 4d
    H = matrix([[1., 0., 0., 0.], [0., 1., 0., 0.]])                                         # measurement function: reflect the fact that we observe x and y but not the two velocities
    R = matrix([[0.1, 0.], [0., 0.1]])                                                       # measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
    I = matrix([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])       # 4d identity matrix

    for _, m in enumerate(measurements):

        # prediction
        x = (F * x) + u
        P = F * P * F.transpose()

        # measurement update
        Z = matrix([m])
        y = Z.transpose() - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - (K * H)) * P

    return x, P
##################
## End of Problem Set 2 code
##################

# pylint: disable=unused-argument
def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER=None):
    # This function will be called after each time the target moves.

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.
    if not OTHER: # first time calling this function, set up my OTHER variables.
        P = matrix([[0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 1000., 0.], [0., 0., 0., 1000.]]) # initial uncertainty: 0 for positions x and y, 1000 for the two velocities
        prev_angle = 0.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = [measurements, hunter_positions, hunter_headings, P, 0.] # now I can keep track of history
    else: # not the first time, update my history
        # OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings, P, prev_angle = OTHER # now I can always refer to these variables

    ##################
    ## Code change from naive_next_move
    ##################
    x_ = matrix([[target_measurement[0]], [target_measurement[1]], [0.], [0.]]) # initial state (location and velocity)

    x, P = kfilter(x_, P, measurements[-20:])

    k_x = x.value[0][0]
    k_y = x.value[1][0]
    k_x_y = (k_x, k_y)
    dy = k_x_y[1] - measurements[-1][1]
    dx = k_x_y[0] - measurements[-1][0]

    angle = atan2(dy, dx)
    prev_step = distance_between(measurements[-1], k_x_y)

    heading = angle*2 - prev_angle
    X = k_x_y[0] + cos(heading) * prev_step
    Y = k_x_y[1] + sin(heading) * prev_step
    xy_estimate = X, Y

    measurements.append(k_x_y)
    OTHER[3] = P
    OTHER[4] = angle
    ##################
    ## Code change from naive_next_move
    ##################

    heading_to_target = get_heading(hunter_position, xy_estimate)
    heading_difference = heading_to_target - hunter_heading
    turning = heading_difference # turn towards the target
    #distance = max_distance # full speed ahead!
    distance = distance_between(hunter_position, xy_estimate)

    return turning, distance, OTHER

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught



def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all
    the target measurements, hunter positions, and hunter headings over time, but it doesn't
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables

    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER

#target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
#measurement_noise = .05*target.distance
#target.set_noise(0.0, 0.0, measurement_noise)
#
#hunter = robot(-10.0, -10.0, 0.0)
#
#print demo_grading(hunter, target, next_move)
