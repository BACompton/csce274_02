#!/usr/bin/python

import math
import matplotlib.pyplot as plt
import sys

import main


class _State:
    x = None
    y = None
    angle = None

    def __init__(self):
        self.reset()

    def reset(self):
        self.x = 0
        self.y = 0

        # Set angle to Upward
        self.angle = math.pi/2


def process_log(log, x_coords, y_coords):
    """
    :type log file
    :param log:
    :return:
    """
    run_index = -1
    curr_state = _State()
    moving = False

    for line in log:
        # Get the datum for current statement w/o additional spaces
        datum = line.split(",")[1].strip()

        if datum == "BUTTON":
            if not moving:
                curr_state.reset()

                # Add new run
                x_coords.append([curr_state.x])
                y_coords.append([curr_state.y])
                run_index += 1
            moving = not moving

        elif datum == "UNSAFE Wheel Drop":
            moving = False

        elif datum.endswith(" mm"):
            dist = float(datum.replace(" mm", ""))

            # Transform the x and y coordinate (in m)
            curr_state.x += dist * math.cos(curr_state.angle)
            curr_state.y += dist * math.sin(curr_state.angle)

            if run_index >= 0 and \
                    len(x_coords) > run_index and len(y_coords) > run_index:
                x_coords[run_index].append(curr_state.x)
                y_coords[run_index].append(curr_state.y)

        elif datum.endswith(" deg"):
            # Convert to datum from degrees to radians
            deg = float(datum.replace(" deg", ""))
            rad = math.radians(deg)

            # Transform the angle
            curr_state.angle += rad

def plot_main():
    file_name = main.default_log

    # Coordinate variables are a set of all run. Each run contains a list of all
    # the coordinates for a specific axis.
    # Ex: The 1st runs coordinates are accessed x_coords[0] while the initial
    #     point in the first run is  x_coords[0][0]
    x_coords = []
    y_coords = []

    if len(sys.argv) > 1:
        file_name = sys.argv[1]

    try:
        log = open(file_name, "r")
    except IOError:
        print "File could not be opened"
        return -1

    process_log(log, x_coords, y_coords)
    # If the same number of runs
    if len(x_coords) == len(y_coords):
        # Plot each run if there is the same amount of x and y coordinates
        for run in range(0, len(x_coords)):
            if len(x_coords[run]) == len(y_coords[run]):
                plt.plot(x_coords[run], y_coords[run])

    # Plot formatting
    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    plt.title("Robot's path based on '" + file_name + "'")
    plt.grid(True)

    plt.show()



    log.close()

if __name__ == '__main__':
    plot_main()