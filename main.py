#!/usr/bin/python

import datetime
import math
import random
import sys
import threading
import time


import robot_inf
import serial_inf
import sensor_inf


# =============================================================================
#                       Main program for project 2
# =============================================================================

# Flag for testing adjustments. This will mainly decide the robot's state.
_prog_test = False
_log_sema = threading.Semaphore()


def _timestamp():
    """ Gets the timestamp of the local system
    :return:
        Returns the current time
    """
    return datetime.datetime.fromtimestamp(time.time())\
                   .strftime("%Y-%m-%d %H:%M:%S")


def _log_stmt(log, datum):
    """ Creates a log statement in the supplied file with the provided datum.

    :type log file
    :param log:
        The log file
    :param datum:
        The data to log
    """
    _log_sema.acquire()                 # Acquire Lock
    log.write(_timestamp()+","+str(datum)+"\n")
    _log_sema.release()                 # Release Lock


class DriveControl(threading.Thread):
    """
        This is the low level actuator controller for the random walk algorithm.

        :type _sensor sensor_inf.Sensor
        :type _log file
        :type _log_unsafe bool
        :type _stop bool
    """
    _sensor = None
    _log = None
    _log_unsafe = None
    _stop = None

    def __init__(self, sensor, log):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self._stop = False
        self._sensor = sensor
        self._log_unsafe = False
        self._log = log

    def run(self):
        robot = self._sensor.get_robot()        # The robot's connection
        wheel_vel = 100                         # The absolute value of a wheel
        next_stmt = "None"                      # Identifies the next log stmt
        wait_time = 0                           # The time between commands

        # Maximum rotation time based on wheel velocity
        max_rot_time = math.pi*robot_inf.WHEEL_BASE/(4*wheel_vel)
        # The reference encoder value for the log statement
        prev_encoder = self._sensor.get_encoders()

        while not self._wait(wait_time, robot_inf.SENSOR_UPDATE_WAIT):
            # Generate random turn time
            wait_time = random.uniform(0, max_rot_time)
            cw = self._turn_cw()                # Turn cw?
            ccw = self._turn_ccw()              # Turn ccw?

            curr_stmt = next_stmt
            next_stmt = "Angle"
            if cw and ccw:
                # Pick random direction to rotate
                if bool(random.getrandbits(1)):
                    self._drive_cw(robot, wheel_vel)
                else:
                    self._drive_ccw(robot, wheel_vel)
                    next_stmt += "CCW"
            elif cw:
                self._drive_cw(robot, wheel_vel)
            elif ccw:
                self._drive_ccw(robot, wheel_vel)
                next_stmt += "CCW"
            else:
                self._drive_forward(robot, wheel_vel)
                next_stmt = "Distance"

            # The statement below will limit the number of distance log actions
            # if curr_stmt != "Distance" or next_stmt != curr_stmt:

            # Log last action
            self._log_movement(curr_stmt, prev_encoder)
            prev_encoder = self._sensor.get_encoders()

        # Stops robot
        self._log_movement(next_stmt, prev_encoder)
        robot.drive_direct(0, 0)

    def stop(self):
        """
            Sets the flag to stop the thread to True. The thread will
            not immediately stop. Instead, the thread will exit safely.
        """
        self._stop = True

    def _wait(self, wait_time, interval):
        """ Internal method that divides the time in between actuator
            commands into small intervals. This enables the ability
            to check for the stopping condition while running an action.

        :param wait_time:
            The total amount of time to wait
        :param interval:
            The amount of time for a single interval.
        :return:
            True if the stopping condition was detected, otherwise false.
        """
        time_left = wait_time

        # Special Case
        if time_left == 0:
            return self._should_stop()

        while time_left > 0:
            start = time.time()
            # Tell the caller it needs to stop
            if self._should_stop():
                return True

            # Wait another time interval
            interval_time = interval
            if interval_time > time_left:
                interval_time = time_left
            # time_left -= sleep time + elapsed time
            time_left -= (interval_time + (time.time()-start))
            time.sleep(interval_time)
        return False

    def _should_stop(self):
        """ Determines if the actuator controller should stop.
        :return:
            True if the actuator controller should stop.
        """
        return self._stop or not self._safe_motion()

    def _safe_motion(self):
        """ Determines if movement or rotation is safe.
        :return:
            True if motion or rotation is safe.
        """
        drops = self._sensor.get_wheel_drops()

        for drop in drops:
            if drops[drop]:
                self._log_unsafe = True
                return False
        return True

    def _turn_cw(self):
        """ Determines if the robot should rotate clockwise
        :return:
            True if the robot should turn clockwise
        """
        cliffs = self._sensor.get_cliffs()

        if self._sensor.is_bump(robot_inf.Bump.BUMP_L):
            return True
        if cliffs[robot_inf.Cliff.CLIFF_L] or cliffs[robot_inf.Cliff.CLIFF_FL]:
            self._log_unsafe = True
            return True
        return False

    def _turn_ccw(self):
        """ Determines if the robot should rotate counter-clockwise
        :return:
            True if the robot should turn counter-clockwise
        """
        cliffs = self._sensor.get_cliffs()

        if self._sensor.is_bump(robot_inf.Bump.BUMP_R):
            return True
        if cliffs[robot_inf.Cliff.CLIFF_R] or cliffs[robot_inf.Cliff.CLIFF_FR]:
            self._log_unsafe = True
            return True
        return False

    def _drive_ccw(self, robot, wheel_vel):
        """ Rotates the robot counter-clockwise.

        :type robot robot_inf.Robot
        :param robot:
            The robot's connection
        :type wheel_vel int
        :param wheel_vel:
            The absolute value of a wheel's velocity
        """
        robot.drive_direct(wheel_vel, -wheel_vel)

    def _drive_cw(self, robot, wheel_vel):
        """ Rotates the robot clockwise.

        :type robot robot_inf.Robot
        :param robot:
            The robot's connection
        :type wheel_vel int
        :param wheel_vel:
            The absolute value of a wheel's velocity
        """
        robot.drive_direct(-wheel_vel, wheel_vel)

    def _drive_forward(self, robot, wheel_vel):
        """ Drives the robot forward.

        :type robot robot_inf.Robot
        :param robot:
            The robot's connection
        :type wheel_vel int
        :param wheel_vel:
            The absolute value of a wheel's velocity
        """
        robot.drive_direct(wheel_vel, wheel_vel)

    def _log_movement(self, stmt, prev_encoder):
        """ Creates a log statement for any movement.
        :param stmt:
            The statement type to log
        :param prev_encoder:
            The reference encoder values
        """
        if stmt == "Distance":
            _log_stmt(self._log,
                      str(self._sensor.get_distance(prev_encoder))+" mm")
        elif stmt.startswith("Angle"):
            _log_stmt(self._log,
                      str(self._sensor.get_angle(prev_encoder,
                                             cw=not stmt.endswith("CCW")))
                      + " deg")

        if self._log_unsafe:
                self._log_unsafe = False
                self._sensor.get_robot().play_warning_song()
                _log_stmt(self._log, "UNSAFE")


class RobotController(threading.Thread):
    """
        High level robot controller. This controller decides when to spawn and
        kill specific low level actuator controllers.
    """
    _stop = None

    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self._stop = False

    def stop(self):
        """
            Sets the flag to stop the thread to True. The thread will
            not immediately stop. Instead, the thread will exit safely.
        """
        self._stop = True

    def run(self):
        port_list = serial_inf.list_serial_ports()
        file_name = "robot_log.log"

        if len(port_list) > 1:
            print "Requires a serial connection."
            return -1

        if len(sys.argv) > 1:
            file_name = sys.argv[1]

        # The State space for the robot is the constants defined in
        # robot_inf.State (change to is thread running)

        robot = robot_inf.Robot(port_list[0])     # Serial connection to robot
        sensor = sensor_inf.Sensor(robot)         # Sensor synchronizer
        act_control = None                        # Low-level actuator control
        log = open(file_name, "w")

        if _prog_test:
            robot.change_state(robot_inf.State.SAFE)
        else:
            robot.change_state(robot_inf.State.FULL)

        print "Connected to robot"

        print "Listening for press"
        while not self._stop:
            # High-Level State Action
            if sensor.is_btn_pressed(robot_inf.Button.CLEAN):
                _log_stmt(log, "BUTTON")

                # Start actuator controller
                if act_control is None:
                    act_control = DriveControl(sensor, log)
                    act_control.start()

                # Stop actuator controller
                else:
                    act_control.stop()
                    act_control = None

            # Clocks while loop to the update rate of the iRobot Create 2.
            time.sleep(robot_inf.SENSOR_UPDATE_WAIT)

        print "Stopping Listening"

        # Stopping all threads, and closing log file
        robot.change_state(robot_inf.State.PASSIVE)
        if act_control is not None:
            act_control.stop()
        sensor.stop_update(join=True)
        log.close()


def main():
    """
        The main loop that spawns and controls all the threads.
    """
    control = RobotController()
    control.start()

    # Prompt to exit safely
    while raw_input("Type 'exit' to quit.") != "exit":
        time.sleep(robot_inf.SENSOR_UPDATE_WAIT)

    control.stop()
    # Wait for every thing to stop safely
    control.join()

if __name__ == '__main__':
    main()
