#!/usr/bin/python

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
prog_test = True


class DriveControl(threading.Thread):
    """
        :type _sensor sensor_inf.Sensor
        :type _stop bool
    """
    _sensor = None
    _stop = None

    def __init__(self, sensor):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self._stop = False
        self._sensor = sensor

    def run(self):
        robot = self._sensor.get_robot()

        while not self._stop:
            # Drive forward
            self._wait(robot_inf.SENSOR_UPDATE_WAIT,
                       robot_inf.SENSOR_UPDATE_WAIT)

        # Stops robot
        self._robot.drive_direct(0, 0)

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

        while time_left > 0:
            # Tell the caller it needs to stop
            if self._stop:
                return True

            # Wait another time interval
            interval_time = interval
            if interval_time > time_left:
                interval_time = time_left
            time_left -= interval_time
            time.sleep(interval_time)
        return False


def robot_controller():
    """

    """

    port_list = serial_inf.list_serial_ports()

    if len(port_list) > 1:
        print "Requires a serial connection."
        return -1

    # The State space for the robot is the constants defined in
    # robot_inf.State (change to is thread running)

    robot = robot_inf.Robot(port_list[0])     # Serial connection to robot
    sensor = sensor_inf.Sensor(robot)         # Sensor synchronizer
    act_control = None                        # Low-level actuator control

    if prog_test:
        robot.change_state(robot_inf.State.SAFE)
    else:
        robot.change_state(robot_inf.State.FULL)

    print "Connected to robot"

    print "Listening for press"
    while True:
        # High-Level State Action
        if sensor.is_btn_pressed(robot_inf.Button.CLEAN):

            # Start actuator controller
            if act_control is None:
                act_control = DriveControl(sensor)
                act_control.start()

            # Stop actuator controller
            else:
                act_control.stop()
                act_control = None

        # Clocks while loop to the update rate of the iRobot Create 2.
        time.sleep(robot_inf.SENSOR_UPDATE_WAIT)
    print "Stopping Listening"

if __name__ == '__main__':
    robot_controller()
