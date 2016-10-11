#!/usr/bin/python

import threading
import time

import robot_inf


# =============================================================================
#                       Internal Sensor Updater
# =============================================================================


class _SensorUpdate(threading.Thread):
    """
        Updates the internal sensor values stored within an instance of Sensor.
    """

    _sensor = None          # Instance of Sensor
    _sensor_sema = None     # The semaphor used to control read/write access
    _interval = None        # The interval between updates
    _stop = False           # The stopping condition

    def __init__(self, sensor, semaphor, interval=robot_inf.SENSOR_UPDATE_WAIT):
        """ Initializes an instance of SensorUpdate with a specified sensor,
            and semaphor.

        :type sensor Sensor:
        :param sensor:
            The instance of Sensor to update.
        :type semaphor Semaphor:
        :param semaphor:
            The semaphor that is used to control read and write access.
        :param interval:
            The interval between sensor updates
        """
        threading.Thread.__init__(self)
        self._sensor = sensor
        self._sensor_sema = semaphor
        self._interval = interval

    def run(self):
        robot = self._sensor.get_robot()

        while not self._stop:
            self._sensor_sema.acquire()         # Acquire Lock

            # Update Buttons
            self._sensor.request_sources.clear()
            self._sensor.btn_prev = self._sensor.btn_down.copy()
            self._sensor.btn_down.clear()

            btns = robot.read_buttons()

            for btn, value in btns.iteritems():
                if value:
                    self._sensor.btn_down[btn] = value

            # Update bump and wheel drops
            self._sensor.bumps = robot.read_bumps()
            self._sensor.wheels = robot.read_wheel_drops()

            # Update cliff sensors
            self._sensor.cliffs = robot.read_cliffs()

            # Update encoders
            self._sensor.encoders = robot.read_encoders()

            self._sensor_sema.release()         # Release Lock

            time.sleep(self._interval)

    def stop(self):
        """
            Flags this instance of SensorUpdate to stop. The instance will not
            immediately exit. Instead it will try to exit safely.
        """
        self._stop = True

# =============================================================================
#                             Sensor Interface
# =============================================================================


class Sensor:
    """
        Interface to help manage and interpret sensor data.

        This interface will automatically update the internal sensor value at
        the provided interval. It will also help determine when a button is
        initially pressed or released.

        This interface also provides helpers to safely access the public data
        members. Please uses there methods to access any sensor data member
        outside of its accompanying module.

        Attributes:
            btn_prev: A dictionary of the buttons down on the previous cycle.
            btn_down: A dictionary of the buttons down for the current cycle.
            request_sources: A dictionary of all request for the current cycle.
            bumps: A dictionary of all bump sensors
            wheels: A dictionary of all wheel drop sensors
            cliffs: A dictionary of all cliff sensors
            encoders: A dictionary of all encoders

        :type _robot robot_inf.Robot
        :type _sensor_sema threading.Semaphor
        :type _sensor_update _SensorUpdate
        :type _interval float
    """
    _robot = None
    _sensor_sema = None
    _sensor_update = None
    _interval = robot_inf.SENSOR_UPDATE_WAIT

    btn_prev = {}
    btn_down = {}
    request_sources = {}
    bumps = {}
    wheels = {}
    cliffs = {}
    encoders = {}

    def __init__(self, robot, interval=robot_inf.SENSOR_UPDATE_WAIT):
        """ Creates an instance of the sensor interface. This will
            automatically spawn a thread to update the internal sensor data.

        :type robot Robot:
        :param robot:
            The robot to pull the sensor data from
        :param interval:
            The interval between sensor updates
        """
        self._robot = robot
        self._sensor_sema = threading.Semaphore()
        self._interval = interval
        self.start_update()

    # -------------------------------------------------------------------- #
    # -                      General Class Methods                       - #
    # -------------------------------------------------------------------- #

    def start_update(self):
        """ Spawns a new thread to update the internal sensor data only if
            there is no thread is currently updating its sensor data.
        """
        if self._sensor_update is None:
            self._sensor_update = _SensorUpdate(self,
                                                self._sensor_sema,
                                                self._interval)
            self._sensor_update.setDaemon(True)
            self._sensor_update.start()

    def stop_update(self):
        """ Stops the thread currently updating the internal sensor data.
        """
        if self._sensor_update is not None:
            self._sensor_update.stop()
            self._sensor_update = None

    def get_robot(self):
        """ Gets the robot that provides the sensor data.
        :return:
            The robot providing the sensor data.
        """
        return self._robot

    # -------------------------------------------------------------------- #
    # -                       Read Sensor Methods                        - #
    # -------------------------------------------------------------------- #

    def is_btn_pressed(self, btn, src="Default", override=False):
        """ Determines if the specified button has been pressed.

            Without overriding this should only be called once per cycle.
        :param btn:
            The button to check
        :type src str:
        :param src:
            The requesting source
        :param override:
            Flag to override the check for a repeated poll of the same sensor
            data.
        :return:
            True if the button is pressed.
        """
        enc_function = "BtnPress"+str(btn)

        self._sensor_sema.acquire()             # Acquire Lock

        # rtn = not down on previous cycle and is down on current cycle
        value = btn not in self.btn_prev and btn in self.btn_down
        rtn = self._check_return(value, False, enc_function, src, override)

        self._sensor_sema.release()             # Release Lock

        return rtn

    def is_btn_released(self, btn, src="Default", override=False):
        """ Determines if the specified button has been released.

            Without overriding this should only be called once per cycle.
        :param btn:
            The button to check
        :type src str:
        :param src:
            The requesting source
        :param override:
            Flag to override the check for a repeated poll of the same sensor
            data.
        :return:
            True if the button is released.
        """
        enc_function = "BtnRelease"+str(btn)

        self._sensor_sema.acquire()             # Acquire Lock

        # rtn = down on previous cycle and is not down on current cycle
        value = btn in self.btn_prev and btn not in self.btn_down
        rtn = self._check_return(value, False, enc_function, src, override)

        self._sensor_sema.release()             # Release Lock

        return rtn

    def is_btn_down(self, btn, src="Default", override=False):
        """ Determines if the specified button is currently down.

            Without overriding this should only be called once per cycle.
        :param btn:
            The button to check
        :type src str:
        :param src:
            The requesting source
        :param override:
            Flag to override the check for a repeated poll of the same sensor
            data.
        :return:
            True if the button is down.
        """
        enc_function = "BtnDown"+str(btn)

        self._sensor_sema.acquire()             # Acquire Lock

        value = btn in self.btn_down
        rtn = self._check_return(value, False, enc_function, src, override)

        self._sensor_sema.release()             # Release Lock

        return rtn

    def is_bump(self, bump, src="Default", override=False):
        """ Determines if the specified bump is currently bumped.

            Without overriding this should only be called once per cycle.
        :param bump:
            The bump to check
        :type src str:
        :param src:
            The requesting source
        :param override:
            Flag to override the check for a repeated poll of the same sensor
            data.
        :return:
            True if the bump is bumped.
        """
        enc_function = "Bump"+str(bump)

        self._sensor_sema.acquire()             # Acquire Lock

        value = self.bump_wheel[bump]
        rtn = self._check_return(value, False, enc_function, src, override)

        self._sensor_sema.release()             # Release Lock

        return rtn

    def get_bumps(self, src="Default", override=False):
        """ Gets all the bump sensor values.

            Without overriding this should only be called once per cycle.
        :type src str:
        :param src:
            The requesting source
        :param override:
            Flag to override the check for a repeated poll of the same sensor
            data.
        :return:
            A dictionary with each bump sensors' value.
            See robot_inf.Robot.read_bumps() for entry referencing.
        """
        enc_function = "Bumps"

        self._sensor_sema.acquire()             # Acquire Lock

        value = self.bump_wheel.copy()
        rtn = self._check_return(value, {}, enc_function, src, override)

        self._sensor_sema.release()             # Release Lock

        return rtn

    def is_wheel_dropped(self, wheel_drop, src="Default", override=False):
        """ Determines if the specified wheel drop is currently dropped.

            Without overriding this should only be called once per cycle.
        :param wheel_drop:
            The wheel drop to check
        :type src str:
        :param src:
            The requesting source
        :param override:
            Flag to override the check for a repeated poll of the same sensor
            data.
        :return:
            True if the wheel drop is dropped.
        """
        enc_function = "WheelDrop"

        self._sensor_sema.acquire()             # Acquire Lock

        value = self._robot.read_wheel_drop(wheel_drop)
        rtn = self._check_return(value, False, enc_function, src, override)

        self._sensor_sema.release()             # Release Lock

        return rtn

    def get_wheel_drops(self, src="Default", override=False):
        """ Gets all the wheel drop sensor values.

            Without overriding this should only be called once per cycle.
        :type src str:
        :param src:
            The requesting source
        :param override:
            Flag to override the check for a repeated poll of the same sensor
            data.
        :return:
            A dictionary with each wheel drop sensors' value.
            See robot_inf.Robot.read_wheel_drops() for entry referencing.
        """
        enc_function = "WheelDrops"

        self._sensor_sema.acquire()             # Acquire Lock

        value = self.wheels.copy()
        rtn = self._check_return(value, {}, enc_function, src, override)

        self._sensor_sema.release()             # Release Lock

        return rtn

    def is_cliff(self, cliff, src="Default", override=False):
        """ Determines if there is a cliff or virtual wall is seen on the
            specified side.

            Without overriding this should only be called once per cycle.
        :param cliff:
            The cliff or virtual wall to check
        :type src str:
        :param src:
            The requesting source
        :param override:
            Flag to override the check for a repeated poll of the same sensor
            data.
        :return:
            True if a cliff or virtual wall is present.
        """
        enc_function = "Cliff"

        self._sensor_sema.acquire()             # Acquire Lock

        value = self.cliffs[cliff]
        rtn = self._check_return(value, False, enc_function, src, override)

        self._sensor_sema.release()             # Release Lock

        return rtn

    def get_cliffs(self, src="Default", override=False):
        """ Gets all cliff and virtual wall sensor data.

            Without overriding this should only be called once per cycle.
        :type src str:
        :param src:
            The requesting source
        :param override:
            Flag to override the check for a repeated poll of the same sensor
            data.
        :return:
            A dictionary with each cliff and virtual wall sensors value.
            See robot_inf.Robot.read_cliffs() for entry referencing.
        """
        enc_function = "Cliffs"

        self._sensor_sema.acquire()             # Acquire Lock

        value = self.cliffs.copy()
        rtn = self._check_return(value, {}, enc_function, src, override)

        self._sensor_sema.release()             # Release Lock

        return rtn

    def get_encoders(self, src="Default", override=False):
        """ Copies the current encoder values.

            Without overriding this should only be called once per cycle.
        :type src str:
        :param src:
            The requesting source
        :param override:
            Flag to override the check for a repeated poll of the same sensor
            data.
        :return:
            A shallow copy of the current encoder values.
        """
        enc_function = "Encoders"

        self._sensor_sema.acquire()             # Acquire Lock

        value = self.encoders.copy()
        rtn = self._check_return(value, {}, enc_function, src, override)

        self._sensor_sema.release()             # Release Lock

        return rtn

    def get_distance(self, ref_dist, forward=True):
        """ Gets the distance between two points while ensuring thread-safe
            access.

        :param ref_dist:
            The reference encoder values
        :param forward:
            Flag used to determine turnover.
        :return:
            The distance between two encoder values
        """
        self._sensor_sema.acquire()             # Acquire Lock
        dist = self._robot.distance(ref_dist, forward=forward)
        self._sensor_sema.release()             # Release Lock

        return dist

    def get_angle(self, ref_angle, radians=False, cw=True):
        """ Gets the angle between two points while ensuring thread-safe
            access.

        :param ref_angle:
            The reference encoder values
        :param cw:
            Flag used to determine turnover.
        :param radians:
            Flag used to determine the units
        :return:
            The angle between two encoder values
        """
        self._sensor_sema.acquire()             # Acquire Lock
        dist = self._robot.angle(ref_angle, radians=radians, cw=cw)
        self._sensor_sema.release()             # Release Lock

        return dist

    # -------------------------------------------------------------------- #
    # -                         Helper Methods                           - #
    # -------------------------------------------------------------------- #

    def _add_function_key(self, enc_function):
        """ Adds a key to the request_sources if it is not already there.
            The key's value will be initialized to an empty dictionary.

        :param enc_function:
            The string representing a particular function.
        """
        if enc_function not in self.request_sources:
            self.request_sources[enc_function] = {}

    def _check_return(self, value, default, funct, src, override):
        """ Check against the request sources to see if the value
            should be returned.

        :param value:
            The current cycle's value
        :param default:
            The default value
        :param funct:
            The encoded function name
        :param src:
            The requesting source
        :param override:
            Flag to override requesting sources value
        :return:
            The value if first request or overridden.
        """
        self._add_function_key(funct)

        if override or src not in self.request_sources[funct]:
            self.request_sources[funct][src] = True
            return value
        else:
            return default