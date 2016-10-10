#!/usr/bin/python

import sys
import glob
import serial

# =============================================================================
#                       Serial Communication Interface
# =============================================================================

class SerialConn:
    """
        Acts as the interface for a Serial connection
    """
    _DEFAULT_TIMEOUT = 1
    _serial_conn = None

    def __init__(self, port, buad, timeout=_DEFAULT_TIMEOUT):
        """ Initializes a serial connection by creating a connection if the port
            is not equal to None. The connection is established using the
            connect() method.

        :param port:
            The name of the serial port. A serial ports name can be gathered
            from the function list_serial_ports()
        :param buad:
            The buad rate for this connection.
        :type timeout float:
        :param timeout:
            The read timeout in seconds
        """
        if port != "":
            self.connect(port, buad, timeout)

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
            Ensures the serial connection is closed.
        """
        self.close()

    def connect(self, port, buad, timeout=_DEFAULT_TIMEOUT):
        """ Creates a serial connection with a specified buad rate and timeout.

        :param port:
            The name of the serial port. A serial ports name can be gathered
            from the function list_serial_ports()
        :param buad:
            The buad rate for this connection.
        :type timeout float:
        :param timeout:
            The read timeout in seconds
        """
        self._serial_conn = serial.Serial(port, baudrate=buad, timeout=timeout)

    def close(self):
        """
            Closes an opened serial connection.
        """
        self._serial_conn.close()

    def send_command(self, cmd):
        """ Encodes the command and then sends the command to the provided
            serial connection.

        :param cmd:
            The command to send. The command should be character code
            delimited by spaces. Example: '138 0'
        :return:
            The number of bytes written to the serial connection.
        """
        # This encodes the delimited command.
        encode_cmd = ""
        for code in cmd.split(" "):
            encode_cmd += chr(int(code))
        # Sends the encoded command.
        return self._serial_conn.write(encode_cmd)

    def read_data(self, bytes):
        """ Reads data on the provided serial connection. If a timeout is set on
            the serial connection, then this may read less than requested.
            However, if no timeout is set, then it will block until the
            requested number of bytes have been read.

        :type bytes int:
        :param bytes:
            The number of bytes to read from the serial connection.
        :return:
            The bytes read from the serial connection.
        """
        return self._serial_conn.read(bytes)

# =============================================================================
#                       Serial Communication Helpers
# =============================================================================

def list_serial_ports():
    """ Creates a list of all connected serial ports on the system. This
        function is only aware of serial port on Windows and Linux systems.

        :raises EnvironmentError:
            Whenever an unsupported platform uses this function.
        :return:
            A list of all available serial ports on the system.
    """

    # Creates a list of all possible serial ports on the system.
    if sys.platform.startswith("win"):
        ports = ["COM%s" % (i+1) for i in range(256)]
    elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
        ports = glob.glob("/dev/tty[A-Za-z]*")
    else:
        raise EnvironmentError("Unsupported Platform")

    # Filters the list of possible serial ports to connectable serial ports.
    result = []
    for port in ports:
        try:
            # Checks if the port is connectable.
            ser = serial.Serial(port)
            ser.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result