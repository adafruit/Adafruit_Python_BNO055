# Adafruit BNO055 WebGL Example
#
# Requires the flask web framework to be installed.  See http://flask.pocoo.org/
# for installation instructions, however on a Linux machine like the Raspberry
# Pi or BeagleBone black you can likely install it by running:
#  sudo apt-get update
#  sudo apt-get install python-pip
#  sudo pip install flask
#
# Copyright (c) 2015 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import json
import logging
import threading
import time

from flask import *

from Adafruit_BNO055 import BNO055


# Create and configure the BNO sensor connection.  Make sure only ONE of the
# below 'bno = ...' lines is uncommented:
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)
# BeagleBone Black configuration with default I2C connection (SCL=P9_19, SDA=P9_20),
# and RST connected to pin P9_12:
#bno = BNO055.BNO055(rst='P9_12')

# Application configuration below.  You probably don't need to change these values.

# How often to update the BNO sensor data (in hertz).
BNO_UPDATE_FREQUENCY_HZ = 10

# Name of the file to store calibration data when the save/load calibration
# button is pressed.  Calibration data is stored in JSON format.
CALIBRATION_FILE = 'calibration.json'

# BNO sensor axes remap values.  These are the parameters to the BNO.set_axis_remap
# function.  Don't change these without consulting section 3.4 of the datasheet.
# The default axes mapping below assumes the Adafruit BNO055 breakout is flat on
# a table with the row of SDA, SCL, GND, VIN, etc pins facing away from you.
BNO_AXIS_REMAP = { 'x': BNO055.AXIS_REMAP_X,
                   'y': BNO055.AXIS_REMAP_Z,
                   'z': BNO055.AXIS_REMAP_Y,
                   'x_sign': BNO055.AXIS_REMAP_POSITIVE,
                   'y_sign': BNO055.AXIS_REMAP_POSITIVE,
                   'z_sign': BNO055.AXIS_REMAP_NEGATIVE }


# Create flask application.
app = Flask(__name__)

# Global state to keep track of the latest readings from the BNO055 sensor.
# This will be accessed from multiple threads so care needs to be taken to
# protect access with a lock (or else inconsistent/partial results might be read).
# A condition object is used both as a lock for safe access across threads, and
# to notify threads that the BNO state has changed.
bno_data = {}
bno_changed = threading.Condition()

# Background thread to read BNO sensor data.  Will be created right before
# the first request is served (see start_bno_thread below).
bno_thread = None


def read_bno():
    """Function to read the BNO sensor and update the bno_data object with the
    latest BNO orientation, etc. state.  Must be run in its own thread because
    it will never return!
    """
    while True:
        # Grab new BNO sensor readings.
        temp = bno.read_temp()
        heading, roll, pitch = bno.read_euler()
        x, y, z, w = bno.read_quaternion()
        sys, gyro, accel, mag = bno.get_calibration_status()
        status, self_test, error = bno.get_system_status(run_self_test=False)
        if error != 0:
            print 'Error! Value: {0}'.format(error)
        # Capture the lock on the bno_changed condition so the bno_data shared
        # state can be updated.
        with bno_changed:
            bno_data['euler'] = (heading, roll, pitch)
            bno_data['temp'] = temp
            bno_data['quaternion'] = (x, y, z, w)
            bno_data['calibration'] = (sys, gyro, accel, mag)
            # Notify any waiting threads that the BNO state has been updated.
            bno_changed.notifyAll()
        # Sleep until the next reading.
        time.sleep(1.0/BNO_UPDATE_FREQUENCY_HZ)

def bno_sse():
    """Function to handle sending BNO055 sensor data to the client web browser
    using HTML5 server sent events (aka server push).  This is a generator function
    that flask will run in a thread and call to get new data that is pushed to
    the client web page.
    """
    # Loop forever waiting for a new BNO055 sensor reading and sending it to
    # the client.  Since this is a generator function the yield statement is
    # used to return a new result.
    while True:
        # Capture the bno_changed condition lock and then wait for it to notify
        # a new reading is available.
        with bno_changed:
            bno_changed.wait()
            # A new reading is available!  Grab the reading value and then give
            # up the lock.
            heading, roll, pitch = bno_data['euler']
            temp = bno_data['temp']
            x, y, z, w = bno_data['quaternion']
            sys, gyro, accel, mag = bno_data['calibration']
        # Send the data to the connected client in HTML5 server sent event format.
        data = {'heading': heading, 'roll': roll, 'pitch': pitch, 'temp': temp,
                'quatX': x, 'quatY': y, 'quatZ': z, 'quatW': w,
                'calSys': sys, 'calGyro': gyro, 'calAccel': accel, 'calMag': mag }
        yield 'data: {0}\n\n'.format(json.dumps(data))


@app.before_first_request
def start_bno_thread():
    # Start the BNO thread right before the first request is served.  This is
    # necessary because in debug mode flask will start multiple main threads so
    # this is the only spot to put code that can only run once after starting.
    # See this SO question for more context:
    #   http://stackoverflow.com/questions/24617795/starting-thread-while-running-flask-with-debug
    global bno_thread
    # Initialize BNO055 sensor.
    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055!')
    bno.set_axis_remap(**BNO_AXIS_REMAP)
    # Kick off BNO055 reading thread.
    bno_thread = threading.Thread(target=read_bno)
    bno_thread.daemon = True  # Don't let the BNO reading thread block exiting.
    bno_thread.start()

@app.route('/bno')
def bno_path():
    # Return SSE response and call bno_sse function to stream sensor data to
    # the webpage.
    return Response(bno_sse(), mimetype='text/event-stream')

@app.route('/save_calibration', methods=['POST'])
def save_calibration():
    # Save calibration data to disk.
    # First grab the lock on BNO sensor access to make sure nothing else is
    # writing to the sensor right now.
    with bno_changed:
        data = bno.get_calibration()
    # Write the calibration to disk.
    with open(CALIBRATION_FILE, 'w') as cal_file:
        json.dump(data, cal_file)
    return 'OK'

@app.route('/load_calibration', methods=['POST'])
def load_calibration():
    # Load calibration from disk.
    with open(CALIBRATION_FILE, 'r') as cal_file:
        data = json.load(cal_file)
    # Grab the lock on BNO sensor access to serial access to the sensor.
    with bno_changed:
        bno.set_calibration(data)
    return 'OK'

@app.route('/')
def root():
    return render_template('index.html')


if __name__ == '__main__':
    # Create a server listening for external connections on the default
    # port 5000.  Enable debug mode for better error messages and live
    # reloading of the server on changes.  Also make the server threaded
    # so multiple connections can be processed at once (very important
    # for using server sent events).
    app.run(host='0.0.0.0', debug=True, threaded=True)
