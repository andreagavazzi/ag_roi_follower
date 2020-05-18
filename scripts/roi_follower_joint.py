#!/usr/bin/env python

# Sottoscrive un topic /roi con messaggio RegionOfInterest e muove un dispositivo Pan&Tilt

import rospy
from sensor_msgs.msg import RegionOfInterest
import time
import pypot.dynamixel

rospy.init_node('roi_follower')

# Connessione e inizializzazione pypot
ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

port = ports[0]
rospy.loginfo('Connecting to the first available port: ' + str(port))

dxl_io = pypot.dynamixel.DxlIO(port)
ids = dxl_io.scan([10, 11])
rospy.loginfo('Connecting to Dinamixel id: ' + str(ids))
rospy.loginfo('Ready to go!')


# Parametri
speed = 150  # base (wheel mode) = 120
pan = 10
tilt = 11
pan_min = -75
pan_max = 75
tilt_min = -45
tilt_max = 30
cam_width = 800
cam_height = 600
tolerance = 80


def zero():
    dxl_io.set_moving_speed({pan: speed, tilt: speed})
    dxl_io.set_goal_position({pan: 0, tilt: 0})
    time.sleep(1)


def stop(dyn):
    dxl_io.set_moving_speed({dyn: 0})


def callback(data):
    roi = data     # recupera il roi dal subscriber
    roi_x_centre = roi.width/2 + roi.x_offset       # centro del roi x
    roi_y_centre = roi.height/2 + roi.y_offset      # centro del roi y

    # movimento pan
    if roi_x_centre > cam_width/2 + tolerance:
        current = dxl_io.get_present_position((10, 11))[0]
        dxl_io.set_goal_position({pan: current - 8})
    elif roi_x_centre < cam_width/2 - tolerance:
        current = dxl_io.get_present_position((10, 11))[0]
        dxl_io.set_goal_position({pan: current + 8})


def main():
    zero()
    while not rospy.is_shutdown():
        rospy.Subscriber("/roi", RegionOfInterest, callback, queue_size=1)
        rospy.spin()
    rospy.on_shutdown(shutdown)


def shutdown():
    print " Shutting down..."
    zero()
    dxl_io.close()
    print "Terminated"


# Main routine
if __name__ == '__main__':
    main()
