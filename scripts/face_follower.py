#!/usr/bin/env python

# Sottoscrive un topic /roi con messaggio RegionOfInterest e muove un dispositivo Pan&Tilt

import rospy
from opencv_apps.msg import FaceArrayStamped
import time
import pypot.dynamixel

rospy.init_node('face_follower')

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
speed = 60  # base = 120
pan = 10
tilt = 11
pan_min = -90
pan_max = 90
tilt_min = -45
tilt_max = 30
cam_width = 800
cam_height = 600
tolerance = 80


def zero():
    dxl_io.disable_torque([10, 11])
    dxl_io.set_joint_mode([10, 11])
    dxl_io.enable_torque([10, 11])
    dxl_io.set_moving_speed({10: 200, 11: 200})
    dxl_io.set_goal_position({10: 0, 11: 0})
    time.sleep(1)
    dxl_io.disable_torque([10, 11])
    dxl_io.set_wheel_mode([10, 11])
    dxl_io.enable_torque([10, 11])
    stop(10)
    stop(11)


def rotate_pan_plus():       # ruota in direzione + (CCW)
    print 'pan plus'
    dxl_io.set_moving_speed({pan: 1 * speed})


def rotate_pan_neg():       # ruota in direzione - (CW)
    print 'pan minus'
    dxl_io.set_moving_speed({pan: -1 * speed})


def rotate_tilt_plus():       # ruota in direzione + (CCW)
    if not dxl_io.get_present_position((10, 11))[1] > tilt_max:
        dxl_io.set_moving_speed({tilt: 1 * speed})
    else:
        stop(tilt)


def rotate_tilt_neg():       # ruota in direzione - (CW)
    if not dxl_io.get_present_position((10, 11))[1] < tilt_min:
        dxl_io.set_moving_speed({tilt: -1 * speed})
    else:
        stop(tilt)


# TODO: funzione di ricerca, pan e tilt si muovono
def look_for_roi():
    pass


def stop(dyn):
    dxl_io.set_moving_speed({dyn: 0})


def callback(data):
    roi = data     # recupera i dati dal subscriber
    if len(roi.faces) > 0:
        roi_x_centre = roi.faces[0].face.x       # centro del roi x
        roi_y_centre = roi.faces[0].face.y      # centro del roi y
        # print roi_x_centre, roi_y_centre
    else:
        roi_x_centre = roi_y_centre = 0
        # print roi_x_centre, roi_y_centre
# TODO: se il roi non cambia allora torna a zero dopo tot secondi
# TODO: invece di stopparsi quando il roi vale zero prosegue per poco la direzione che aveva ?
# TODO: zona tolleranza proporzionale alla larghezza del roi ?

# Movimento pan
    if roi_x_centre > 0:
        # print dxl_io.get_present_position((10, 11))[0]
        # ruota pan se il roi si trova fuori dalla zona di tolleranza
        if roi_x_centre > cam_width/2 + tolerance and dxl_io.get_present_position((10, 11))[0] > pan_min:
            rotate_pan_neg()
        elif roi_x_centre < cam_width/2 - tolerance and dxl_io.get_present_position((10, 11))[0] < pan_max:
            rotate_pan_plus()
        else:
            stop(pan)
    else:
        stop(pan)

# Movimento tilt
    if roi_y_centre > 0:
        print dxl_io.get_present_position((10, 11))[1]
        # ruota tilt se il roi si trova fuori dalla zona di tolleranza
        if roi_y_centre > cam_height/2 + tolerance and dxl_io.get_present_position((10, 11))[1] > tilt_min:
            rotate_tilt_plus()
        elif roi_y_centre < cam_height/2 - tolerance and dxl_io.get_present_position((10, 11))[1] < tilt_max:
            rotate_tilt_neg()
        else:
            stop(tilt)
    else:
        stop(tilt)

# Debug
# rospy.loginfo(str(dxl_io.get_present_position((1, 2))))


def main():
    zero()
    while not rospy.is_shutdown():
        rospy.Subscriber("/face_detection/faces", FaceArrayStamped, callback, queue_size=1)
        rospy.spin()
    rospy.on_shutdown(shutdown)


def shutdown():
    print " Shutting down..."
    zero()
    dxl_io.disable_torque([10, 11])
    dxl_io.close()
    print "Terminated"


# Main routine
if __name__ == '__main__':
    main()
