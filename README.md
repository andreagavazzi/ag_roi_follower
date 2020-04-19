# ag_roi_tracker
Pan and Tilt roi follower for 2 Dynamixel actuators

The node subscribes a /roi topic that brings a sensor_msgs/RegionOfInterest.msg:  

| Data  | Description | Content |
| ------------- | ------------- | ------------- |
| x_offset  | Leftmost pixel of the ROI  | uint32  |
| y_offset  | Topmost pixel of the ROI | uint32  |
| height | Height of ROI  | uint32  |
| width   | Width of ROI  | uint32  |



<img src="https://github.com/andreagavazzi/ag_roi_follower/tree/master/assets/pan_tilt.PNG" alt="Your image title" width="500"/>

The difference with all the other pan&tilt subscribers is that my code is actually using the wheel mode of the Dynamixel motors.
This is allowing smoother movements and quite a lean code. Perfect when used on a 2 dof head.




___
![alt text](https://gavazzionline.files.wordpress.com/2014/01/img_6916.jpg?w=200)
