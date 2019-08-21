# Hand_gesture_quadchopter

### Project description:-
   Build hand gesture controlled quadcopter. Arduino, MPU-6050 (gyro+accelerometer) IMU sensor, NRF module, ultrasonic sensors were mainly used components in the project. MPU-6050 was used to detect hand gesture to control quadcopter’s roll, pitch, and yaw. The potentiometer was used for throttle.  NRF module ware used for wireless communication.  

### Hand_gesture_quadchopter:-
![alt text](https://github.com/ankitgc1/Hand_gesture_quadchopter/blob/master/pics/Hand_gesture_quad_chopter.jpg)

### Video:-

[![Watch the video](https://github.com/ankitgc1/Hand_gesture_quadchopter/blob/master/pics/drone.png)](https://youtu.be/Ep3qxZhunCg)

### Required things :- 
   you can find all the components's [pics](https://github.com/ankitgc1/Hand_gesture_quadchopter/tree/master/pics) directory.
##### For Transmitter part:-
1. Arduino mini/micro
2. NRF module 
3. Potensiometer
4. MPU6050(IMU sensor)
5. Push button
6. 9V battery
7. Glove
8. 2 LEDs

##### For Receiver part:-
1. Drone frame
2. CC3D flight controller
3. 4 BLDC motors
4. 4 ESCs
5. 4 Propellers
6. NRF module
7. Arduino mini/micro
8. Ultrasonic sensor
9. Lipo battery
10. Power distributer


### Connections:- 

#### Transmitter connections:-
![alt text](https://github.com/ankitgc1/Hand_gesture_quadchopter/blob/master/pics/transmitter_connections-1.jpg)

#### Receiver connections:- 
![alt text](https://github.com/ankitgc1/Hand_gesture_quadchopter/blob/master/pics/receiver_connections-1.jpg)

### Last processing
First, Finish your drone assembly. Here is one good video on drone assembly. You can follow this link.

[![Watch the video)](https://www.youtube.com/watch?v=ZK7IkctChu8)

After Finishing the drone assembly. Just finish all your connections.
In "[nrf_gy_trans](https://github.com/ankitgc1/Hand_gesture_quadchopter/tree/master/nrf_gy_trans)" file has transmitter code, and "[nrf_recv](https://github.com/ankitgc1/Hand_gesture_quadchopter/tree/master/nrf_recv)" file has transmitter code. The codes are pretty self-explaining. Upload the respective codes in their Arduinos.

