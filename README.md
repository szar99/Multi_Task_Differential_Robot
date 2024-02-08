# Multi-Task Differential Robot
The Multi-Task Differential Robot project focuses on developing a versatile robot equipped with multiple sensors to perform various tasks. Firstly, it can accurately follow a predefined path by utilizing a line following sensor. Secondly, it features obstacle avoidance functionality, leveraging infrared distance sensors to detect and steer clear of objects within its field of view. When encountering an obstacle, the robot adjusts its direction or maneuvers around it, especially in corner scenarios. Lastly, the robot incorporates object tracking capabilities facilitated by the Pixy2 camera. This enables the robot to track and pursue objects, such as a rolling ball on the ground, enhancing its adaptability and usefulness across different applications.

## Table of Contents


## Repository structure


## Hardware
Components and sensors employed in the design:
- Nucleo-F446RE with custom PES board
- 2 x 31:1 Metal Gearmotor 20Dx43L mm 12V CB
- SparkFun Line Follower Array
- 4 x IR distance sensors SHARP 2Y0A21 
- Pixy2 cam
- 2 x Towerpro MG90S servos 
- 2 x Conrad energy NiMH receiver battery packs 6V, 2300mAh
- Jumper wires

<!-- TODO Add link to this folder -->
All frame parts 3D models are localized in CAD folder.

<!-- TODO Add all the link -->
<details Closed>
<summary>Links to hardware</summary>

[Nucleo-F446RE][1] <br>
[78:1 Metal Gearmotor 20Dx43L mm 12V CB][2] <br>
[SparkFun Line Follower Array][3] <br>
[Conrad energy NiMH receiver battery packs 6V, 2300mAh][4] 

</details>

## Prerequisites
- Mbed Studio
- Libraries:
    - mbed-os 6.17.0
    - eigen
    - pm2_drivers

In order to run the project you need to have Mbed Studio with which you can compile the ``main`` program and run it on the microcontroller. All the libraries you need are included in the repository. If you use a different design, kinematics, or hardware, you need to change the appropriate variables for the program to run properly. Also, if you are not using a custom PES Board, you need to change pins names that are passed as parameters while objects are defined.

## Kinematics
Comprehensive explanation and description of the kinematics of a differential robot can be found here. <!-- TODO add hyperlinks  -->

## Functionalities
As mentioned, the robot has three different functionalities listed below.
<!-- TODO add hyperlinks  -->
### 1. Line follower
The line follower functionality enables the robot to autonomously navigate along predefined paths by detecting and following lines on the ground. Using a specialized line following sensor, the robot continuously scans the surface for contrasting colors or reflective markers that define the desired path. Real-time adjustments in the robot's movements are made based on the sensor's feedback, ensuring precise tracking and adherence to the designated route. Details of working algorithm you can find HERE 
### 2. Obstacle avoider
The obstacle avoidance functionality equips the robot with infrared distance sensors to detect objects within its vicinity. When an obstacle is detected, the robot swiftly adjusts its course to avoid collision, either by changing direction or maneuvering around the obstacle. This feature ensures the robot's safe navigation in dynamic environments, preventing potential collisions and ensuring smooth operation. Details of working algorithm you can find HERE 
### 3. Pixy2 object follower
The Pixy2 object follower functionality allows the robot to seamlessly track and follow designated objects within its view. By leveraging the capabilities of the Pixy2 camera, the robot identifies and locks onto the target object, ensuring precise tracking as it moves. This enables the robot to autonomously follow the object, adjusting its trajectory accordingly for smooth and accurate pursuit. Details of working algorithm you can find HERE 

## Comments
- The files include a map that should be printed on a large scale for the algorithm to run properly: [MAP](docs/Highspeed_2.pdf)
- 

## Notes:
- Update dependencies after releasing drivers! 