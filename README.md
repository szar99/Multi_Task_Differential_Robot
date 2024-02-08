# Multi-Task Differential Robot
The Multi-Task Differential Robot project focuses on developing a versatile robot equipped with multiple sensors to perform various tasks. Firstly, it can accurately follow a predefined path by utilizing a line following sensor. Secondly, it features obstacle avoidance functionality, leveraging infrared distance sensors to detect and steer clear of objects within its field of view. When encountering an obstacle, the robot adjusts its direction or maneuvers around it, especially in corner scenarios. Lastly, the robot incorporates object tracking capabilities facilitated by the Pixy2 camera. This enables the robot to track and pursue objects, such as a rolling ball on the ground, enhancing its adaptability and usefulness across different applications.

<!-- TODO picture of the robot -->
## Table of Contents
1. [Repository structure](#repository-structure)
2. [Hardware](#hardware)
3. [Prerequisites](#prerequisites)
4. [Kinematics](#kinematics)
5. [Functionalities](#functionalities)
    * [Line follower](#1-line-follower)
    * [Obstacle avoider](#2-obstacle-avoider)
    * [Pixy2 object follower](#3-pixy2-object-follower)
6. [Additions](#additions)
7. [Notes](#notes)



## Repository structure
The ``main`` file contains all three algorithms. The current algorithm is visible, the rest is commented out.

- ``/docs/cad`` - cad files with division into vehicle frame and gimbala construction for Pixy2. Inside there is a division into different extensions, including print-ready files. <br>
- ``/docs/documentation`` - folder contains all additional markdown files <br>
- ``/docs/images`` - folder contains all images used in markdown files <br>
- ``/docs/solutions`` - The folder contains the main files of three different algorithms. 

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

All frame parts 3D models are localized in [CAD folder](/docs/cad/)

<!-- TODO Add all the links -->
<details Closed>
<summary>Links to hardware</summary>

[Nucleo-F446RE][1] <br>
[78:1 Metal Gearmotor 20Dx43L mm 12V CB][2] <br>
[SparkFun Line Follower Array][3] <br>
[Conrad energy NiMH receiver battery packs 6V, 2300mAh][4] 

</details>
<br>

## Prerequisites
- Mbed Studio
- Libraries:
    - mbed-os 6.17.0
    - eigen
    - pm2_drivers

In order to run the project you need to have Mbed Studio with which you can compile the ``main`` program and run it on the microcontroller. All the libraries you need are included in the repository. If you use a different design, kinematics, or hardware, you need to change the appropriate variables for the program to run properly. Also, if you are not using a custom PES Board, you need to change pins names that are passed as parameters while objects are defined.

## Kinematics
Comprehensive explanation and description of the kinematics of a differential robot can be found [here](/docs/documentation/kinematics.md).

## Functionalities
As mentioned, the robot has three different functionalities listed below.
### 1. Line follower
The line follower functionality enables the robot to autonomously navigate predefined paths by continuously scanning for contrasting colors or reflective markers with a specialized sensor, making real-time adjustments based on feedback to ensure precise tracking and adherence to the designated route. Details of algorithm you can find [here](/docs/documentation/line_follower.md) <br>
Main file [**Line follower main**](/docs/solutions/Line_follower_main.txt)
### 2. Obstacle avoider
The obstacle avoidance functionality equips the robot with infrared distance sensors to detect objects within its vicinity. When an obstacle is detected, the robot swiftly adjusts its course to avoid collision, either by changing direction or maneuvering around the obstacle. This feature ensures the robot's safe navigation in dynamic environments, preventing potential collisions and ensuring smooth operation. Details of algorithm you can find [here](/docs/documentation/obstacle_avoider.md) <br>
Main file [**Obstacle avoider**](/docs/solutions/Line_follower_main.txt)
### 3. Pixy2 object follower
The Pixy2 object follower functionality allows the robot to seamlessly track and follow designated objects within its view. By leveraging the capabilities of the Pixy2 camera, the robot identifies and locks onto the target object, ensuring precise tracking as it moves. This enables the robot to autonomously follow the object, adjusting its trajectory accordingly for smooth and accurate pursuit. Details of algorithm you can find [here](/docs/documentation/pixy_follower.md) <br>
Main file [**Pixy2 object follower**](/docs/solutions/Line_follower_main.txt)


## Additions
- The files include a map that should be printed on a large scale for the algorithm to run properly: [MAP](docs/Highspeed_2.pdf) 

## Notes:
- Update dependencies after releasing drivers! 