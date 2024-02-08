# Line follower
The line follower functionality enables the robot to autonomously navigate along predefined paths by detecting and following lines on the ground. Using a specialized line following sensor, the robot continuously scans the surface for contrasting colors or reflective markers that define the desired path. Real-time adjustments in the robot's movements are made based on the sensor's feedback, ensuring precise tracking and adherence to the designated route.

## Hardware
To use this functionality [Sparkfun Line follower sensor array][1] is used.
<!-- TODO add picture -->

The sensor incorporates eight diodes for line detection, with each diode's illumination indicating the presence of a line beneath it. Additionally, a calibration knob enables users to adjust detection sensitivity, accommodating variations in substrate and line color shades. More information is available [here][1]

## Software
The following is a description of the software used to perform the task in the form of line tracking.
### Main


1. Na jakiej zasadzie to działa - tutaj mamy driver - opisać z poziomu main co i jak jak to działa a potem opisać driver
2. Opisać jaki tu mamy state machine i co można w niej zmienić


2. szczegóły drivera
    1. co można zmieniać i ustawiać (opiać zmienne)
    2. jak działa algorytm wewnątrz
    3. funkcje determinujące wyznaczanie prędkości






<!-- Links: -->
[1]: https://www.sparkfun.com/products/13582
[2]: https://learn.sparkfun.com/tutorials/sparkfun-line-follower-array-hookup-guide