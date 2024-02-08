# Differential robot kinematics

The kinematics of a differential drive robot describe the relationship between the robot's motion and the control inputs applied to its wheels. This typically involves equations that relate the linear and angular velocities of the robot to the velocities of its individual wheels. The kinematic model accounts for factors such as wheel radius, wheelbase (distance between the wheels), and the differential drive configuration to accurately predict the robot's trajectory based on the control commands given to its wheels.

## Problem overiew
As mentioned above, the goal is to be able to get the right trajectory for the robot's movement. Our goal is to properly calculate the velocity values of the motors that drive the left and right wheels to get the desired motion. The input data in this case are:
- Wheel radius $ r $
- Wheel base $ L $
- Linear velocity $ v $ 
- Rotational velocity with respect to robot's Z axis $\omega$ 

The output values in this case are:
- left_wheel_velocity (rotational) $\omega_1$ 
- right_wheel_velocity (rotational) $\omega_2$ 

**NOTE:** For these calculations, we'll leverage the Eigen library, renowned for its ability to define matrices and vectors, streamlining the computation process.

## Math


The computational process should start by defining geometric relationships and begin with simple formulas that will allow us to create transformation matrices from the wheels to the center of the robot, that is, from their wheel speeds to linear and rotational speeds.

### Linear velocity
The linear speed of the robot is calculated as the average value of the speed of the right and left wheels.

$\displaystyle v = \frac{v_1 + v_2}{2} $

The linear speed of a wheel is equal to the product of the rotational speed and the radius of the wheel.

$\displaystyle v_1 = \omega_1 * r  $

Therefore, we can write down:

$\displaystyle v = \frac{\omega_1 * r + \omega_2 * r}{2} $

We break this into two fractions:




### Rotational velocity with respect to robot's Z axis


