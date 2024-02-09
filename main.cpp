#include <mbed.h>
#include <math.h>

#include "pm2_drivers/DebounceIn.h"
#include "pm2_drivers/Servo.h"
#include "pm2_drivers/EncoderCounter.h"
#include "pm2_drivers/DCMotor.h"
#include "pm2_drivers/UltrasonicSensor.h"
#include "pm2_drivers/IMU.h"
#include "pm2_drivers/SensorBar.h"
#include "pm2_drivers/PESBoardPinMap.h"
#include "eigen/Dense.h"
#include "pm2_drivers/LineFollower.h"
#include "pm2_drivers/PixyCam2.h"


#define M_PI 3.14159265358979323846  // number pi


// logical variable main task
bool do_execute_main_task = false;

// user button on nucleo board
Timer user_button_timer;            
DebounceIn user_button(USER_BUTTON);
// Function that triggers main task execution   
void user_button_pressed_fcn();

PixyCam2 pixy(PC_10, PC_11, 230400);

// Controling functions
float pixy_following_x_fcn(const float& x, const float& x_threshold, const float& Kx);
float pixy_following_y_fcn(const float& y, const float& y_t, const float& Ky, const float& y_boundary);
float vel_cntrl_v2_fcn(const float& wheel_speed_max, const float& b, const float& robot_omega, const Eigen::Matrix2f& Cwheel2robot);


int main()
{
    enum RobotState {
        INITIAL,
        FOLLOW,
        SLEEP
    } robot_state = RobotState::INITIAL;

    // attach button fall function to user button object
    user_button.fall(&user_button_pressed_fcn);

    // while loop gets executed every main_task_period_ms milliseconds
    const int main_task_period_ms = 20;   // define main task period time in ms
    Timer main_task_timer;  

    // led on nucleo board
    DigitalOut user_led(USER_LED); 

    // Robot kinematics
    const float r_wheel = 0.0563f / 2.0f; // wheel radius
    const float L_wheel = 0.13f;         // distance from wheel to wheel
    Eigen::Matrix2f Cwheel2robot; // transform wheel to robot
    //Eigen::Matrix2f Crobot2wheel; // transform robot to wheel
    Cwheel2robot <<  r_wheel / 2.0f   ,  r_wheel / 2.0f   ,
                     r_wheel / L_wheel, -r_wheel / L_wheel;
    //Crobot2wheel << 1.0f / r_wheel,  L_wheel / (2.0f * r_wheel),
    //                1.0f / r_wheel, -L_wheel / (2.0f * r_wheel);
    Eigen::Vector2f robot_coord;  // contains v and w (robot translational and rotational velocities)
    Eigen::Vector2f wheel_speed;  // w1 w2 (wheel speed)
    robot_coord.setZero();
    wheel_speed.setZero();

    const static float b = L_wheel / (2.0f * r_wheel);

    // Digital out object for enabling motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    // In the robot there will be used 31:1 Metal Gearmotor 20Dx44L mm 12V CB
    // Define variables and create DC motor objects
    const float voltage_max = 12.0f;
    const float gear_ratio = 31.25f; 
    const float kn = 450.0f / 12.0f; //motor constant rpm / V
    const float velocity_max = kn * voltage_max / 60.0f; // Max velocity that can be reached
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max); //RIGHT
    motor_M1.setEnableMotionPlanner(true);
    motor_M1.setMaxVelocity(velocity_max / 2.5f); //right
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max); //LEFT
    motor_M2.setEnableMotionPlanner(true);
    motor_M2.setMaxVelocity(velocity_max / 2.5f); //left


    // Timer to measure task execution time
    main_task_timer.start();

    // Evaluation of velocity controller
    float y;
    float x;
    const float y_threshold = 800.0f;
    const float y_boundary = 500.0f;
    const float x_threshold = 500.0f;
    const float Kx = 1.0f / 200.0f;
    const float Ky = 1.0f / 200.0f;
    static float wheel_speed_eval;

    bool isNewMessagePixy = false;
    bool found;
    int i;
    int j;

    

    while (true) {
        
        main_task_timer.reset();
        
        if (do_execute_main_task) {
            y = (float)pixy.getTiltCommand();
            x = (float)pixy.getPanCommand();
            // state machine
            switch (robot_state) {
                case RobotState::INITIAL:
                    enable_motors = 1;
                    pixy.followerEnable(true);
                    isNewMessagePixy = pixy.checkForNewMessage();
                    if (isNewMessagePixy == true) {
                        robot_state = RobotState::FOLLOW;
                    } else {
                        robot_state = RobotState::SLEEP;
                    }
                    break;

                case RobotState::FOLLOW:
                    robot_coord(1) = pixy_following_x_fcn(x, x_threshold, Kx);
                    wheel_speed_eval = pixy_following_y_fcn(y, y_threshold, Ky, y_boundary);
                    robot_coord(0) = vel_cntrl_v2_fcn(wheel_speed_eval, b, robot_coord(1), Cwheel2robot);
                    // transform robot coordinates to wheel speed
                    wheel_speed = Cwheel2robot.inverse() * robot_coord;

                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PI));
                    motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PI));
                    isNewMessagePixy = pixy.checkForNewMessage();

                    if (isNewMessagePixy == false) {
                        robot_state = RobotState::SLEEP;
                    }
                    break;
                case RobotState::SLEEP:
                    motor_M1.setVelocity(0.0f);
                    motor_M2.setVelocity(0.0f);
                    pixy.followerEnable(false);
                    pixy.setServos(500, 500);
                    isNewMessagePixy = pixy.checkForNewMessage();

                    if (isNewMessagePixy == true) {
                        pixy.followerEnable(true);
                        robot_state = RobotState::FOLLOW;
                    }
                    break;
                /*
                case RobotState::LOOKFOR:
                    motor_M1.setVelocity(0.0f);
                    motor_M2.setVelocity(0.0f);
                    pixy.followerEnable(false);

                    for (i = pixy.getPanCommand(); i < 1000; i++) {
                        pixy.setServos(i, 500);
                        if (pixy.checkForNewMessage() == true) {
                            found = true;
                            break;
                        }
                    }
                    if (found == false) {
                        for (j = pixy.getTiltCommand(); j < 1000; j++) {
                            pixy.setServos(i, j);
                            if (pixy.checkForNewMessage() == true) {
                                found = true;
                                break;
                            }
                        }
                    }
                    if (found == false) {
                        for (i = 1000; i > 0; i--) {
                            pixy.setServos(i, j);
                            if (pixy.checkForNewMessage() == true) {
                                found = true;
                                break;
                            }
                        }
                    }
                    if (found == false) {
                        for (j = 1000; j > 500; j--) {
                            pixy.setServos(i, j);
                            if (pixy.checkForNewMessage() == true) {
                                found = true;
                                break;
                            }
                        }
                    }
                    if (found == false) {
                        for (i = 0; i < pixy.getPanCommand(); i ++) {
                            pixy.setServos(i, j);
                            if (pixy.checkForNewMessage() == true) {
                                found = true;
                                break;
                            }
                        }
                    }

                    
                    iter += 1;
                    if (iter == 1) {
                        pixy.setServos(500, y_threshold);
                    } else if (iter == 100) {
                        pixy.setServos(0, y_threshold);
                    } else if (iter == 200) {
                        pixy.setServos(0, 500);
                    } else if (iter == 300) {
                        pixy.setServos(500, 500);
                    } else if (iter == 400) {
                        pixy.setServos(1000, 500);
                    } else if (iter == 500) {
                        pixy.setServos(1000, y_threshold);
                    } else if (iter == 600) {
                        iter = 0;
                    }
                    
                    printf("%d \n",found);
                    if (found == true) {
                        found = false;
                        robot_state = RobotState::TURN2OBJECT;
                    }
                    break;

                case RobotState::TURN2OBJECT:
                    robot_coord(1) = pixy_following_x_fcn(i, x_threshold, Kx);
                    wheel_speed_eval = 0.5f;
                    robot_coord(0) = vel_cntrl_v2_fcn(wheel_speed_eval, b, robot_coord(1), Cwheel2robot);
                    // transform robot coordinates to wheel speed
                    wheel_speed = Cwheel2robot.inverse() * robot_coord;

                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PI));
                    motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PI));

                    pixy.followerEnable(true);
                    
                    if (isNewMessagePixy == true) {
                        robot_state = RobotState::FOLLOW;
                    }
                    isNewMessagePixy = pixy.checkForNewMessage();

                    break;
                */
                default:
                    break; // do nothing
            }
        }    
        user_led = !user_led;

        //printf("%f, %f \n",x, y);
        //printf("F %f,B %f,R %f,L %f \r\n", ir_distance_front_cm, ir_distance_back_cm, ir_distance_right_cm, ir_distance_left_cm);
        //printf("init %f,actual %f \r\n", init_rotation, actual_rotation);
        //printf("%f, %f \n", motor_M1.getRotation(), motor_M2.getRotation());
        //printf("%f, %f \n", wheel_speed(0) / (2.0f * M_PI), wheel_speed(1) / (2.0f * M_PI));
        //printf("%ld, %f \n", motor_M2.getEncoderCount(), motor_M2.getRotation());


        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_elapsed_time_ms <= 20){
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
        }
        
    }
}
void user_button_pressed_fcn()
{
    // do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
}

float pixy_following_x_fcn(const float& x, const float& x_threshold, const float& Kx)
{
    static float robot_omega;
    robot_omega = Kx * (x - x_threshold);
    return robot_omega;
}

float pixy_following_y_fcn(const float& y, const float& y_t, const float& Ky, const float& y_boundary) 
{
    static float wheel_speed_eval;
    if (y > y_boundary) {
        wheel_speed_eval = 2.0f * M_PI * Ky * (y_t - y);
    } else {
        wheel_speed_eval = 2.0f * M_PI * Ky * (y - (1000.0f - y_t));
    }
    return wheel_speed_eval;
}


float vel_cntrl_v2_fcn(const float& wheel_speed_max, const float& b, const float& robot_omega, const Eigen::Matrix2f& Cwheel2robot)
{
    static Eigen::Matrix<float, 2, 1> _wheel_speed;
    static Eigen::Matrix<float, 2, 1> _robot_coord;
    //wheel_speed(0) -> RIGHT
    //wheel_speed(1) -> LEFT
    if (wheel_speed_max >= 0) {
        if (robot_omega > 0) {
            _wheel_speed(0) = wheel_speed_max;
            _wheel_speed(1) = wheel_speed_max - 2*b*robot_omega;
        } else {
            _wheel_speed(0) = wheel_speed_max + 2*b*robot_omega;
            _wheel_speed(1) = wheel_speed_max;
        }
    }
    else {
        if (robot_omega > 0) {
            _wheel_speed(0) = wheel_speed_max + 2*b*robot_omega;
            _wheel_speed(1) = wheel_speed_max;
        } else {
            _wheel_speed(0) = wheel_speed_max;
            _wheel_speed(1) = wheel_speed_max - 2*b*robot_omega; 
        }
    }
    _robot_coord = Cwheel2robot * _wheel_speed;

    return _robot_coord(0);
}


/*
//OBSTACLES AVOIDING ALGORITHM
// logical variable main task
bool do_execute_main_task = false;

// user button on nucleo board
Timer user_button_timer;            
DebounceIn user_button(USER_BUTTON);
// Function that triggers main task execution   
void user_button_pressed_fcn();

// Sensor evaluation function
float ir_sensor_compensation(float _ir_distance_mV);

// Controling functions
float rot_vel_from_radius_fcn(const float& wheel_speed_max, const float& r_wheel, const float& L_wheel ,const float& R_turn);
float turn_angle_limit_fcn(const float& R_turn, const float& r_wheel, const float& L_wheel, const float& angle);
float vel_cntrl_v2_fcn(const float& wheel_speed_max, const float& b, const float& robot_omega, const Eigen::Matrix2f& Cwheel2robot);


int main()
{
    // states and actual state for state machine, machine will have 8 states:
    // initial - to enable motors
    // forward - to move forward
    // backward - to move backward and make turn in case of facing obstacle by front sensor
    // backward_corner_left - to move backward and make tight turn in case of facing obstacle by front and left sensor
    // backward_corner_right - to move backward and make tight turn in case of facing obstacle by front and right sensor
    // right - to turn right in case of facing obstacle from left side
    // left - to turn left in case of facing obstacle from right side
    // sleep - to do nothing
    enum RobotState {
        INITIAL,
        FORWARD,
        BACKWARD,
        BACKWARD_CORNER_LEFT,
        BACKWARD_CORNER_RIGHT,
        RIGHT,
        LEFT,
        SLEEP,
    } robot_state = RobotState::INITIAL;

    // attach button fall function to user button object
    user_button.fall(&user_button_pressed_fcn);

    // while loop gets executed every main_task_period_ms milliseconds
    const int main_task_period_ms = 20;   // define main task period time in ms
    Timer main_task_timer;  

    // led on nucleo board
    DigitalOut user_led(USER_LED); 

    // Robot kinematics
    const float r_wheel = 0.0563f / 2.0f; // wheel radius
    const float L_wheel = 0.13f;          // distance from wheel to wheel
    Eigen::Matrix2f Cwheel2robot; // transform wheel to robot
    //Eigen::Matrix2f Crobot2wheel; // transform robot to wheel
    Cwheel2robot <<  r_wheel / 2.0f   ,  r_wheel / 2.0f   ,
                     r_wheel / L_wheel, -r_wheel / L_wheel;
    //Crobot2wheel << 1.0f / r_wheel,  L_wheel / (2.0f * r_wheel),
    //                1.0f / r_wheel, -L_wheel / (2.0f * r_wheel);
    Eigen::Vector2f robot_coord;  // contains v and w (robot translational and rotational velocities)
    Eigen::Vector2f wheel_speed;  // w1 w2 (wheel speed)
    robot_coord.setZero();
    wheel_speed.setZero();

    const float R_turn_forward = L_wheel;
    const float R_turn_backward = L_wheel;
    const float R_turn_corner_backward = L_wheel / 2.0f;  

    const float alfa_backward = 180.0f;
    const float alfa_backward_corner = 270.0f;
    const float wheel_rounds_backward = turn_angle_limit_fcn(R_turn_backward, r_wheel, L_wheel, alfa_backward);
    const float wheel_rounds_backward_corner = turn_angle_limit_fcn(R_turn_corner_backward, r_wheel, L_wheel, alfa_backward_corner);

    static float init_rotation;
    static float actual_rotation;
    static bool isFirst = false;

    const float b = L_wheel / (2.0f * r_wheel);

    //IR range sensors:
    AnalogIn ir_analog_in_left(PC_1);
    AnalogIn ir_analog_in_right(PC_3);
    AnalogIn ir_analog_in_front(PC_5);
    AnalogIn ir_analog_in_back(PB_1);

    static float ir_distance_left_cm = 0.0f;
    static float ir_distance_right_cm = 0.0f;
    static float ir_distance_front_cm = 0.0f;
    static float ir_distance_back_cm = 0.0f;

    // Distance threshold for sensors
    const float threshold = 13.0f;

    // Digital out object for enabling motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    // In the robot there will be used 31:1 Metal Gearmotor 20Dx44L mm 12V CB
    // Define variables and create DC motor objects
    const float voltage_max = 12.0f;
    const float gear_ratio = 31.25f; 
    const float kn = 450.0f / 12.0f; //motor constant rpm / V
    const float velocity_max = kn * voltage_max / (60.0f * 6.0f); // Max velocity that can be reached
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max); //RIGHT
    motor_M1.setEnableMotionPlanner(true);
    motor_M1.setMaxVelocity(velocity_max); //right
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max); //LEFT
    motor_M2.setEnableMotionPlanner(true);
    motor_M2.setMaxVelocity(velocity_max); //left

    const float wheel_speed_max_rads = 2.0f * M_PI * velocity_max; 

    // Timer to measure task execution time
    main_task_timer.start();

    while (true) {

        main_task_timer.reset();

        if (do_execute_main_task) {
            
            //IR sensors distances
            ir_distance_left_cm = ir_sensor_compensation(1.0e3f * ir_analog_in_left.read() * 3.3f);
            ir_distance_right_cm = ir_sensor_compensation(1.0e3f * ir_analog_in_right.read() * 3.3f);
            ir_distance_front_cm = ir_sensor_compensation(1.0e3f * ir_analog_in_front.read() * 3.3f);
            ir_distance_back_cm = ir_sensor_compensation(1.0e3f * ir_analog_in_back.read() * 3.3f);

            // transform robot coordinates to wheel speed
            wheel_speed = Cwheel2robot.inverse() * robot_coord;
            
            switch (robot_state) {
                case RobotState::INITIAL:
                    enable_motors = 1;
                    robot_state = RobotState::FORWARD;
                    break;

                case RobotState::FORWARD:
                    printf("Forward \n");
                    robot_coord(1) = 0;
                    robot_coord(0) = vel_cntrl_v2_fcn(wheel_speed_max_rads, b, robot_coord(1), Cwheel2robot);
                    wheel_speed = Cwheel2robot.inverse() * robot_coord;

                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PI));
                    motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PI));
                    
                    if (ir_distance_front_cm < threshold & ir_distance_right_cm > threshold & ir_distance_left_cm > threshold) {
                        isFirst = true;
                        robot_state = RobotState::BACKWARD;
                    }
                    else if (ir_distance_left_cm < threshold) {
                        robot_state = RobotState::RIGHT;
                    }
                    else if (ir_distance_right_cm < threshold) {
                        robot_state = RobotState::LEFT;
                    }
                    else if (ir_distance_front_cm < threshold & ir_distance_right_cm < threshold) {
                        isFirst = true;
                        robot_state = RobotState::BACKWARD_CORNER_RIGHT;
                    }
                    else if (ir_distance_front_cm < threshold & ir_distance_left_cm < threshold) {
                        isFirst = true;
                        robot_state = RobotState::BACKWARD_CORNER_LEFT;
                    }
                    break;

                case RobotState::BACKWARD: // add routine to turn with bigger radius and go forward after 45 degrees turn
                    printf("Backward \n");
                    if (isFirst) {
                        if (ir_distance_left_cm < ir_distance_right_cm) {
                            init_rotation = motor_M1.getRotation();
                            actual_rotation = motor_M1.getRotation();
                            robot_coord(1) = - rot_vel_from_radius_fcn(wheel_speed_max_rads, r_wheel, L_wheel, R_turn_backward);
                        }
                        else {
                            init_rotation = motor_M2.getRotation();
                            actual_rotation = motor_M2.getRotation();
                            robot_coord(1) = rot_vel_from_radius_fcn(wheel_speed_max_rads, r_wheel, L_wheel, R_turn_backward);
                        } 
                    } 
                    else {
                        if (ir_distance_left_cm < ir_distance_right_cm) {
                            actual_rotation = motor_M1.getRotation();
                        }
                        else {
                            actual_rotation = motor_M2.getRotation();
                        }
                    }
                    isFirst = false;
                    robot_coord(0) = vel_cntrl_v2_fcn(-wheel_speed_max_rads, b, robot_coord(1), Cwheel2robot);
                    wheel_speed = Cwheel2robot.inverse() * robot_coord;
                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PI));
                    motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PI));
                    

                    if (ir_distance_back_cm < threshold | actual_rotation < (init_rotation - wheel_rounds_backward)) {
                        init_rotation = 0.0f;
                        actual_rotation = 0.0f;
                        robot_state = RobotState::FORWARD;
                    }
                    break;

                case RobotState::RIGHT:
                    robot_coord(1) = - rot_vel_from_radius_fcn(wheel_speed_max_rads, r_wheel, L_wheel, R_turn_forward);
                    robot_coord(0) = vel_cntrl_v2_fcn(wheel_speed_max_rads, b, robot_coord(1), Cwheel2robot);
                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PI));
                    motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PI));

                    if  (ir_distance_left_cm > threshold) {
                        robot_state = RobotState::FORWARD;
                    }
                    else if (ir_distance_front_cm < threshold & ir_distance_left_cm < threshold) {
                        isFirst = true;
                        robot_state = RobotState::BACKWARD_CORNER_LEFT;
                    }
                    break;

                case RobotState::LEFT:
                    robot_coord(1) = rot_vel_from_radius_fcn(wheel_speed_max_rads, r_wheel, L_wheel, R_turn_forward);
                    robot_coord(0) = vel_cntrl_v2_fcn(wheel_speed_max_rads, b, robot_coord(1), Cwheel2robot);
                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PI));
                    motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PI));

                    if  (ir_distance_right_cm > threshold) {
                        robot_state = RobotState::FORWARD;
                    }
                    else if (ir_distance_front_cm < threshold & ir_distance_right_cm < threshold) {
                        isFirst = true;
                        robot_state = RobotState::BACKWARD_CORNER_RIGHT;
                    }
                    break;

                // Write routine to turn around one of the wheel for both cases, add condition then after turning 90 degrees go forward
                case RobotState::BACKWARD_CORNER_LEFT:
                    printf("Backward left \n");
                    if (isFirst) {
                        init_rotation = motor_M1.getRotation();
                        actual_rotation = motor_M1.getRotation();
                    } 
                    else {
                        actual_rotation = motor_M1.getRotation();
                    }
                    isFirst = false; 
                    robot_coord(1) = - rot_vel_from_radius_fcn(wheel_speed_max_rads, r_wheel, L_wheel, R_turn_corner_backward);
                    robot_coord(0) = vel_cntrl_v2_fcn(0.0f, b, robot_coord(1), Cwheel2robot);
                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PI));
                    motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PI));

                    if  (ir_distance_back_cm < threshold | actual_rotation < (init_rotation - wheel_rounds_backward_corner)) {
                        init_rotation = 0.0f;
                        actual_rotation = 0.0f;
                        robot_state = RobotState::FORWARD;
                    }
                    break;

                case RobotState::BACKWARD_CORNER_RIGHT:
                    printf("Backward right\n");
                    if (isFirst) {
                        init_rotation = motor_M2.getRotation();
                        actual_rotation = motor_M2.getRotation();
                    } 
                    else {
                        actual_rotation = motor_M2.getRotation();
                    }
                    isFirst = false;
                    robot_coord(1) = rot_vel_from_radius_fcn(wheel_speed_max_rads, r_wheel, L_wheel, R_turn_corner_backward);
                    robot_coord(0) = vel_cntrl_v2_fcn(0.0f, b, robot_coord(1), Cwheel2robot);
                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PI));
                    motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PI));

                    if  (ir_distance_back_cm < threshold | actual_rotation < (init_rotation - wheel_rounds_backward_corner)) {
                        init_rotation = 0.0f;
                        actual_rotation = 0.0f;
                        robot_state = RobotState::FORWARD;
                    }
                    break;

                default:
                    break; // do nothing
            }
        }
            user_led = !user_led;

        //printf("F %f,B %f,R %f,L %f \r\n", ir_distance_front_cm, ir_distance_back_cm, ir_distance_right_cm, ir_distance_left_cm);
        //printf("init %f,actual %f \r\n", init_rotation, actual_rotation);
        //printf("%f, %f \n", motor_M1.getRotation(), motor_M2.getRotation());
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}
void user_button_pressed_fcn()
{
    // do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
}

float turn_angle_limit_fcn(const float& R_turn, const float& r_wheel, const float& L_wheel, const float& angle)
{
    static float R_out_wheel;
    static float circle_slice_l;
    static float wheel_rounds;

    R_out_wheel = R_turn + L_wheel/2;
    circle_slice_l = angle * 2.0f * M_PI * R_out_wheel / 360.0f;
    wheel_rounds = circle_slice_l/(2 * M_PI * r_wheel);
    return wheel_rounds; 
}

float rot_vel_from_radius_fcn(const float& wheel_speed_max, const float& r_wheel, const float& L_wheel, const float& R_turn)
{
    static float robot_omega;
    robot_omega = 2.0f * r_wheel * wheel_speed_max / (2.0f*R_turn + L_wheel);
    return robot_omega; 
}

float vel_cntrl_v2_fcn(const float& wheel_speed_max, const float& b, const float& robot_omega, const Eigen::Matrix2f& Cwheel2robot)
{
    static Eigen::Matrix<float, 2, 1> _wheel_speed;
    static Eigen::Matrix<float, 2, 1> _robot_coord;
    //wheel_speed(0) -> RIGHT
    //wheel_speed(1) -> LEFT
    if (wheel_speed_max >= 0) {
        if (robot_omega > 0) {
            _wheel_speed(0) = wheel_speed_max;
            _wheel_speed(1) = wheel_speed_max - 2*b*robot_omega;
        } else {
            _wheel_speed(0) = wheel_speed_max + 2*b*robot_omega;
            _wheel_speed(1) = wheel_speed_max;
        }
    }
    else {
        if (robot_omega > 0) {
            _wheel_speed(0) = wheel_speed_max + 2*b*robot_omega;
            _wheel_speed(1) = wheel_speed_max;
        } else {
            _wheel_speed(0) = wheel_speed_max;
            _wheel_speed(1) = wheel_speed_max - 2*b*robot_omega; 
        }
    }
    _robot_coord = Cwheel2robot * _wheel_speed;

    return _robot_coord(0);
}

float ir_sensor_compensation(float _ir_distance_mV) {

    static const float a = 2.574e+04f;
    static const float b = -29.37f;

    static float ir_distance_cm = 0.0f;
    if (_ir_distance_mV + b == 0) ir_distance_cm = -1.0f;
    else ir_distance_cm = a / (_ir_distance_mV + b);

    return ir_distance_cm;
}
*/

// LINE FOLLOWING ALGORITHM
/*
// logical variable main task
bool do_execute_main_task = false; 

// user button on Nucleo board
Timer user_button_timer;            
DebounceIn user_button(USER_BUTTON);
// Function that triggers main task execution   
void user_button_pressed_fcn();    

int main()
{
    // states and actual state for the state machine, the machine will have 3 states:
    // initial - to enable all systems
    // follow - to follow the line
    // sleep - to wait for the signal from the environment (e.g. line detection)
    enum RobotState {
        INITIAL,
        FOLLOW,
        SLEEP,
    } robot_state = RobotState::INITIAL;

    // attach button fall function to user button object
    user_button.fall(&user_button_pressed_fcn);

    // while loop gets executed every main_task_period_ms milliseconds
    const int main_task_period_ms = 20;   // define main task period time in ms
    Timer main_task_timer;  

    // led on nucleo board
    DigitalOut user_led(USER_LED); 

    // digital out object for enabling motors
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    // In the robot there will be used 78:1 Metal Gearmotor 20Dx44L mm 12V CB
    // Define variables and create DC motor objects
    const float voltage_max = 12.0f;
    const float gear_ratio = 31.25f; 
    const float kn = 450.0f / 12.0f; //motor constant rpm / V
    const float velocity_max = kn * voltage_max / 60.0f; // Max velocity that can be reached
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    motor_M1.setMaxVelocity(velocity_max / (2.5f)); //right
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);
    motor_M2.setMaxVelocity(velocity_max / (2.5f)); //left

    const float d_wheel = 0.0564f;        // wheel diameter
    const float L_wheel = 0.13f;          // distance from wheel to wheel
    // sensor data evalution
    const float bar_dist = 0.083f;

    LineFollower lineFollower(PB_9, PB_8, bar_dist, d_wheel, L_wheel, velocity_max);
    lineFollower.setRotationalVelGain(2.0f, 22.0f);

    // condition for state machine that will stop the robot 1 seconds after leaving the line (CAN BE CHANGED)
    bool move = false;
    const static float stop_time = 1.0f; //seconds
    const static int stop_time_iteration = stop_time * 1000 / main_task_period_ms;
    static int i = stop_time_iteration + 1;

    // timer to measure task execution time
    main_task_timer.start();

    while (true) {

        main_task_timer.reset();

        if (do_execute_main_task) {

            // line detection checking
            if (lineFollower.isLedActive()) {
                i = 0;
            } else {
                i += 1;
            }

            // robot stop condition checking
            if (i > stop_time_iteration) {
                move = false;
            } else {
                move = true;
            }

            // state machine
            switch (robot_state) {
                case RobotState::INITIAL:
                    enable_motors = 1;
                    if (move == true) {
                        robot_state = RobotState::FOLLOW;
                    } else {
                        robot_state = RobotState::SLEEP;
                    }
                    break;

                case RobotState::FOLLOW:
                    motor_M1.setVelocity(lineFollower.getRightWheelVelocity()); // set a desired speed for speed controlled dc motors M1
                    motor_M2.setVelocity(lineFollower.getLeftWheelVelocity()); // set a desired speed for speed controlled dc motors M2

                    if (move == false) {
                        robot_state = RobotState::SLEEP;
                    }
                    break;

                case RobotState::SLEEP:
                    motor_M1.setVelocity(0);
                    motor_M2.setVelocity(0);

                    if (move == true) {
                        robot_state = RobotState::FOLLOW;
                    }
                    break;
                default:
                    break; // do nothing
            }
        }
        // toggling user-led
        user_led = !user_led;

        // printing parameters
        //printf("Right command:%f, Right real: %f, Left command: %f, Left real: %f \r\n",lineFollower.getRightWheelVelocity(), motor_M1.getVelocity(), lineFollower.getLeftWheelVelocity(), motor_M2.getVelocity());

        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void user_button_pressed_fcn()
{
    // do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
}
*/