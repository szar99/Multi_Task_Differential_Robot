#include <mbed.h>
#include <math.h>

#include "pm2_drivers/DebounceIn.h"
#include "pm2_drivers/Servo.h"
#include "pm2_drivers/EncoderCounter.h"
#include "pm2_drivers/DCMotor.h"
#include "pm2_drivers/UltrasonicSensor.h"
#include "pm2_drivers/IMU.h"
#include "pm2_drivers/SensorBar.h"
#include "eigen/Dense.h"

#define NEW_PES_BOARD_VERSION
#ifdef NEW_PES_BOARD_VERSION
    #define USER_BUTTON PC_13
    #define USER_LED PA_5

    #define PB_D0 PB_2
    #define PB_D1 PC_8
    #define PB_D2 PC_6
    #define PB_D3 PB_12

    #define PB_PWM_M1 PB_13
    #define PB_PWM_M2 PA_9
    #define PB_PWM_M3 PA_10

    #define PB_ENC_A_M1 PA_6
    #define PB_ENC_B_M1 PC_7
    #define PB_ENC_A_M2 PB_6
    #define PB_ENC_B_M2 PB_7
    #define PB_ENC_A_M3 PA_0
    #define PB_ENC_B_M3 PA_1

    #define PB_IMU_SDA PC_9
    #define PB_IMU_SCL PA_8

    #define PN_ENABLE_DCMOTORS PB_15
#else
    #define USER_BUTTON PC_13
    #define USER_LED PA_5

    #define PB_D0 PC_9  // ???
    #define PB_D1 PC_8  // ???
    #define PB_D2 PC_6  // ???
    #define PB_D3 PB_12 // ???

    #define PB_PWM_M1 PA_8
    #define PB_PWM_M2 PA_9
    #define PB_PWM_M3 PA_13 // ???

    #define PB_ENC_A_M1 PB_6
    #define PB_ENC_B_M1 PB_7
    #define PB_ENC_A_M2 PA_6
    #define PB_ENC_A_M2 PC_7
    #define PB_ENC_A_M3 PA_1 // ???
    #define PB_ENC_B_M3 PA_0 // ???

    #define PB_IMU_SDA // ???
    #define PB_IMU_SCL // ???
    
    #define PN_ENABLE_DCMOTORS PB_2 // PB_13 ???
#endif

#define M_PI 3.14159265358979323846  // number pi

// logical variable main task
bool do_execute_main_task = false; 

// user button on nucleo board
Timer user_button_timer;            
DebounceIn user_button(USER_BUTTON);
// Function that triggers main task execution   
void user_button_pressed_fcn();    

// controller functions
float ang_cntrl_fcn(const float& Kp, const float& Kp_nl, const float& angle);
float vel_cntrl_v1_fcn(const float& vel_max, const float& vel_min, const float& ang_max, const float& angle);
float vel_cntrl_v2_fcn(const float& wheel_speed_max, const float& b, const float& robot_omega, const Eigen::Matrix2f& Cwheel2robot);

int main()
{
    // states and actual state for state machine, machine will have 3 states:
    // initial - to enable all systems
    // follow - to follow the object or line
    // sleep - to wait for signal from the environment (e.g. new object)
    enum RobotState {
        INITIAL,
        FOLLOW,
        SLEEP,
    } robot_state = RobotState::INITIAL;

    // Condition for sensor
    int i = 51;

    // attach button fall function to user button object
    user_button.fall(&user_button_pressed_fcn);

    // while loop gets executed every main_task_period_ms milliseconds
    const int main_task_period_ms = 20;   // define main task period time in ms
    Timer main_task_timer;  

    // led on nucleo board
    DigitalOut user_led(USER_LED); 

    // Robot kinematics
    const float r_wheel = 0.0358f / 2.0f; // wheel radius
    const float L_wheel = 0.143f;         // distance from wheel to wheel
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

    // Sensor data evalution
    const float bar_dist = 0.1175f; // distance from bar to wheel axis 

    // As robot can have multiple sensors on, here is the place to define them
    // Pixy cam ...
    // IR range sensor ...
    // Sensor bar
    I2C i2c(PB_9, PB_8);
    SensorBar sensor_bar(i2c, bar_dist);

    // Digital out object for enabling motors
    DigitalOut enable_motors(PN_ENABLE_DCMOTORS);

    // In the robot there will be used 78:1 Metal Gearmotor 20Dx44L mm 12V CB
    // Define variables and create DC motor objects
    const float voltage_max = 12.0f;
    const float gear_ratio = 78.125f; 
    const float kn = 180.0f / 12.0f; //motor constant rpm / V
    const float velocity_max = kn * voltage_max / 60.0f; // Max velocity that can be reached
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    motor_M1.setMaxVelocity(velocity_max); 
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);
    motor_M2.setMaxVelocity(velocity_max);

    // Timer to measure task execution time
    main_task_timer.start();

    while (true) {

        main_task_timer.reset();

        if (do_execute_main_task) { //After pressing user button
            static float sensor_bar_avgAngleRad = 0.0f;
            if (sensor_bar.isAnyLedActive()) {
                sensor_bar_avgAngleRad = sensor_bar.getAvgAngleRad();
                i = 0;
            } 
            else {
                i += 1;
            }

            const static float Kp = 2.0f; // by making this const static it will not be overwritten and only initiliazed once
            const static float Kp_nl = 17.0f;
            robot_coord(1) = ang_cntrl_fcn(Kp, Kp_nl, sensor_bar_avgAngleRad);
            // nonlinear controllers version 2 (one wheel always at full speed controller)

            const static float wheel_speed_max = 20* voltage_max * kn / (60.0f * 2.0f * M_PI); 
            const static float b = L_wheel / (2.0f * r_wheel);
            robot_coord(0) = vel_cntrl_v2_fcn(wheel_speed_max, b, robot_coord(1), Cwheel2robot);

            // transform robot coordinates to wheel speed
            wheel_speed = Cwheel2robot.inverse() * robot_coord;

            motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PI)); // set a desired speed for speed controlled dc motors M1
            motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PI)); // set a desired speed for speed controlled dc motors M2

            
            switch (robot_state) {
                case RobotState::INITIAL:
                    enable_motors = 1;

                    if (i > 50) {
                        robot_state = RobotState::SLEEP;
                    } else {
                        robot_state = RobotState::FOLLOW;
                    }
                    break;

                case RobotState::FOLLOW:
                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PI)); // set a desired speed for speed controlled dc motors M1
                    motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PI)); // set a desired speed for speed controlled dc motors M2

                    if (i > 50) {
                        robot_state = RobotState::SLEEP;
                    }
                    break;

                case RobotState::SLEEP:
                    motor_M1.setVelocity(0);
                    motor_M2.setVelocity(0);

                    if (i == 0) {
                        robot_state = RobotState::FOLLOW;
                    }
                    break;
                default:
                    break; // do nothing
            }
        }
        user_led = !user_led;

        printf("%f, %f, %f, %d, %d\r\n", motor_M1.getVelocity(), motor_M2.getVelocity(), sensor_bar.getAvgAngleRad() * 180.0f / M_PI, i, sensor_bar.isAnyLedActive());

        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void user_button_pressed_fcn()
{
    // do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
}

float ang_cntrl_fcn(const float& Kp, const float& Kp_nl, const float& angle)
{
    static float retval = 0.0f;
    if (angle > 0) {
        retval = Kp * angle + Kp_nl * angle * angle;
    } else if (angle <= 0) {
        retval = Kp * angle - Kp_nl * angle * angle;
    }
    return retval;
}

float vel_cntrl_v1_fcn(const float& vel_max, const float& vel_min, const float& ang_max, const float& angle)
{
    const static float gain = (vel_min - vel_max) / ang_max;
    const static float offset = vel_max;
    return gain * fabs(angle) + offset;
}

float vel_cntrl_v2_fcn(const float& wheel_speed_max, const float& b, const float& robot_omega, const Eigen::Matrix2f& Cwheel2robot)
{
    static Eigen::Matrix<float, 2, 2> _wheel_speed;
    static Eigen::Matrix<float, 2, 2> _robot_coord;
    if (robot_omega > 0) {
        _wheel_speed(0) = wheel_speed_max;
        _wheel_speed(1) = wheel_speed_max - 2*b*robot_omega;
    } else {
        _wheel_speed(0) = wheel_speed_max + 2*b*robot_omega;
        _wheel_speed(1) = wheel_speed_max;
    }
    _robot_coord = Cwheel2robot * _wheel_speed;

    return _robot_coord(0);
}