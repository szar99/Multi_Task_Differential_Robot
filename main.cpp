#include <mbed.h>
#include <math.h>

#include "pm2_drivers/PM2_Drivers.h"
#include "eigen/Dense.h"

#define NEW_PES_BOARD_VERSION

#ifdef NEW_PES_BOARD_VERSION
    #define PN_enable_Motors PB_15
    #define PN_pwm_M1 PB_13
    #define PN_pwm_M2 PA_9
    #define PN_encoder_M1_A PA_6
    #define PN_encoder_M1_B PC_7
    #define PN_encoder_M2_A PB_6
    #define PN_encoder_M2_B PB_7
#else
    #define PN_enable_Motors PB_2
    #define PN_pwm_M1 PA_8
    #define PN_pwm_M2 PA_9
    #define PN_encoder_M1_A PB_6
    #define PN_encoder_M1_B PB_7
    #define PN_encoder_M2_A PA_6
    #define PN_encoder_M2_B PC_7
#endif

#define M_PI 3.14159265358979323846  // number pi

// logical variable main task
bool do_execute_main_task = false;  // this variable will be toggled via the user button (blue button) to or not to execute the main task

// user button on nucleo board
Timer user_button_timer;            // create Timer object which we use to check if user button was pressed for a certain time (robust against signal bouncing)
InterruptIn user_button(PC_13);     // create InterruptIn interface object to evaluate user button falling and rising edge (no blocking code in ISR)
void user_button_pressed_fcn();     // custom functions which gets executed when user button gets pressed and released, definition below
void user_button_released_fcn();

// controller functions
float ang_cntrl_fcn(const float& Kp, const float& Kp_nl, const float& angle);
float vel_cntrl_v1_fcn(const float& vel_max, const float& vel_min, const float& ang_max, const float& angle);
float vel_cntrl_v2_fcn(const float& wheel_speed_max, const float& b, const float& robot_omega, const Eigen::Matrix2f& Cwheel2robot);

int main()
{
    // while loop gets executed every main_task_period_ms milliseconds
    const int main_task_period_ms = 10;   // define main task period time in ms e.g. 50 ms -> main task runns 20 times per second
    Timer main_task_timer;                // create Timer object which we use to run the main task every main task period time in ms

    // led on nucleo board
    DigitalOut user_led(LED1);      // create DigitalOut object to command user led

    // Sharp GP2Y0A41SK0F, 4-40 cm IR Sensor
    float ir_distance_mV = 0.0f;    // define variable to store measurement
    AnalogIn ir_analog_in(PC_2);    // create AnalogIn object to read in infrared distance sensor, 0...3.3V are mapped to 0...1

    // 78:1, 100:1, ... Metal Gearmotor 20Dx44L mm 12V CB
    DigitalOut enable_motors(PN_enable_Motors);    // create DigitalOut object to enable dc motors

    // create SpeedController objects
    FastPWM pwm_M1(PN_pwm_M1);  // motor M1 is closed-loop speed controlled (angle velocity)
    FastPWM pwm_M2(PN_pwm_M2);   // motor M2 is closed-loop speed controlled (angle velocity)
    EncoderCounter  encoder_M1(PN_encoder_M1_A, PN_encoder_M1_B); // create encoder objects to read in the encoder counter values
    EncoderCounter  encoder_M2(PN_encoder_M2_A, PN_encoder_M2_B);
    const float max_voltage = 12.0f;                  // define maximum voltage of battery packs, adjust this to 6.0f V if you only use one batterypack
    const float counts_per_turn = 20.0f * 78.125f;    // define counts per turn at gearbox end: counts/turn * gearratio
    const float kn = 180.0f / 12.0f;                  // define motor constant in rpm per V
    
    // create SpeedController objects
    SpeedController* speedControllers[2];
    speedControllers[0] = new SpeedController(counts_per_turn, kn, max_voltage, pwm_M1, encoder_M1);
    speedControllers[1] = new SpeedController(counts_per_turn, kn, max_voltage, pwm_M2, encoder_M2);
    //speedControllers[0]->setMaxAccelerationRPS(999.0f); // big number, so it is no doing anything
    //speedControllers[1]->setMaxAccelerationRPS(999.0f); // big number, so it is no doing anything

    // create SensorBar object for sparkfun line follower array
    I2C i2c(PB_9, PB_8);
    SensorBar sensor_bar(i2c, 0.1175f); // second input argument is distance from bar to wheel axis

    // robot kinematics
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

    // attach button fall and rise functions to user button object
    user_button.fall(&user_button_pressed_fcn);
    user_button.rise(&user_button_released_fcn);

    // start timer
    main_task_timer.start();

    while (true) { // this loop will run forever

        main_task_timer.reset();

        if (do_execute_main_task) {

            // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
            enable_motors = 1;

            // read SensorBar
            static float sensor_bar_avgAngleRad = 0.0f; // by making this static it will not be overwritten (only fist time set to zero)
            if (sensor_bar.isAnyLedActive()) {
                sensor_bar_avgAngleRad = sensor_bar.getAvgAngleRad();
            }

            const static float Kp = 2.0f; // by making this const static it will not be overwritten and only initiliazed once
            const static float Kp_nl = 17.0f;
            robot_coord(1) = ang_cntrl_fcn(Kp, Kp_nl, sensor_bar_avgAngleRad);

            // nonlinear controllers version 1 (whatever came to my mind)
            /*
            const static float vel_max = 0.3374f; //0.10f;
            const static float vel_min = 0.00f; //0.02f;
            const static float ang_max = 27.0f * M_PI / 180.0f;
            robot_coord(0) = vel_cntrl_v1_fcn(vel_max, vel_min, ang_max, sensor_bar_avgAngleRad);
            */

            // nonlinear controllers version 2 (one wheel always at full speed controller)
            ///*
            const static float wheel_speed_max = max_voltage * kn / 60.0f * 2.0f * M_PI;
            const static float b = L_wheel / (2.0f * r_wheel);
            robot_coord(0) = vel_cntrl_v2_fcn(wheel_speed_max, b, robot_coord(1), Cwheel2robot);
            //*/

            // transform robot coordinates to wheel speed
            wheel_speed = Cwheel2robot.inverse() * robot_coord;

            // read analog input
            ir_distance_mV = 1.0e3f * ir_analog_in.read() * 3.3f;

            // command speedController objects
            speedControllers[0]->setDesiredSpeedRPS(wheel_speed(0) / (2.0f * M_PI)); // set a desired speed for speed controlled dc motors M1
            speedControllers[1]->setDesiredSpeedRPS(wheel_speed(1) / (2.0f * M_PI)); // set a desired speed for speed controlled dc motors M2

        } else {

            enable_motors = 0;

            ir_distance_mV = 0.0f;

            speedControllers[0]->setDesiredSpeedRPS(0.0f);
            speedControllers[1]->setDesiredSpeedRPS(0.0f);

        }

        user_led = !user_led;

        // do only output via serial what's really necessary (this makes your code slow)
        printf("%f, %f, %f\r\n", speedControllers[0]->getSpeedRPS(), speedControllers[1]->getSpeedRPS(), sensor_bar.getAvgAngleRad() * 180.0f / M_PI);

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void user_button_pressed_fcn()
{
    user_button_timer.start();
    user_button_timer.reset();
}

void user_button_released_fcn()
{
    // read timer and toggle do_execute_main_task if the button was pressed longer than the below specified time
    int user_button_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(user_button_timer.elapsed_time()).count();
    user_button_timer.stop();
    if (user_button_elapsed_time_ms > 200) {
        do_execute_main_task = !do_execute_main_task;
    }
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