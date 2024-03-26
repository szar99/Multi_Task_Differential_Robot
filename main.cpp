#include "mbed.h"

// pes board pin map
#include "pm2_drivers/PESBoardPinMap.h"

// drivers
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

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once
// while loop gets executed every main_task_period_ms milliseconds, this is a
// simple approach to repeatedly execute main
const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, there for
                                    // the main task will run 50 times per second

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(USER_BUTTON); // create DebounceIn object to evaluate the user button
                                     // falling and rising edge
void toggle_do_execute_main_fcn();   // custom function which is getting executed when user
                                     // button gets pressed, definition below
//float iir_filter(float u_k);         // function for implementing low pass filter 

// main runs as an own thread
int main()
{
    // attach button fall function address to user button object, button has a pull-up resistor
    user_button.fall(&toggle_do_execute_main_fcn);

    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms
    Timer timer;

    // led on nucleo board
    DigitalOut user_led(USER_LED);
 
    // start timer
    main_task_timer.start();
    timer.start();

    // PIXY FOLLOWER OBJECTS AND VARIABLES
    PixyCam2 pixy(PC_10, PC_11, 230400);

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {
            pixy.followerEnable(true);
            printf("Hello \n");
        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

            }
        }
        // toggling the user led
        user_led = !user_led;

        
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}
