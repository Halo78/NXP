/****
 * Made by Marouane Gaoua
 * 2023
 * ALL RIGHTS RESERVED
*/

#include "nxpcup_race.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <uORB/Subscription.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

#define KP 0.6f// increase this value to make the car turn tighter
#define KD 1.5f // you can adjust this value if needed
#define KI 0.0000f // you can adjust this value if needed
float previous_error = 0.0f;
float integral = 0.0f;
float previous_steer = 0.0f;
int num_frame = 0 ;


uint8_t get_num_vectors(const pixy_vector_s &pixy) {
    uint8_t numVectors = 0;
    
    if (pixy.m0_x0 || pixy.m0_x1 || pixy.m0_y0 || pixy.m0_y1) {
        numVectors++;
    }
    
    if (pixy.m1_x0 || pixy.m1_x1 || pixy.m1_y0 || pixy.m1_y1) {
        numVectors++;
    }

    return numVectors;
}

roverControl raceTrack(const pixy_vector_s &pixy)
{
    Vector main_vec;
    uint8_t frameWidth = 79;
    uint8_t frameHeight = 52;
    int16_t window_center = (frameWidth / 2);
    roverControl control{};
    float x, y;
    static hrt_abstime no_line_time = 0;
    hrt_abstime time_diff = 0;
    static bool first_call = true;
    uORB::Subscription distance_sensor_sub{ORB_ID(distance_sensor)}; 
    struct distance_sensor_s distance_sensor;
    distance_sensor_sub.copy(&distance_sensor);

    PX4_INFO("num_frame = %d", num_frame);

    if (distance_sensor.current_distance > 0.6f) {
        switch (get_num_vectors(pixy)) {
            case 0: {
                if (first_call) {
                    no_line_time = hrt_absolute_time();
                    first_call = false;
                    control.steer = 0.0f;
                    control.speed = SPEED_FAST;
                }
                time_diff = hrt_absolute_time() - no_line_time;
                if (time_diff < 500000) {
                    control.steer = 0.0f;
                    control.speed = SPEED_FAST;
                } else {
                    control.steer = 0.0f;
                    control.speed = SPEED_FAST;
                }
                control.steer = 0.0f;
                control.speed = SPEED_FAST;
                break;
            }
            case 2: {
                num_frame++;
                first_call = true;
                main_vec.m_x1 = (pixy.m0_x1 + pixy.m1_x1) / 2;
                float current_error = -(float)(main_vec.m_x1 - window_center) / (float)frameWidth;
                float proportional = KP * current_error;
                float derivative = KD * (current_error - previous_error);
                integral += KI * current_error;
                float total_error = proportional + derivative + integral;
                control.steer = total_error;
                previous_error = current_error;
                control.steer = fmaxf(fminf(control.steer, 1.0f), -1.0f);    
                control.speed = SPEED_FAST;
    
                break;
            }
         default: {
                first_call = true;
                num_frame = 0;
                float factor = frameWidth / frameHeight;
                if (pixy.m0_x1 > pixy.m0_x0) {
                    x = (float)((float)pixy.m0_x1 - (float)pixy.m0_x0) ;
                    y = (float)((float)pixy.m0_y1 - (float)pixy.m0_y0) ;
                } else {
                    x = (float)((float)pixy.m0_x0 - (float)pixy.m0_x1) ;
                    y = (float)((float)pixy.m0_y0 - (float)pixy.m0_y1) ;
                }
                if (pixy.m0_x0 != pixy.m0_x1) {
                    float total_error = factor * x/y;
                    control.steer = fmaxf(fminf(total_error, 1.0f), -1.0f);
            
                 //propotional control of the speed  to the steering angle such as the minimum speed is 0.09 and the maximum is 0.13
                    control.speed = SPEED_NORMAL;
                    break;
                }
            }
        }
    } else {
        control.steer = 0.0f;
        control.speed = SPEED_STOP;
    }
    return control;
}