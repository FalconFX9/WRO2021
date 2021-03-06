/**
 * Template for your project (led blinking).
 *
 * Author: Jaroslav Páral (jarekparal)
 */

#include "ev3api.h"
#include "app.h"
#include "kernel.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define ClearTimer(...)
#define ClearTimerMS(...)

SYSTIM TimerMS(int unused) {
    SYSTIM tim;
    get_tim(&tim);
    return tim;
}

const motor_port_t lm = EV3_PORT_B, rm = EV3_PORT_C;
const sensor_port_t gyro = EV3_PORT_1, color_left = EV3_PORT_2, color_right = EV3_PORT_3;
const colorid_t house = COLOR_RED;

float PID_controller(int reflected_light, float dt, float *integral, int *previous_error){
    const int target = 75;
    const float Kp=0.2, Ki=0.0, Kd=0.35;
    int error;
    float derivative;
    error = target - reflected_light;
    (*integral) += error;
    derivative = (error - (*previous_error));
    (*previous_error) = error;
    float output;
    output = Kp * error + Ki * (*integral) + Kd * derivative;
    return output;
}


void PID_for_cts(int counts, int power, int config){
    float integral = 0;
    int previous_error = 0;
    float tdt = 5;
    unsigned long last_time = TimerMS(0);
    unsigned long time;
    counts += ev3_motor_get_counts(lm);
    sensor_port_t color_sensor;
    if (config == 1){
        color_sensor = color_left;
    } else {
        color_sensor = color_right;
    }
    while (counts > ev3_motor_get_counts(lm)){
        //time = TimerMS(0);
        float dt = 1;
        //last_time = TimerMS(0);
        ev3_motor_steer(lm, rm, power, PID_controller(ev3_color_sensor_get_reflect(color_sensor), dt, &integral, &previous_error));
    }
    ev3_speaker_play_tone(4000, 1000);
}


void PID_to_line(int power, int config){
    float integral = 0;
    int previous_error = 0;
    float tdt = 5;
    int color;
    if (config == 0){
        color = ev3_color_sensor_get_color(color_right);
        while (color != COLOR_BLUE && color != COLOR_BLACK){
        ev3_motor_steer(lm, rm, power, PID_controller(ev3_color_sensor_get_reflect(color_left), 0, &integral, &previous_error));
        color = ev3_color_sensor_get_color(color_right);
        }
    } else {
        color = ev3_color_sensor_get_color(color_left);
        while (color != COLOR_BROWN && color != COLOR_BLACK){
        ev3_motor_steer(lm, rm, power, -PID_controller(ev3_color_sensor_get_reflect(color_right), 0, &integral, &previous_error));
        color = ev3_color_sensor_get_color(color_left);
        }
    }
    
    ev3_speaker_play_tone(4000, 1000);
}


void PID_to_color(int power, int config, int color){
    float integral = 0;
    int previous_error = 0;
    float tdt = 5;
    if (config == 0){
        while (ev3_color_sensor_get_color(color_right) != color){
        ev3_motor_steer(lm, rm, power, PID_controller(ev3_color_sensor_get_reflect(color_left), 0, &integral, &previous_error));
        }
    } else {
        while (ev3_color_sensor_get_color(color_left) != color){
        ev3_motor_steer(lm, rm, power, -PID_controller(ev3_color_sensor_get_reflect(color_right), 0, &integral, &previous_error));
        }
    }
    
    ev3_speaker_play_tone(4000, 1000);
}


float turn_PID(int degrees, float estimate, float *integral){
    const float Kp=0.6/(degrees/90.0), Ki=0.0001/(degrees/90.0);
    int error;
    error = degrees - estimate;
    (*integral) += error;
    float output;
    output = Kp * error + Ki * (*integral);
    return output;
}

void turn_IMU(int degrees, int steering){
    int init_degrees = ev3_gyro_sensor_get_angle(gyro);
    int lm_cts = ev3_motor_get_counts(lm);
    int rm_cts = ev3_motor_get_counts(rm);
    float power = 50.0;
    int rel_deg = 0;
    float estimated_orientation = 0;
    FILE *file;
    file = fopen("/test.txt", "a");
    int rel_mt_cts;
    float weighted_cts;
    rel_deg = ev3_gyro_sensor_get_angle(gyro) - init_degrees;
    estimated_orientation = 0;
    degrees = abs(degrees);
    float integral = 0;
    while (estimated_orientation < degrees - 5){
        rel_deg = ev3_gyro_sensor_get_angle(gyro) - init_degrees;
        if (steering < 0){
            rel_mt_cts = ev3_motor_get_counts(rm) - rm_cts;
        } else {
            rel_mt_cts = ev3_motor_get_counts(lm) - lm_cts;
        }
        weighted_cts = abs(rel_mt_cts) * (8.0/25.0) * (abs(steering)/100.0);
        if (weighted_cts - 5 < abs(rel_deg) < weighted_cts + 5){
            estimated_orientation = abs(rel_deg) * 0.2 + weighted_cts * 0.8;
        } else {
            estimated_orientation = weighted_cts;
        }
        
        power = turn_PID(degrees, estimated_orientation, &integral);
        fprintf(file, "%f;%d;%f;%f\n", estimated_orientation, rel_deg, weighted_cts/0.8, power);
        ev3_motor_steer(lm, rm, power, steering);
    }
    ev3_motor_stop(lm, true);
    ev3_motor_stop(rm, true);
    fclose(file);
    return;
}


void turn_180(int direction){
    turn_IMU(170, direction*100);
}

void turn_90(int direction){
    turn_IMU(80, direction*50);
}


void turn_CS(){
    return;
}


void sleep(unsigned long ms) {
    unsigned long time0;
    ClearTimerMS(0);
    ClearTimer(0);
    time0 = TimerMS(0);
    while ((TimerMS(0) - time0) < ms) { 
    }
}


void stop(bool brake){
    ev3_motor_stop(lm, brake);
    ev3_motor_stop(rm, brake);
}


void main_task(intptr_t unused) {
    ev3_motor_config(lm, MEDIUM_MOTOR);
    ev3_motor_config(rm, MEDIUM_MOTOR);
    ev3_sensor_config(gyro, GYRO_SENSOR);
    ev3_sensor_config(color_left, COLOR_SENSOR);
    ev3_sensor_config(color_right, COLOR_SENSOR);
    /*
    FILE *file;
    file = fopen("/test.txt", "a");
    while (3500 > ev3_motor_get_counts(lm)){
        ev3_motor_steer(rm, lm, 50, 0);
        fprintf(file, "%d\n", ev3_gyro_sensor_get_angle(gyro));
    }
    ev3_motor_stop(lm, true);
    ev3_motor_stop(rm, true);
    fclose(file);
    */
    PID_to_line(50, 1);
    stop(true);
    turn_90(1);
    PID_for_cts(2550, 50, 0);
    stop(false);
    turn_90(1);
    PID_to_color(50, 1, house);
    stop(true);
    turn_180(-1);
    stop(false);
    PID_to_line(50, 0);
    stop(true);
    turn_90(-1);
    stop(false);
    PID_for_cts(2200, 50, 1);
    PID_to_line(50, 0);
    stop(true);
    turn_90(-1);
    stop(false);
    PID_for_cts(1000, 50, 1);
    stop(true);
}
