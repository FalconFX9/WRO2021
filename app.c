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

#define ClearTimer(...)
#define ClearTimerMS(...)

SYSTIM TimerMS(int unused) {
    SYSTIM tim;
    get_tim(&tim);
    return tim;
}

const motor_port_t lm = EV3_PORT_B, rm = EV3_PORT_C;
const sensor_port_t gyro = EV3_PORT_1, color_left = EV3_PORT_2;

float PID_controller(int reflected_light, float dt, float *integral, int *previous_error){
    const int target = 35;
    const float Kp=0.3, Ki=0.0, Kd=0.2;
    int error;
    float derivative;
    error = target - reflected_light;
    (*integral) += error;
    if ((*integral) >= 19){
        (*integral) -= 19;
    } else if ((*integral) <= -19){
        (*integral) += 19;
    }
    derivative = (error - (*previous_error));
    (*previous_error) = error;
    float output;
    output = Kp * error + Ki * (*integral) + Kd * derivative;
    return output;
}


void PID_for_cts(int counts, int power){
    float integral = 0;
    int previous_error = 0;
    float tdt = 5;
    unsigned long last_time = TimerMS(0);
    unsigned long time;
    while (counts > ev3_motor_get_counts(lm)){
        //time = TimerMS(0);
        float dt = 1;
        //last_time = TimerMS(0);
        ev3_motor_steer(lm, rm, power, PID_controller(ev3_color_sensor_get_reflect(color_left), dt, &integral, &previous_error));
    }
    ev3_speaker_play_tone(4000, 1000);
    ev3_motor_stop(lm, true);
    ev3_motor_stop(rm, true);
}


void turn_IMU(int degrees){
    int init_degrees = ev3_gyro_sensor_get_angle(gyro);
    int lm_cts = ev3_motor_get_counts(lm);
    int rm_cts = ev3_motor_get_counts(rm);
    int power = 50;
    int rel_deg = 0;
    float estimated_orientation = 0;
    FILE *file;
    file = fopen("/test.txt", "a");
    int rel_mt_cts;
    float weighted_cts;
    if (degrees < 0){
        rel_deg = ev3_gyro_sensor_get_angle(gyro) - init_degrees;
        estimated_orientation = abs(rel_deg) * 0.5 + (abs(ev3_motor_get_counts(rm) - rm_cts) * (3.0/11.0)) * 0.5;
        ev3_motor_steer(lm, rm, power, -100);
        fprintf(file, "%f1st\n", estimated_orientation);
        degrees = abs(degrees);
        while (estimated_orientation < degrees - 5){
            ev3_motor_steer(lm, rm, power, -100);
            rel_deg = ev3_gyro_sensor_get_angle(gyro) - init_degrees;
            rel_mt_cts = ev3_motor_get_counts(rm) - rm_cts;
            weighted_cts = abs(rel_mt_cts) * (3.0/22.0);
            estimated_orientation = abs(rel_deg) * 0.5 + weighted_cts;
            fprintf(file, "%f;%d;%d;%f\n", estimated_orientation, rel_deg, rel_mt_cts, weighted_cts);
        }
        ev3_motor_stop(lm, true);
        ev3_motor_stop(rm, true);
        fclose(file);
    }
    return;
}

void turn_CS(){
    return;
}

void main_task(intptr_t unused) {
    ev3_motor_config(lm, MEDIUM_MOTOR);
    ev3_motor_config(rm, MEDIUM_MOTOR);
    ev3_sensor_config(gyro, GYRO_SENSOR);
    ev3_sensor_config(color_left, COLOR_SENSOR);

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
    PID_for_cts(2500, 50);
    turn_IMU(-90);
}
