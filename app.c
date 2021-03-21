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


const motor_port_t flm = EV3_PORT_A, lm = EV3_PORT_B, rm = EV3_PORT_C, rlm = EV3_PORT_D;
const sensor_port_t color_middle = EV3_PORT_1, color_left = EV3_PORT_2, color_right = EV3_PORT_3, htcs = EV3_PORT_4;
const colorid_t house = COLOR_RED;

float PID_controller(int reflected_light, float pr, float *integral, int *previous_error){
    const int target = 25;
    const float Kp=0.35*pr, Ki=0.0, Kd=0.35*pr;
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


void PID_for_cts(int counts, int power, int side){
    FILE *file;
    file = fopen("/test.txt", "a");
    float integral = 0;
    int previous_error = 0;
    //long loops = 0;
    float pr = (50.0/power);
    //unsigned long last_time = TimerMS(0);
    //unsigned long time;
    counts += ev3_motor_get_counts(lm);
    float steering;
    int cts_lm = ev3_motor_get_counts(lm);
    int cts_rm = ev3_motor_get_counts(rm);
    while (counts > ev3_motor_get_counts(lm)){
        //time = TimerMS(0);
        //last_time = TimerMS(0);
        steering = PID_controller(ev3_color_sensor_get_reflect(color_middle), pr, &integral, &previous_error);
        //fprintf(file, "%d;%d\n", ev3_motor_get_counts(rm)-cts_rm, ev3_motor_get_counts(lm)-cts_lm);
        ev3_motor_steer(lm, rm, power, steering*side);
        tslp_tsk(5U*1000U);
        //loops++;
    }
    ev3_speaker_play_tone(4000, 1000);
    fclose(file);
}


void forward_for_cts(int counts, int power){
    counts += ev3_motor_get_counts(lm);
    while (counts > ev3_motor_get_counts(lm)){
        ev3_motor_steer(lm, rm, power, 0);
    }
}


void backwards_for_cts(int counts, int power){
    counts += ev3_motor_get_counts(lm);
    while (counts < ev3_motor_get_counts(lm)){
        ev3_motor_steer(lm, rm, -power, 0);
    }
}


void PID_to_line(int power, int config, int side){
    FILE *file;
    file = fopen("/test.txt", "a");
    float integral = 0;
    int previous_error = 0;
    float pr = (50.0/power);
    int color;
    float steering;
    //long loops = 0;
    if (config == 0){
        color = ev3_color_sensor_get_reflect(color_right);
        while (color > 20){
        steering = PID_controller(ev3_color_sensor_get_reflect(color_middle), pr, &integral, &previous_error) * side;
        //fprintf(file, "%f\n", steering);

        ev3_motor_steer(lm, rm, power, -steering);
        color = ev3_color_sensor_get_reflect(color_right);
        tslp_tsk(5U*1000U);
        //loops++;
        }
    } else {
        color = ev3_color_sensor_get_reflect(color_left); 
        while (color > 20){
        steering = PID_controller(ev3_color_sensor_get_reflect(color_middle), pr, &integral, &previous_error) * side;
        //fprintf(file, "%f\n", steering);

        ev3_motor_steer(lm, rm, power, -steering);
        color = ev3_color_sensor_get_reflect(color_left);
        tslp_tsk(5U*1000U);
        //loops++;
        }
    }
    fclose(file);
    ev3_speaker_play_tone(4000, 1000);
}


void PID_to_color(int power, int config, int color){
    float integral = 0;
    int previous_error = 0;
    float pr = (50.0/power)/2.0;
    if (config == 0){
        while (ev3_color_sensor_get_color(color_right) != color){
        ev3_motor_steer(lm, rm, power, PID_controller(ev3_color_sensor_get_reflect(color_left), pr, &integral, &previous_error));
        }
    } else {
        while (ev3_color_sensor_get_color(color_left) != color){
        ev3_motor_steer(lm, rm, power, -PID_controller(ev3_color_sensor_get_reflect(color_right), pr, &integral, &previous_error));
        }
    }
    
    ev3_speaker_play_tone(4000, 1000);
}


void go_to_line(int power, int config){
    int color;
    if (config == 0){
        color = ev3_color_sensor_get_reflect(color_right);
        while (color > 20){
        ev3_motor_steer(lm, rm, power, 0);
        color = ev3_color_sensor_get_reflect(color_right);
        }
    } else {
        color = ev3_color_sensor_get_reflect(color_left); 
        while (color > 20){
        ev3_motor_steer(lm, rm, power, 0);
        color = ev3_color_sensor_get_reflect(color_left);
        }
    }
}


float turn_PID(int degrees, float estimate, float *integral){
    const float Kp=0.6/(degrees/90.0), Ki=0.001/(degrees/90.0);
    int error;
    error = degrees - estimate;
    (*integral) += error;
    float output;
    output = Kp * error + Ki * (*integral);
    return output;
}

void turn_IMU(int degrees, int steering, int backwards){
    //int init_degrees = ev3_gyro_sensor_get_angle(gyro);
    int lm_cts = ev3_motor_get_counts(lm);
    int rm_cts = ev3_motor_get_counts(rm);
    float power = 50.0;
    int rel_deg = 0;
    float estimated_orientation = 0;
    //FILE *file;
    //file = fopen("/test.txt", "a");
    int rel_mt_cts;
    float weighted_cts;
    //rel_deg = ev3_gyro_sensor_get_angle(gyro) - init_degrees;
    estimated_orientation = 0;
    degrees = abs(degrees);
    float integral = 0;
    int log_opposite_wheel;
    float estimated_orientation_inside_wheel = 0;
    while (estimated_orientation < degrees - 5){
        //rel_deg = ev3_gyro_sensor_get_angle(gyro) - init_degrees;
        if (steering < 0){
            rel_mt_cts = ev3_motor_get_counts(rm) - rm_cts;
            log_opposite_wheel = ev3_motor_get_counts(lm) - lm_cts;
            if (log_opposite_wheel >= 0 && backwards == 1){
                rm_cts = ev3_motor_get_counts(rm);
            }
        } else {
            rel_mt_cts = ev3_motor_get_counts(lm) - lm_cts;
            log_opposite_wheel = ev3_motor_get_counts(rm) - rm_cts;
            if (log_opposite_wheel >= 0 && backwards == 1){
                lm_cts = ev3_motor_get_counts(lm);
            }
        }
        weighted_cts = abs(rel_mt_cts) * (8.0/25.0) * (abs(steering)/100.0);
        estimated_orientation = weighted_cts;
        estimated_orientation_inside_wheel = (abs(log_opposite_wheel)/((abs(steering)-50)/50.0)) * (8.0/25.0) * (abs(steering)/100.0);
        power = turn_PID(degrees, estimated_orientation, &integral)*backwards;
        //fprintf(file, "%f;%f;%d;%d\n", estimated_orientation, estimated_orientation_inside_wheel, rel_mt_cts, log_opposite_wheel);
        tslp_tsk(1000U); //for some fucking reason if the loop runs at full speed it shits itself
        ev3_motor_steer(lm, rm, power, steering);
    }
    ev3_motor_stop(lm, true);
    ev3_motor_stop(rm, true);
    //fclose(file);
    return;
}


void turn_180(int direction){
    turn_IMU(170, direction*100, 1);
}

void turn_90(int direction){
    tslp_tsk(350U * 1000U);
    turn_IMU(80 , direction*70, 1);
}


void turn_90b(int direction){
    turn_IMU(90, direction*60, -1);
}



void stop(bool brake){
    ev3_motor_stop(lm, brake);
    ev3_motor_stop(rm, brake);
}


void grab(){
    ev3_motor_rotate(flm, -280, 30, true);
}

void turn_CS(int direction){
    while (ev3_color_sensor_get_reflect(color_middle) > 20){
        ev3_motor_steer(lm, rm, 50, 70*direction);
    }
    stop(true);
    return;
}

void get_green(){
    ev3_motor_rotate(flm, -95, 10, true);
    forward_for_cts(90, 20);
    stop(true);
    ev3_motor_rotate(flm, -65, 10, true);
    backwards_for_cts(-90, 20);
    stop(true);
    ev3_motor_rotate(flm, -75, 10, true);
    forward_for_cts(90, 20);
    stop(true);
    ev3_motor_rotate(flm, -45, 10, true);
}


void get_yB(){
    forward_for_cts(200, 20);
    stop(true);
    ev3_motor_rotate(flm, -280, 30, true);
    backwards_for_cts(-300, 40);
    stop(true);
}


void get_colors(){
    //FILE *file;
    //file = fopen("/test.txt", "a");
    rgb_raw_t rgb;
    //fprintf(file, "Start:");
    ht_nxt_color_sensor_measure_rgb(htcs, &rgb);
    tslp_tsk(50U * 1000U);
    ht_nxt_color_sensor_measure_rgb(htcs, &rgb);
    //fprintf(file, "%d;%d;%d\n", rgb.r, rgb.g, rgb.b);
    
    forward_for_cts(105, 30);
    stop(true);
    tslp_tsk(100U * 1000U);
    ht_nxt_color_sensor_measure_rgb(htcs, &rgb);
    tslp_tsk(50U * 1000U);
    ht_nxt_color_sensor_measure_rgb(htcs, &rgb);
    //fprintf(file, "%d;%d;%d\n", rgb.r, rgb.g, rgb.b);
    //fclose(file);
    /*
    if (rgb.b > rgb.g && rgb.b > rgb.r){
        ev3_speaker_play_tone(500, 250);
    } else if (rgb.g > rgb.b && rgb.g > rgb.r){
        ev3_speaker_play_tone(2000, 1000);
    } else {
        if (rgb.g > 5 || rgb.r > 5) {
            ev3_speaker_play_tone(6000, 2000);
        }
    }
    */
    
}


void main_task(intptr_t unused) {
    ev3_motor_config(lm, MEDIUM_MOTOR);
    ev3_motor_config(rm, MEDIUM_MOTOR);
    ev3_motor_config(flm, MEDIUM_MOTOR);
    ev3_sensor_config(color_middle, COLOR_SENSOR);
    ev3_sensor_config(color_left, COLOR_SENSOR);
    ev3_sensor_config(color_right, COLOR_SENSOR);
    ev3_sensor_config(htcs, HT_NXT_COLOR_SENSOR);
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
    PID_to_line(50, 1);
    stop(true);
    turn_90(1);
    PID_for_cts(2550, 50, -1);
    stop(false);
    turn_90(1);
    stop(true);
    PID_to_color(50, 1, house);
    stop(true);
    turn_180(-1);
    stop(false);
    PID_to_line(50, 0);
    stop(true);
    turn_90(-1);
    stop(false);
    PID_for_cts(2200, 50);
    PID_to_line(50, 0);
    stop(true);
    turn_90(-1);
    stop(false);
    PID_for_cts(1000, 50);
    stop(true);
    */
    //get_green();
    forward_for_cts(300, 50);
    stop(true);
    PID_to_line(50, 1, 1);
    stop(true);
    turn_90(1);
    PID_to_line(35, 0, 1);
    stop(true);
    turn_90(-1);
    PID_for_cts(120, 20, -1);
    stop(true);
    turn_90(-1);
    forward_for_cts(140, 20);
    stop(true);
    tslp_tsk(100U * 1000U);
    get_colors();
    turn_90b(1);
    go_to_line(40, 1);
    forward_for_cts(50, 40);
    stop(true);
    //tslp_tsk(500U * 1000U);
    //turn_CS(-1);
    turn_90(-1);
    tslp_tsk(2000U * 1000U);
    PID_for_cts(200, 50, -1);
    PID_to_line(50, 0, -1);
    stop(true);
    //tslp_tsk(500U * 1000U);
    turn_90(-1);
}
