#include "BWWheel.cuh"

BWWheel::BWWheel(float r_in, float w_in, float m_in) {
    r = r_in;
    w = w_in;
    m = m_in;
}

void BWWheel::Initialize(float pos_x_in, float pos_y_in, float pos_z_in) {
    pos_x = pos_x_in;
    pos_y = pos_y_in;
    pos_z = pos_z_in;

    vel_x = 0.f;
    vel_y = 0.f;
    vel_z = 0.f;

    acc_x = 0.f;
    acc_y = 0.f;
    acc_z = 0.f;
}

void BWWheel::Advance(float time_step) {
    vel_x = vel_x + acc_x * time_step;
    vel_y = vel_y + acc_y * time_step;
    vel_z = vel_z + acc_z * time_step;

    pos_x = pos_x + vel_x * time_step;
    pos_y = pos_y + vel_y * time_step;
    pos_z = pos_z + vel_z * time_step;
}
