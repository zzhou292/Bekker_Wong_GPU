// ME 759 Spring 2021 Final Project
// BWWheel.cu
// Author: Jason Zhou
#include "BWWheel.cuh"
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

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

void BWWheel::WriteOutput(std::string FileName) {
    // Utility function to write out mesh representaion of the BWTerrain
    boost::filesystem::path dir("OUTPUT");
    if (!(boost::filesystem::exists(dir))) {
        std::cout << " Output Folder Doesn't Exists" << std::endl;

        if (boost::filesystem::create_directory(dir))
            std::cout << "....Successfully Created !" << std::endl;
    }

    // create and open an obj file
    std::ofstream OutCSV("OUTPUT/" + FileName + ".csv");

    OutCSV << pos_x << "," << pos_y << "," << pos_z << std::endl;

    // close the file
    OutCSV.close();
}
