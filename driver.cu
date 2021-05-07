// ME 759 Spring 2021 Final Project
// driver.cu
// Author: Jason Zhou

#include <cstdio>
#include <random>
#include <iostream>
#include <string>

#include "BWTerrain.cuh"

int main(int argc, char* argv[]) {
    // Note: Currently the minimum resolution tested to be working 0.05f
    // Experiments have shown that when goes below 0.05f, Bulldozing algorithm has flaws

    // declare a terrain with size 5 m x 5 m with resolution 0.05 m
    BWTerrain terrain = BWTerrain(5.f, 5.f, 0.05f);
    // declare a wheel with radius of 0.5 m, a width of 1 m, and a mass of 10 kg
    BWWheel wheel = BWWheel(0.5f, 1.f, 10.f);

    // initialize the wheel object
    wheel.Initialize(1.0f, 2.5f, 0.5f);
    // set z direction acceleration to -9.8, simulate gravity effect
    wheel.acc_z = -9.8;
    // single - wheel constant x direction velocity set to 0.5
    wheel.vel_x = 0.5;

    std::cout << "Terrain: " << terrain.Get_X_Size() << "," << terrain.Get_Y_Size() << "," << terrain.Get_Resolution()
              << std::endl;
    std::cout << "Wheel: " << wheel.Get_R() << "," << wheel.Get_W() << std::endl;

    // enable bulldozing effect - defaultly it's turned off
    terrain.Set_Bulldozing(false);

    // intialize the terrain
    terrain.Initialize();

    // perform simulation
    // step size set to 0.01 s
    // total simulation time is 5 s
    for (int i = 0; i < 500; i++) {
        wheel.Advance(0.01);
        terrain.Advance(0.01, &wheel);
        if (i < 10) {
            terrain.WriteOutput("ter-000" + std::to_string(i));
            wheel.WriteOutput("whe-000" + std::to_string(i));
        } else if (i < 100) {
            terrain.WriteOutput("ter-00" + std::to_string(i));
            wheel.WriteOutput("whe-00" + std::to_string(i));
        } else {
            terrain.WriteOutput("ter-0" + std::to_string(i));
            wheel.WriteOutput("whe-0" + std::to_string(i));
        }

        std::cout << "time: " << i * 0.01 << " s" << std::endl;
        std::cout << "wheel pos:" << wheel.pos_x << "," << wheel.pos_y << "," << wheel.pos_z << std::endl;
        std::cout << "wheel acc_z:" << wheel.acc_z << std::endl;
        std::cout << "==============================================" << std::endl;
    }

    // safely free up all GPU memory
    terrain.Destroy();
}