#include <cstdio>
#include <random>
#include <iostream>
#include <string>

#include "BWTerrain.cuh"

int main(int argc, char* argv[]) {
    BWTerrain terrain = BWTerrain(10.f, 5.f, 0.1f);
    BWWheel wheel = BWWheel(0.5f, 1.f, 10.f);
    wheel.Initialize(1.0f, 2.5f, 1.2f);
    wheel.acc_z = -9.8;
    wheel.vel_x = 0.5;
    std::cout << "Terrain: " << terrain.Get_X_Size() << "," << terrain.Get_Y_Size() << "," << terrain.Get_Resolution()
              << std::endl;
    std::cout << "Wheel: " << wheel.Get_R() << "," << wheel.Get_W() << std::endl;
    terrain.Initialize();
    for (int i = 0; i < 2; i++) {
        wheel.Advance(0.001);
        terrain.Advance(0.001, &wheel);
        // terrain.WriteOutput("test" + std::to_string(i));
        // std::cout << "pos:" << wheel.pos_x << "," << wheel.pos_y << "," << wheel.pos_z << std::endl;
        std::cout << "acc_z:" << wheel.acc_z << std::endl;
    }
    terrain.Destroy();
}