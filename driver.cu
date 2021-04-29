#include <cstdio>
#include <random>
#include <iostream>

#include "BWTerrain.cuh"
#include "BWWheel.cuh"

int main(int argc, char* argv[]) {
    BWTerrain terrain = BWTerrain(3.f, 3.f, 1.f);
    BWWheel wheel = BWWheel(1.f, 2.f);
    std::cout << "Terrain: " << terrain.Get_X_Size() << "," << terrain.Get_Y_Size() << "," << terrain.Get_Resolution()
              << std::endl;
    std::cout << "Wheel: " << wheel.Get_R() << "," << wheel.Get_W() << std::endl;
    terrain.Initialize();
    terrain.WriteOutput("test");
}