#include <cstdio>
#include <random>
#include <iostream>

#include "BWTerrain.cuh"
#include "BWWheel.cuh"

int main(int argc, char* argv[]) {
    BWTerrain terrain = BWTerrain(10.f, 10.f, 0.05f);
    BWWheel wheel = BWWheel(1.f, 2.f);
    std::cout << "Terrain: " << terrain.Get_X_Size() << "," << terrain.Get_Y_Size() << "," << terrain.Get_Resolution()
              << std::endl;
    std::cout << "Wheel: " << wheel.Get_R() << "," << wheel.Get_W() << std::endl;
}