#include <cstdio>
#include <random>
#include <iostream>

#include "BWTerrain.cuh"

int main(int argc, char* argv[]) {
    BWTerrain terrain = BWTerrain(3.f, 3.f, 0.5f);
    BWWheel wheel = BWWheel(1.f, 2.f);
    wheel.Initialize(0.5f, 1.f, 0.f);
    std::cout << "Terrain: " << terrain.Get_X_Size() << "," << terrain.Get_Y_Size() << "," << terrain.Get_Resolution()
              << std::endl;
    std::cout << "Wheel: " << wheel.Get_R() << "," << wheel.Get_W() << std::endl;
    terrain.Initialize();
    terrain.Advance(0.0001, &wheel);
    terrain.WriteOutput("test");
    terrain.Destroy();
}