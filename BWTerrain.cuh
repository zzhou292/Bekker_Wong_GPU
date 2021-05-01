#include <math.h>
#include <string>
#include "BWWheel.cuh"

class BWTerrain {
  private:
    float x;           // x dimension of the BW Terrain
    float y;           // y dimension of the BW Terrain
    float resolution;  // Terrain resolution

    float* x_arr;  // cpu pos x arr
    float* y_arr;  // cpu pos y arr
    float* z_arr;  // cpu pos z arr

    float* gpu_x_arr;  // gpu pos x arr
    float* gpu_y_arr;  // gpu pos y arr
    float* gpu_z_arr;  // gpu pos z arr

    int n_node;
    int x_n_node;
    int y_n_node;

    // private utility functions
    std::vector<int> Util_Find_Active(float x_min, float x_max, float y_min, float y_max);
    void Util_Compute_Internal_Force(int* idx_arr, int idx_arr_size, BWWheel* wheel);

  public:
    BWTerrain(float x_in, float y_in, float resolution_in);
    float Get_X_Size() { return x; }
    float Get_Y_Size() { return y; }
    float Get_Resolution() { return resolution; }

    void Initialize();
    void WriteOutput(std::string FileName);
    void Advance(float time_step, BWWheel* wheel);

    void Destroy();  // safely free memory from GPU
};