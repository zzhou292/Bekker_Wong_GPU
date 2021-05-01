#include <math.h>
#include <string>
#include "BWWheel.cuh"

typedef struct {
    // The b constant in Bekker-Wong terrain formulation is approximated as 2*area/perimeter
    float Kphi;  // Bekker Kphi
    float Kc;    // Bekker Kc
    float n;     // Bekker n exponent
    float f_s;   // static friction coefficient - this coefficient is only an add-on, if set to 0 then only z direction
                 // is considered.
} BWParameters;

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

    float area;

    int n_node;
    int x_n_node;
    int y_n_node;

    BWParameters* terrain_params;

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

    void SetBWParams(BWParameters* params_in);

    void Destroy();  // safely free memory from GPU
};
