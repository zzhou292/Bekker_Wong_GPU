#include <cuda_device_runtime_api.h>
#include <driver_types.h>
#include <fstream>
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include <vector>
#include <cmath>
#include "BWTerrain.cuh"

BWTerrain::BWTerrain(float x_in, float y_in, float resolution_in) {
    x = x_in;
    y = y_in;
    resolution = resolution_in;
}

void BWTerrain::Initialize() {
    x_n_node = x / resolution + 1;
    y_n_node = y / resolution + 1;
    n_node = x_n_node * y_n_node;

    x_arr = new float[n_node];
    y_arr = new float[n_node];
    z_arr = new float[n_node];

    for (int i = 0; i < n_node; i++) {
        x_arr[i] = (i % x_n_node) * resolution;
        y_arr[i] = (int)(i / x_n_node) * resolution;
        z_arr[i] = 0.f;
    }

    // malloc GPU memory
    cudaMalloc((float**)&gpu_x_arr, n_node * sizeof(float));
    cudaMalloc((float**)&gpu_y_arr, n_node * sizeof(float));
    cudaMalloc((float**)&gpu_z_arr, n_node * sizeof(float));

    // copy CPU data to GPU
    cudaMemcpy(gpu_x_arr, x_arr, n_node * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(gpu_y_arr, y_arr, n_node * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(gpu_z_arr, z_arr, n_node * sizeof(float), cudaMemcpyHostToDevice);
}

void BWTerrain::Destroy() {
    cudaFree(gpu_x_arr);
    cudaFree(gpu_y_arr);
    cudaFree(gpu_z_arr);
}

void BWTerrain::Advance(float time_step, BWWheel* wheel) {
    float x_min = wheel->pos_x - wheel->Get_R();
    float x_max = wheel->pos_x + wheel->Get_R();

    float y_min = wheel->pos_y - wheel->Get_W() / 2.f;
    float y_max = wheel->pos_y + wheel->Get_W() / 2.f;

    // find all vertices in the region of the cylinder
    std::vector<int> active_idx = Util_Find_Active(x_min, x_max, y_min, y_max);

    std::cout << "num_active: " << active_idx.size() << std::endl;
    int* active_arr = active_idx.data();
    int size = active_idx.size();
    // std::cout << "active:" << size << stUtil_Find_Activendl;
    BWTerrain::Util_Compute_Internal_Force(active_arr, size, wheel);

    cudaMemcpy(x_arr, gpu_x_arr, n_node * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(y_arr, gpu_y_arr, n_node * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(z_arr, gpu_z_arr, n_node * sizeof(float), cudaMemcpyDeviceToHost);
}

void BWTerrain::WriteOutput(std::string FileName) {
    boost::filesystem::path dir("OUTPUT");

    if (!(boost::filesystem::exists(dir))) {
        std::cout << " Output Folder Doesn't Exists" << std::endl;

        if (boost::filesystem::create_directory(dir))
            std::cout << "....Successfully Created !" << std::endl;
    }

    // Create and open a text file
    std::ofstream OutOBJ("OUTPUT/" + FileName + ".obj");

    for (int i = 0; i < n_node; i++) {
        OutOBJ << "v"
               << " " << x_arr[i] << " " << y_arr[i] << " " << z_arr[i] << std::endl;
    }

    for (int j = 0; j < y_n_node - 1; j++) {
        for (int i = 0; i < x_n_node - 1; i++) {
            OutOBJ << "f"
                   << " " << j * x_n_node + i + 1 << " " << j * x_n_node + i + 1 + 1 << " "
                   << (j + 1) * x_n_node + i + 1 << std::endl;
            OutOBJ << "f"
                   << " " << j * x_n_node + i + 1 + 1 << " " << (j + 1) * x_n_node + i + 1 << " "
                   << (j + 1) * x_n_node + i + 1 + 1 << std::endl;
        }
    }

    // Close the file
    OutOBJ.close();
}

// Utility Funtions:
__global__ void Ker_Find_Active(float* gpu_x_in,
                                float* gpu_y_in,
                                float x_min,
                                float x_max,
                                float y_min,
                                float y_max,
                                int size,
                                bool* out_bool) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx > size)
        return;
    if (gpu_x_in[idx] >= x_min && gpu_x_in[idx] <= x_max && gpu_y_in[idx] >= y_min && gpu_y_in[idx] <= y_max)
        out_bool[idx] = true;
    __syncthreads();
}

// Note: This util function assumes that the out_idx array is a float array uninitialized
std::vector<int> BWTerrain::Util_Find_Active(float x_min, float x_max, float y_min, float y_max) {
    int block_size = 1024;
    int n_block = n_node / 1024 + 1;

    bool* out_bool = new bool[n_node];
    for (int i = 0; i < n_node; i++) {
        out_bool[i] = false;
    }
    bool* gpu_out_bool;
    cudaMalloc((bool**)&gpu_out_bool, n_node * sizeof(bool));
    cudaMemcpy(gpu_out_bool, out_bool, n_node * sizeof(bool), cudaMemcpyHostToDevice);

    Ker_Find_Active<<<n_block, block_size>>>(gpu_x_arr, gpu_y_arr, x_min, x_max, y_min, y_max, n_node, gpu_out_bool);

    cudaMemcpy(out_bool, gpu_out_bool, n_node * sizeof(bool), cudaMemcpyDeviceToHost);

    std::vector<int> idx_vec;

    for (int i = 0; i < n_node; i++) {
        if (out_bool[i] == true) {
            idx_vec.push_back(i);
        }
    }

    cudaFree(gpu_out_bool);

    return idx_vec;
}

// Utility Funtions:
__global__ void Ker_Compute_Force(float* gpu_x_in,
                                  float* gpu_y_in,
                                  float* gpu_z_in,
                                  int* active_idx,
                                  int idx_size,
                                  float pos_x,
                                  float pos_y,
                                  float pos_z,
                                  float r,
                                  float* gpu_out_force) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx > idx_size)
        return;

    // Current vertex z direction ray-casting
    float a = abs(gpu_x_in[active_idx[idx]]) - pos_x;
    float c = sqrt(pow(a, 2) + pow(r, 2));
    float wheel_z = r * (c - r) / c;
    float delta_z = gpu_z_in[active_idx[idx]] - wheel_z;

    gpu_z_in[active_idx[idx]] = wheel_z;

    // generate a fictitious force
    if (delta_z > 0)
        gpu_out_force[active_idx[idx]] = delta_z * 1000.f;

    __syncthreads();
}

// Utility function for z-direction ray casting and internal force computation
void BWTerrain::Util_Compute_Internal_Force(int* idx_arr, int idx_arr_size, BWWheel* wheel) {
    float* out_force = new float[n_node];
    float* gpu_out_force;

    int block_size = 1024;
    int n_block = idx_arr_size / 1024 + 1;

    for (int i = 0; i < n_node; i++) {
        out_force[i] = 0.f;
    }

    cudaMalloc((float**)&gpu_out_force, n_node * sizeof(float));

    cudaMemcpy(gpu_out_force, out_force, n_node * sizeof(float), cudaMemcpyHostToDevice);

    Ker_Compute_Force<<<n_block, block_size>>>(gpu_x_arr, gpu_y_arr, gpu_z_arr, idx_arr, idx_arr_size, wheel->pos_x,
                                               wheel->pos_y, wheel->pos_z, wheel->Get_R(), gpu_out_force);

    cudaMemcpy(out_force, gpu_out_force, n_node * sizeof(float), cudaMemcpyDeviceToHost);

    float sum_force = 0.f;
    for (int i = 0; i < n_node; i++) {
        sum_force += out_force[i];
    }
    std::cout << "sum_force:" << sum_force << std::endl;
    wheel->acc_z = sum_force / wheel->Get_M() - 9.8f;

    cudaFree(gpu_out_force);
}