#include <cuda_device_runtime_api.h>
#include <cuda_runtime.h>
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
    // Initialize terrain size and area information
    x_n_node = x / resolution + 1;
    y_n_node = y / resolution + 1;
    n_node = x_n_node * y_n_node;
    area = resolution * resolution;

    x_arr = new float[n_node];
    y_arr = new float[n_node];
    z_arr = new float[n_node];

    for (int i = 0; i < n_node; i++) {
        x_arr[i] = (i % x_n_node) * resolution;
        y_arr[i] = (int)(i / x_n_node) * resolution;
        z_arr[i] = 0.f;
    }

    // initialize unified memory for Bekker soil parameter structure
    // set all parameters to default values
    cudaMallocManaged(&terrain_params, sizeof(BWParameters));
    terrain_params->Kphi = 0.2e6;
    terrain_params->Kc = 0;
    terrain_params->n = 1.1;
    terrain_params->f_s = 0.0;

    // malloc GPU memory for z array
    cudaMalloc((float**)&gpu_x_arr, n_node * sizeof(float));
    cudaMalloc((float**)&gpu_y_arr, n_node * sizeof(float));
    cudaMalloc((float**)&gpu_z_arr, n_node * sizeof(float));

    // copy CPU data to GPU
    cudaMemcpy(gpu_x_arr, x_arr, n_node * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(gpu_y_arr, y_arr, n_node * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(gpu_z_arr, z_arr, n_node * sizeof(float), cudaMemcpyHostToDevice);
}

void BWTerrain::Destroy() {
    // detroy all allocated GPU memory
    cudaFree(gpu_x_arr);
    cudaFree(gpu_y_arr);
    cudaFree(gpu_z_arr);
    cudaFree(terrain_params);
}

void BWTerrain::Advance(float time_step, BWWheel* wheel) {
    // calculate x boundary
    float x_min = wheel->pos_x - wheel->Get_R();
    float x_max = wheel->pos_x + wheel->Get_R();

    // calculate y boundary
    float y_min = wheel->pos_y - wheel->Get_W() / 2.f;
    float y_max = wheel->pos_y + wheel->Get_W() / 2.f;

    // find all vertices in the region of the cylinder - we call them active vertices
    std::vector<int> active_idx = Util_Find_Active(x_min, x_max, y_min, y_max);

    // convert std::vector into int array for CUDA
    int* active_arr = active_idx.data();
    int size = active_idx.size();

    // compute internal force
    BWTerrain::Util_Compute_Internal_Force(active_arr, size, wheel);

    // copy updated z array back to CPU
    cudaMemcpy(z_arr, gpu_z_arr, n_node * sizeof(float), cudaMemcpyDeviceToHost);
}

void BWTerrain::SetBWParams(BWParameters* params_in) {
    // update all terrain parameters based on the BWParameters input
    terrain_params->Kphi = params_in->Kphi;
    terrain_params->Kc = params_in->Kc;
    terrain_params->n = params_in->n;
    terrain_params->f_s = params_in->f_s;
}

void BWTerrain::WriteOutput(std::string FileName) {
    boost::filesystem::path dir("OUTPUT");

    if (!(boost::filesystem::exists(dir))) {
        std::cout << " Output Folder Doesn't Exists" << std::endl;

        if (boost::filesystem::create_directory(dir))
            std::cout << "....Successfully Created !" << std::endl;
    }

    // create and open an obj file
    std::ofstream OutOBJ("OUTPUT/" + FileName + ".obj");

    for (int i = 0; i < n_node; i++) {
        OutOBJ << "v"
               << " " << x_arr[i] << " " << y_arr[i] << " " << z_arr[i] << std::endl;
    }

    // write out all vertices and faces information
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

    // close the file
    OutOBJ.close();
}

// CUDA kernel call to find active vertices
__global__ void Ker_Find_Active(float* gpu_x_in,
                                float* gpu_y_in,
                                float x_min,
                                float x_max,
                                float y_min,
                                float y_max,
                                int size,
                                bool* out_bool) {
    // find the id for the current thread
    int idx = threadIdx.x + blockIdx.x * blockDim.x;

    // if out of range, quick rejection
    if (idx >= size)
        return;

    // if inside interest region, update out_bool arr
    if (gpu_x_in[idx] >= x_min && gpu_x_in[idx] <= x_max && gpu_y_in[idx] >= y_min && gpu_y_in[idx] <= y_max)
        out_bool[idx] = true;

    // waiting for all threads to finish
    __syncthreads();
}

// Wrapper for CUDA kernal call to find all active vertices
std::vector<int> BWTerrain::Util_Find_Active(float x_min, float x_max, float y_min, float y_max) {
    // default block size set to 1024
    int block_size = 1024;
    int n_block = n_node / block_size + 1;

    // create and copy an output boolean array into GPU memory
    bool* out_bool = new bool[n_node];
    for (int i = 0; i < n_node; i++) {
        out_bool[i] = false;
    }
    bool* gpu_out_bool;
    cudaMalloc((bool**)&gpu_out_bool, n_node * sizeof(bool));
    cudaMemcpy(gpu_out_bool, out_bool, n_node * sizeof(bool), cudaMemcpyHostToDevice);

    // call CUDA kernel
    Ker_Find_Active<<<n_block, block_size>>>(gpu_x_arr, gpu_y_arr, x_min, x_max, y_min, y_max, n_node, gpu_out_bool);

    // copy data back to cpu array
    cudaMemcpy(out_bool, gpu_out_bool, n_node * sizeof(bool), cudaMemcpyDeviceToHost);

    // store all active vertices in a std::vector
    std::vector<int> idx_vec;
    for (int i = 0; i < n_node; i++) {
        if (out_bool[i] == true) {
            idx_vec.push_back(i);
        }
    }

    // free temporary GPU memory
    cudaFree(gpu_out_bool);

    return idx_vec;
}

// CUDA kernel call to compute force based on Bekker-Wong Pressure-Sinkage Formulation
__global__ void Ker_Compute_Force(float* gpu_x_in,
                                  float* gpu_y_in,
                                  float* gpu_z_in,
                                  int* active_idx,
                                  int idx_size,
                                  float pos_x,
                                  float pos_y,
                                  float pos_z,
                                  float r,
                                  float area,
                                  float b,
                                  float* gpu_out_force,
                                  BWParameters* params_in) {
    // calculate the idx for the current thread
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    // if out of range, return
    if (idx >= idx_size)
        return;

    // current vertex z direction ray-casting
    float a = fabsf(gpu_x_in[active_idx[idx]]) - pos_x;
    float absz = sqrtf(powf(r, 2) - powf(a, 2));
    // ray-casting: the lowest z direction on the cylinderical wheel
    float wheel_z = pos_z - absz;
    float delta_z = gpu_z_in[active_idx[idx]] - wheel_z;
    // generate force based on BW formula
    if (delta_z > 0) {
        float p_pressure = (params_in->Kc / b + params_in->Kphi) * powf(delta_z, params_in->n);
        gpu_out_force[active_idx[idx]] = p_pressure * area;
        gpu_z_in[active_idx[idx]] = wheel_z;
    }

    // waiting for all threads to finish
    __syncthreads();
}

// Wrapper for CUDA kernal call to compute force based on Bekker-Wong Formulation
void BWTerrain::Util_Compute_Internal_Force(int* idx_arr, int idx_arr_size, BWWheel* wheel) {
    float* out_force = new float[n_node];
    int* gpu_idx_arr;
    float* gpu_out_force;

    int block_size = 1024;
    int n_block = idx_arr_size / 1024 + 1;

    for (int i = 0; i < n_node; i++) {
        out_force[i] = 0.f;
    }

    cudaMalloc((float**)&gpu_out_force, n_node * sizeof(float));
    cudaMalloc((int**)&gpu_idx_arr, idx_arr_size * sizeof(int));

    cudaMemcpy(gpu_out_force, out_force, n_node * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(gpu_idx_arr, idx_arr, idx_arr_size * sizeof(int), cudaMemcpyHostToDevice);

    float b = resolution * resolution / (resolution * 4);

    // call CUDA kernal
    Ker_Compute_Force<<<n_block, block_size>>>(gpu_x_arr, gpu_y_arr, gpu_z_arr, gpu_idx_arr, idx_arr_size, wheel->pos_x,
                                               wheel->pos_y, wheel->pos_z, wheel->Get_R(), area, b, gpu_out_force,
                                               terrain_params);

    cudaMemcpy(out_force, gpu_out_force, n_node * sizeof(float), cudaMemcpyDeviceToHost);

    float sum_force = 0.f;
    for (int i = 0; i < n_node; i++) {
        sum_force += out_force[i];
    }

    wheel->acc_z = sum_force / wheel->Get_M() - 9.8f;

    cudaFree(gpu_out_force);
    cudaFree(gpu_idx_arr);
}
