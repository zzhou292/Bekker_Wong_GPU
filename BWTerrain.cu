#include <cuda_device_runtime_api.h>
#include <cuda_runtime.h>
#include <driver_types.h>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
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
    terrain_params->bz_ratio = 0.2;

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
    if (enable_bulldozing)
        displacement_arr = new float[size];

    // compute internal force
    BWTerrain::Util_Compute_Internal_Force(active_arr, size, wheel, displacement_arr);

    // copy updated z array back to CPU
    cudaMemcpy(z_arr, gpu_z_arr, n_node * sizeof(float), cudaMemcpyDeviceToHost);

    // compute bulldozing force
    if (enable_bulldozing) {
        BWTerrain::Util_Compute_Bulldozing(active_arr, displacement_arr, size);
    }

    // copy updated z array back to CPU
    cudaMemcpy(z_arr, gpu_z_arr, n_node * sizeof(float), cudaMemcpyDeviceToHost);
}

void BWTerrain::SetBWParams(BWParameters* params_in) {
    // update all terrain parameters based on the BWParameters input
    terrain_params->Kphi = params_in->Kphi;
    terrain_params->Kc = params_in->Kc;
    terrain_params->n = params_in->n;
    terrain_params->f_s = params_in->f_s;
    terrain_params->bz_ratio = params_in->bz_ratio;
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
                                  float* gpu_displacement_arr,
                                  int idx_size,
                                  float pos_x,
                                  float pos_y,
                                  float pos_z,
                                  float r,
                                  float area,
                                  float b,
                                  float* gpu_out_force,
                                  BWParameters* params_in,
                                  bool bulldozing) {
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
        if (bulldozing)
            gpu_displacement_arr[idx] = delta_z;
    }

    // waiting for all threads to finish
    __syncthreads();
}

// Wrapper for CUDA kernal call to compute force based on Bekker-Wong Formulation
void BWTerrain::Util_Compute_Internal_Force(int* idx_arr, int idx_arr_size, BWWheel* wheel, float* displacement_arr) {
    float* out_force = new float[n_node];
    int* gpu_idx_arr;
    float* gpu_out_force;

    float* gpu_displacement_arr;

    int block_size = 1024;
    int n_block = idx_arr_size / 1024 + 1;

    for (int i = 0; i < n_node; i++) {
        out_force[i] = 0.f;
    }

    cudaMalloc((float**)&gpu_out_force, n_node * sizeof(float));
    cudaMalloc((int**)&gpu_idx_arr, idx_arr_size * sizeof(int));
    if (enable_bulldozing)
        cudaMalloc((float**)&gpu_displacement_arr, idx_arr_size * sizeof(float));

    cudaMemcpy(gpu_out_force, out_force, n_node * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(gpu_idx_arr, idx_arr, idx_arr_size * sizeof(int), cudaMemcpyHostToDevice);

    float b = resolution * resolution / (resolution * 4);

    // call CUDA kernal
    Ker_Compute_Force<<<n_block, block_size>>>(gpu_x_arr, gpu_y_arr, gpu_z_arr, gpu_idx_arr, gpu_displacement_arr,
                                               idx_arr_size, wheel->pos_x, wheel->pos_y, wheel->pos_z, wheel->Get_R(),
                                               area, b, gpu_out_force, terrain_params, enable_bulldozing);

    cudaMemcpy(out_force, gpu_out_force, n_node * sizeof(float), cudaMemcpyDeviceToHost);
    if (enable_bulldozing)
        cudaMemcpy(displacement_arr, gpu_displacement_arr, idx_arr_size * sizeof(float), cudaMemcpyDeviceToHost);

    float sum_force = 0.f;
    for (int i = 0; i < n_node; i++) {
        sum_force += out_force[i];
    }

    wheel->acc_z = sum_force / wheel->Get_M() - 9.8f;

    cudaFree(gpu_out_force);
    cudaFree(gpu_idx_arr);
    if (enable_bulldozing)
        cudaFree(gpu_displacement_arr);
}

__global__ void Ker_Get_Bz_Neighbours(int* idx_in, int* idx_out, int size, int x_node_num, int y_node_num) {
    // calculate the idx for the current thread
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    // if out of range, return
    if (idx >= size)
        return;

    int y_idx = (int)idx_in[idx] / x_node_num;
    int x_idx = idx_in[idx] % x_node_num;

    int x_idx_1 = x_idx + 1;
    int x_idx_2 = x_idx - 1;
    int y_idx_1 = y_idx + 1;
    int y_idx_2 = y_idx - 1;

    if (x_idx_1 >= x_node_num) {
        idx_out[4 * idx] = -1;
    } else {
        // idx_out[4 * idx] = y_idx * x_node_num + x_idx_1;
        idx_out[4 * idx] = idx_in[idx] + 1;
    }

    if (x_idx_2 < 0) {
        idx_out[4 * idx + 1] = -1;
    } else {
        // idx_out[4 * idx + 1] = y_idx * x_node_num + x_idx_2;
        idx_out[4 * idx + 1] = idx_in[idx] - 1;
    }

    if (y_idx_1 >= y_node_num) {
        idx_out[4 * idx + 2] = -1;
    } else {
        // idx_out[4 * idx + 2] = y_idx_1 * x_node_num + x_idx;
        idx_out[4 * idx + 2] = idx_in[idx] + x_node_num;
    }

    if (y_idx_2 < 0) {
        idx_out[4 * idx + 3] = -1;
    } else {
        // idx_out[4 * idx + 3] = y_idx_2 * x_node_num + x_idx;
        idx_out[4 * idx + 3] = idx_in[idx] - x_node_num;
    }

    __syncthreads();
}

__global__ void Ker_Bz_Raise_Neighbour(int* idx_in, int size, float* gpu_z_arr, float avg_soil_raise, float bz_ratio) {
    // calculate the idx for the current thread
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    // if out of range, return
    if (idx >= size)
        return;

    gpu_z_arr[idx_in[idx]] += bz_ratio * avg_soil_raise;
}

void BWTerrain::Util_Compute_Bulldozing(int* idx_in, float* displacement_in, int active_size) {
    float tot_displacement = 0.f;
    std::vector<int> hit_vertices;
    for (int i = 0; i < active_size; i++) {
        if (displacement_in[i] > 0) {
            hit_vertices.push_back(idx_in[i]);
            tot_displacement += displacement_in[i];
        }
    }

    int hit_size = hit_vertices.size();

    if (hit_size == 0) {
        return;
    }

    int* hit_arr = hit_vertices.data();

    int* gpu_idx_in;   // input index array, including all hit vertices from the last loop
    int* gpu_idx_out;  // output index array containing the neighbour of all 'hit_size' vertices
    int* neighbour_idx = new int[4 * hit_size];

    cudaMalloc((int**)&gpu_idx_in, hit_size * sizeof(int));
    cudaMalloc((int**)&gpu_idx_out, 4 * hit_size * sizeof(int));

    cudaMemcpy(gpu_idx_in, hit_arr, hit_size * sizeof(int), cudaMemcpyHostToDevice);

    int block_size = 1024;
    int n_block = hit_size / 1024 + 1;

    Ker_Get_Bz_Neighbours<<<n_block, block_size>>>(gpu_idx_in, gpu_idx_out, hit_size, x_n_node, y_n_node);

    cudaMemcpy(neighbour_idx, gpu_idx_out, 4 * hit_size * sizeof(int), cudaMemcpyDeviceToHost);

    std::vector<int> raw_neighbour;
    for (int i = 0; i < 4 * hit_size; i++) {
        raw_neighbour.push_back(neighbour_idx[i]);
    }

    std::sort(raw_neighbour.begin(), raw_neighbour.end());

    std::vector<int>::iterator ip;
    ip = std::unique(raw_neighbour.begin(), raw_neighbour.begin() + raw_neighbour.size());
    raw_neighbour.resize(std::distance(raw_neighbour.begin(), ip));

    if (raw_neighbour[0] == -1) {
        raw_neighbour.erase(raw_neighbour.begin() + 0);
    }

    cudaFree(gpu_idx_in);
    cudaFree(gpu_idx_out);

    int neigh_size = raw_neighbour.size();
    int* neigh_arr = raw_neighbour.data();

    int* gpu_neigh_arr;

    cudaMalloc((int**)&gpu_neigh_arr, neigh_size * sizeof(int));
    cudaMemcpy(gpu_neigh_arr, neigh_arr, neigh_size * sizeof(int), cudaMemcpyHostToDevice);

    float avg_soil_raise = tot_displacement / neigh_size;

    // Raise Neighourbing Nodes
    Ker_Bz_Raise_Neighbour<<<n_block, block_size>>>(gpu_neigh_arr, neigh_size, gpu_z_arr, avg_soil_raise,
                                                    terrain_params->bz_ratio);

    cudaFree(gpu_neigh_arr);
}
