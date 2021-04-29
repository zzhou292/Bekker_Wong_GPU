#include <fstream>
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
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
        z_arr[i] = 0;
    }
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