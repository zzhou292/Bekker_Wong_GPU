// ME 759 Spring 2021 Final Project
// BWWheel.cuh
// Author: Jason Zhou

#include <math.h>
#include <string>

class BWWheel {
  private:
    float r;  // radius of the cylinderical wheel
    float w;  // width of the cylinderical wheel
    float m;  // mass of the cylinderical wheel
  public:
    float pos_x;
    float pos_y;
    float pos_z;

    float vel_x;
    float vel_y;
    float vel_z;

    float acc_x;
    float acc_y;
    float acc_z;

    BWWheel(float r_in, float w_in, float m_in);
    float Get_R() { return r; }
    float Get_W() { return w; }
    float Get_M() { return m; }

    void Initialize(float pos_x_in, float pos_y_in, float pos_z_in);
    void Advance(float time_step);
    void WriteOutput(std::string FileName);
};