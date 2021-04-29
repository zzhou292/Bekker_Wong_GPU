class BWTerrain
{
private:
  float x; // x dimension of the BW Terrain
  float y; // y dimension of the BW Terrain
  float resolution;	// Terrain resolution
public:
	BWTerrain(float x_in, float y_in, float resolution_in);
    float Get_X_Size() { return x; }
    float Get_Y_Size() { return y; }
    float Get_Resolution() { return resolution; }
};