class BWWheel {
  private:
    float r;  // radius of the cylinderical wheel
    float w;  // width of the cylinderical wheel
  public:
    BWWheel(float r_in, float w_in);
    float Get_R() { return r; }
    float Get_W() { return w; }
};