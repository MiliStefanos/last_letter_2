class Aerodynamics 
{
  public:
    float rho = 1.2250;  // need fix. rho=model->airdata.rho
    float M = 50;
    float oswald;
    float a0; 
    float airspeed, alpha, beta;

    last_letter_2_msgs::aero_wrenches aero_wrenches;
    Model *model;
    Aerodynamics(Model *);
    // ~Aerodynamics();
    void calcWrench();
    void calcAdditionalData();
    virtual void calcForces()=0;
    virtual void calcTorques()=0;
};

class NoAerodynamics : public Aerodynamics
{
  public:
    NoAerodynamics(Model *);
    void calcForces();
    void calcTorques();
};

class StdLinearAero : public Aerodynamics
{
  public:
  float c_drag_q, c_drag_deltae, c_drag_p, c_drag_0;
  float c_lift_0, c_lift_deltae, c_lift_q, c_lift_a;
  float b, c, s;
  float c_y_0, c_y_b, c_y_p, c_y_r, c_y_deltaa, c_y_deltar;
  float c_l_0, c_l_p, c_l_b, c_l_r, c_l_deltaa, c_l_deltar;
  float c_n_0, c_n_p, c_n_b, c_n_r, c_n_deltaa, c_n_deltar;
  float c_m_0, c_m_a, c_m_q, c_m_deltae;
  StdLinearAero(Model *);
  void initParam();
  void calcForces();
  void calcTorques();
};