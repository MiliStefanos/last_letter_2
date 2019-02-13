#include <last_letter_2_libs.hpp>


class Dynamics
{
  public:
  Model * model;
  Aerodynamics *aerodynamics;
  Propulsion * propulsion;
  Dynamics(Model *);
  void calcAero();
  void calcProp();
};

class Aerodynamics 
{
  public:
    float rho = 1.2250;  // need fix. rho=model->airdata.rho
    float M = 50;
    float oswald;
    float a0; 
    float c_drag_q, c_drag_deltae, c_drag_p, c_drag_0;
    float c_lift_0, c_lift_deltae, c_lift_q, c_lift_a;
    float b, c, s;
    float c_y_0, c_y_b, c_y_p, c_y_r, c_y_deltaa, c_y_deltar;
    float c_l_0, c_l_p, c_l_b, c_l_r, c_l_deltaa, c_l_deltar;
    float c_n_0, c_n_p, c_n_b, c_n_r, c_n_deltaa, c_n_deltar;
    float c_m_0, c_m_a, c_m_q, c_m_deltae;
    float airspeed, alpha, beta;

    last_letter_2_msgs::aero_wrenches aero_wrenches;
    Model *model;
    Aerodynamics(Model *);
    // ~Aerodynamics();
    void initParam();
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
  StdLinearAero(Model *);
  void calcForces();
  void calcTorques();
};

 class Propulsion
 {
   public:
   float rho = 1.2250;  // need fix. rho=model->airdata.rho
   float s_prop, c_prop, k_motor, k_omega, k_t_p;
   float airspeed;
    last_letter_2_msgs::prop_wrenches prop_wrenches;
    Model *model;
    Propulsion(Model *);
    // ~Propulsion();
    void initParam();
    void calcWrench();
    void calcAdditionalData();
    virtual void calcThrust()=0;
    virtual void calcTorque()=0;
 };

 class BeardEngine : public Propulsion
 {
   public:
   BeardEngine(Model *);
   void calcThrust();
   void calcTorque();
 };

class NoEngine : public Propulsion
 {
   public:
   NoEngine(Model *);
   void calcThrust();
   void calcTorque();
 };

class Model
{
  public:
    last_letter_2_msgs::model_states model_states;
    last_letter_2_msgs::air_data airdata;
    last_letter_2_msgs::control_signals control_signals;
    last_letter_2_msgs::model_wrenches model_wrenches;
    
    ros::NodeHandle nh;
    ros::ServiceClient states_client;
    ros::ServiceClient control_signals_client;
    ros::ServiceClient airdata_client;
    ros::ServiceClient apply_wrench_client;
    ros::ServiceClient sim_step_client;
    ros::ServiceClient pauseGazebo;

    ros::Publisher signals_publisher;
    last_letter_2_msgs::get_model_states_srv states_srv;
    last_letter_2_msgs::get_control_signals_srv signals_srv;
    last_letter_2_msgs::airdata_srv air_data;
    last_letter_2_msgs::apply_wrench_srv apply_wrench_srv;

    Dynamics dynamics;
    // Airdata airdata;
    Model();
    void modelStep();
    void getStates();
    void getControlSignals();
    void getAirdata();
    void calcWrenches();
    void applyWrenches();
    void simulationStep();
};

class Master
{
  public:
    // Model model;
    ros::NodeHandle nh;
    ros::Subscriber gazebo_sub;
    Model model;
    Master();
    // ~Master();
    void gazeboClockClb(const rosgraph_msgs::Clock::ConstPtr&);
};

class Factory
{
    public:
    Aerodynamics * buildAerodynamics(Model *);
    Propulsion * buildPropulsion(Model *);
};
