class Propulsion
 {
   public:
   double rho = 1.2250;  // need fix. rho=model->airdata.rho
   double airspeed;
    last_letter_2_msgs::prop_wrenches prop_wrenches;
    Model *model;
    Propulsion(Model *);
    // ~Propulsion();
    void calcWrench();
    void calcAdditionalData();
    virtual void calcThrust()=0;
    virtual void calcTorque()=0;
 };

 class BeardEngine : public Propulsion
 {
   public:
   double s_prop, c_prop, k_motor, k_omega, k_t_p;
   BeardEngine(Model *);
   void initParam();
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