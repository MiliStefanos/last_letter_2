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