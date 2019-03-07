class Propulsion
{
public:
  double rho = 1.2250; // need fix. rho=model->airdata.rho
  double airspeed;
  geometry_msgs::Vector3 relativeWind;
  double normalWind;
  double rotationDir; // motor direction of rotation

  last_letter_2_msgs::prop_wrenches prop_wrenches;
  Model *model;
  Propulsion(Model *);
  // ~Propulsion();
  void rotateWind();
  void calcWrench();
  virtual void calcThrust() = 0;
  virtual void calcTorque() = 0;
};

#include "noEngine.hpp"
#include "beardEngine.hpp"
#include "electricEngine.hpp"
