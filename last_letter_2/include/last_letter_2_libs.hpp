#include <ros/ros.h>
#include <string>
#include <rosgraph_msgs/Clock.h>
#include <last_letter_2_msgs/get_model_states_srv.h>
#include <last_letter_2_msgs/get_control_signals_srv.h>
#include <last_letter_2_msgs/airdata_srv.h>
#include <last_letter_2_msgs/air_data.h>
#include <last_letter_2_msgs/control_signals.h>
#include <last_letter_2_msgs/model_states.h>
#include <last_letter_2_msgs/aero_wrenches.h>
#include <last_letter_2_msgs/prop_wrenches.h>
#include <geometry_msgs/Point.h>
#include <last_letter_2_msgs/model_wrenches.h>
#include <last_letter_2_msgs/apply_wrench_srv.h>
#include <std_srvs/Empty.h>
#include <math.h>
#include <ctime>

class Aerodynamics;
class Propulsion;
class Dynamics;
class Model;
class Master;
class Factory;
class Polynomial;

//////////
// Classes

class Polynomial
{
public:
	Polynomial();
	~Polynomial();
	virtual double evaluate() {return 0;}
	virtual double evaluate(double x) {return 0;}
	virtual double evaluate(double x, double y) {return 0;}
};

class Polynomial1D : public Polynomial
{
public:
	double coeffNo;
	double * coeffs;

	Polynomial1D(int maxOrder, double * coeffArray);
	~Polynomial1D();
	double evaluate(double x);
};

class Polynomial2D : public Polynomial
{
public:
	double coeffNo1, coeffNo2;
	double * coeffs;

	Polynomial2D(int maxOrder1, int maxOrder2, double * coeffArray);
	~Polynomial2D();
	double evaluate(double x, double y);
	// Important notes: maxOrder of variable y must be greater or equal to maxOrder of variable x
	// Pass the coefficient array with the following ordering (eg for maxOrder1=1, maxOrder2=3):
	// [00 01 02 03 10 11 12]
};

class Spline3 : public Polynomial
{
public:
	int breaksNo;
	double * breaks, * coeffs;

	Spline3(int breaksNoIn, double * breaksIn, double * coeffsIn);
	~Spline3();
	double evaluate(double x);
};