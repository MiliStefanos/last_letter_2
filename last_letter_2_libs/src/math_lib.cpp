#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <cstdio>
#include <cmath>

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


////////////////////
// Class Definitions
////////////////////

////////////////////
// Define Polynomial

Polynomial::Polynomial ()
{
};

Polynomial::~Polynomial ()
{
};


//////////////////////
// Define Polynomial1D

// class constructor
Polynomial1D::Polynomial1D (int maxOrder, double * coeffArray) : Polynomial()
{
    int i;
    coeffNo = maxOrder;
    // Create and initialize polynomial coefficients container
    coeffs = (double*)malloc(sizeof(double) * (coeffNo+1));
    for (i=0; i<=coeffNo; i++) {
        coeffs[i] = coeffArray[i];
    }
}

// class destructor
Polynomial1D::~Polynomial1D ()
{
    free(coeffs);
}

// polynomial evaluation
double Polynomial1D::evaluate (double x) {
    int i;
    double sum=0;
    for (i=0; i<=coeffNo; i++) {
        sum += coeffs[i]*pow(x,i);
    }
    return sum;
}


//////////////////////
// Define Polynomial2D

// class constructor
Polynomial2D::Polynomial2D (int maxOrder1, int maxOrder2, double * coeffArray) : Polynomial()
{
    // Attention! maxOrder2 > maxOrder1. If not, swap the variables!
    int i;
    coeffNo1 = maxOrder1;
    coeffNo2 = maxOrder2;
    // Create and initialize polynomial coefficients container
    int arrayLen = (2*maxOrder2 + 2*maxOrder1*maxOrder2 + maxOrder1 - maxOrder1*maxOrder1 + 2)/2;
    coeffs = (double*)malloc(sizeof(double) * arrayLen);
    for (i=0; i<arrayLen; i++) {
        coeffs[i] = coeffArray[i];
    }
}

// class destructor
Polynomial2D::~Polynomial2D ()
{
    free(coeffs);
}

// polynomial evaluation
double Polynomial2D::evaluate (double x, double y) {
    int i, j, k=0;
    double sum=0;
    for (i=0; i<=coeffNo1; i++) {
        for (j=0; j<=coeffNo2; j++) {
            if (i+j<=coeffNo2) {
                sum += coeffs[k]*pow(x,i)*pow(y,j);
                k++;
            }
        }
    }
    // std::cout << "2DPoly: " << x << " " << y << " " << sum << std::endl; // Sanity check output
    return sum;
}


/////////////////
// Define Spline3
// Cubic spline, 4 parameters per variable interval

// class constructor
Spline3::Spline3(int breaksNoIn, double * breaksIn, double * coeffsIn) : Polynomial()
{
    int i;
    breaksNo = breaksNoIn;
    // Create and initialize breaks container
    breaks = (double*)malloc(sizeof(double) * (breaksNo+1));
    for (i=0; i<=breaksNo; i++) {
        breaks[i] = breaksIn[i];
    }
    // Create and initialize polynomial coefficients container
    coeffs = (double*)malloc(sizeof(double) * (breaksNo*4));
    for (i=0; i<(breaksNo*4); i++) {
        coeffs[i] = coeffsIn[i];
    }
}

// class destructor
Spline3::~Spline3()
{
    free(breaks);
    free(coeffs);
}

// polynomial evaluation
double Spline3::evaluate(double x)
{
    int i;
    for (i=0;i<breaksNo;i++) {
    if (x<=breaks[i+1])
      break;
    }
    if (i==breaksNo) i--;
    double delta = x-breaks[i];
    double value = coeffs[4*i]*pow(delta,3) + coeffs[4*i+1]*pow(delta,2) + coeffs[4*i+2]*delta + coeffs[4*i+3];
    return value;
}
