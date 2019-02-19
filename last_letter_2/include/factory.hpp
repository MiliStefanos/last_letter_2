class Factory
{
    public:
    Aerodynamics * buildAerodynamics(Model *);
    Propulsion * buildPropulsion(Model *);
    Polynomial * buildPolynomial(char * );
};