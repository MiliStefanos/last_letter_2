class Factory
{
    public:
    Aerodynamics * buildAerodynamics(Model *);
    Propulsion * buildPropulsion(Model *);
};