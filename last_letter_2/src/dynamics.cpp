Dynamics::Dynamics(Model *parent)
{
    printf("dynamic constuctor \n");
    model=parent;
    Factory factory;
    aerodynamics=factory.buildAerodynamics(model);
    propulsion = factory.buildPropulsion(model);
}

void Dynamics::calcAero()
{
    aerodynamics->calcWrench();
}

void Dynamics::calcProp()
{
    propulsion->calcWrench();
}