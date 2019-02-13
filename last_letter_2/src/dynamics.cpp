Dynamics::Dynamics(Model *parent)
{
    printf("dynamic constuctor \n");
    model=parent;
    aerodynamics=new StdLinearAero(model);
    // aerodynamics=factory.buildNoAerodynamics(model);
    propulsion = new BeardEngine(model);
    // propulsion=factory.buildPropulsion();
}

void Dynamics::calcAero()
{
    aerodynamics->calcWrench();
}

void Dynamics::calcProp()
{
    propulsion->calcWrench();
}