
Master::Master()
{
    printf("Master contructor\n");
    gazebo_sub = nh.subscribe("clock", 1, &Master::gazeboClockClb, this);
}

void Master::gazeboClockClb(const rosgraph_msgs::Clock::ConstPtr& msg)
{
    model.modelStep();

}

