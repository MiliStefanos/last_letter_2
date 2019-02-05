
Master::Master()
{
    printf("Master contructor\n");
    gazebo_sub = nh.subscribe("clock", 1, &Master::gazebo_clk_clb, this);
}

void Master::gazebo_clk_clb(const rosgraph_msgs::Clock::ConstPtr& msg)
{
    model.modelStep();

}

