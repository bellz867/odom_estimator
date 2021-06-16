#include <odom_estimator.h>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_node");

	//handle to launch file parameters
	ros::NodeHandle nhp("~");

	// mocap names to track
	std::vector<std::string> names;
	nhp.param<std::vector<std::string>>("names",names,{"none"});

  if (names.at(0) == "none")
  {
    ROS_ERROR("NO OBJECTS TOLD TO TRACK");
    return 0;
  }

  std::vector<OdomEstimator*> odomEstimators;
  OdomEstimator* odomEstimator;
  for (int ii = 0; ii < names.size(); ii++)
  {
    odomEstimator = new OdomEstimator(names.at(ii));
    odomEstimators.push_back(odomEstimator);
  }

  ros::AsyncSpinner spinner(8);
  spinner.start();
  ros::waitForShutdown();

  for (int ii = 0; ii < odomEstimators.size(); ii++)
  {
    delete odomEstimators.at(ii);
  }

  return 0;
}
