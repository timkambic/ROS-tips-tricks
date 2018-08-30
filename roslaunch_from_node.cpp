void launchROSLAUNCH(std::string &ros_distribution,std::vector<std::string> command){ // convert command arguments from vector<string> to char* [] and call execv
		std::string path_to_roslaunch = "/opt/ros/" + ros_distribution + "/bin/roslaunch"; // path where roslaunch executable is located
		auto **argv = new const char* [command.size()+2];   // +2 is extra room for program name and sentinel
		argv [0] = "roslaunch";         // by convention, argv[0] is program name
		for (int j = 0;  j < command.size()+1;  ++j)     // copy args
			argv [j+1] = command[j] .c_str();
		argv [command.size()+1] = NULL;  // end of arguments sentinel is NULL
		execv (path_to_roslaunch.c_str(), (char **)argv);
	}


int main(){
	ros::NodeHandle n;
	std::string ros_distro;
	n.getParam("/rosdistro", ros_distro); // get ros distribution to know where the roslaunch executable is located
	ros_distro.erase(std::remove(ros_distro.begin(), ros_distro.end(), '\n'), ros_distro.end()); 
	pid_t PID;
	PID = fork();
	if (PID == 0) { 
		std::vector<std::string> command ={"package_name", "pacgake_launch_file.launch", "argument1:= true" , "argument2:=100"};
		launchROSLAUNCH(ros_distro,command); 
		exit(1);
	}

	ros::service::waitForService("launched_node_name/get_loggers");

	// DO SOME OTHER STUFF

	kill(PID, SIGTERM); 
	int status;
	waitpid(PID_orient,&status,0);
}
