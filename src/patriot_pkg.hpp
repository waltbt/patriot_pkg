class PatriotNode {
    private:
    TrackerControl* tracker;
    _Quaternion Quaternion;

    // int num_stations;
    // int soft_station_num;
    // int cur_hemisphere[3];
    // float second_sensor_pose[7];
    // float cur_soft_arm_pose[7];
    //#int master_axis;


    ros::Publisher arm_pose_pub;

    public:
    PatriotNode(ros::NodeHandle &nh);
    ~PatriotNode(void);


     // PatriotNode(ros::NodeHandle &nh) {
    // tracker = new TrackerControl;
		// //~ nh.getParam("/soft_arm_tip_station", soft_station_num);
		// soft_station_num = 0;
		// Patriot PatriotDevice;
		// //~ set_hemisphere(PatriotDevice, 0, 0, -1); //Upper hemisphere
		// set_hemisphere(PatriotDevice, 0, 1, 0);
		// num_stations = PatriotDevice.nstations;
    //     arm_pose_pub = nh.advertise<patriot_pkg::arm_pose>("patriot/arm_pose", 10);
    //
    //     get_pose_loop(PatriotDevice);
    // }





  //   void get_pose_loop(Patriot &PatriotDevice){
	// 	ros::Rate rate(60);
	// 	while(ros::ok()) {
	// 		patriot_pkg::arm_pose msg;
	// 		for(int i=0;i < num_stations; i++){
	// 		std::vector<float> pose = PatriotDevice.get_pose(i);
	// 		// std::cout << pose[0] << std::endl;
  //           if(i == soft_station_num){
  //
  //               for (int n=0;n<7;n++){
  //                   cur_soft_arm_pose[n] = pose[n];
  //               }
  //               //std::cout << "X: " << pose[0] << " Y: " << pose[1] << " Z: " << pose[2] << std::endl;
  //               msg.soft_arm_pose = pose;
  //           }else{
	// 				//std::cout << "No sensor action given" << std::endl;
  //               for (int n=0;n<3;n++){
  //                   //#//msg.soft_arm_pose_w[n] = pose[n];
  //                   //cur_soft_arm_pose[n] = pose[n];
  //                   second_sensor_pose[n] = pose[n];
  //               }
  //               msg.second_sensor_pose = pose;
  //           }
  //           //#std::cout << "F" << std::endl;
	// 	}
	// 	//#msg.rigid_arm_pose_w = cur_rigid_arm_pose;
	// 	//#msg.soft_arm_pose_w = cur_soft_arm_pose;
  //       arm_pose_pub.publish(msg);
  //       //#check_hemisphere(PatriotDevice);
	// 	rate.sleep();
	// 	}
  //
  //
	// }

};
