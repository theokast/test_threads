
#include <autharl_core>
#include <ros/ros.h>
#include <lwr_robot/robot.h>
#include <thread>
#include <boost/shared_ptr.hpp>
#include <threadSmall.h>
#include <threadLarge.h>

#include <sched.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{auto t_start = std::chrono::high_resolution_clock::now();
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud2);

  kdtree.setInputCloud(cloud2);

  std::cout<<"calll2"<<std::endl;

 std::this_thread::sleep_for(std::chrono::milliseconds(3));
 std::vector<int> myvector;
for(int i=0; i<10000;i++){
  myvector.push_back(i);
}
auto t_end= std::chrono::high_resolution_clock::now();

double duration2=std::chrono::duration<double,std::milli>(t_end-t_start).count();
//std::cout <<"Call Wall clock time passed: "    << duration2 << " ms\n"<<std::endl;
}


int main(int argc, char** argv){
    // Initialize the ROS node

	ros::init(argc, argv, "test_threads_node");
	ros::NodeHandle  nh(ros::this_node::getName());
	ros::Subscriber sub = nh.subscribe("/master_slave_constraints_publisher_node/vessel", 1, pointCloudCallback);


	auto model = std::make_shared<arl::robot::ROSModel>();
  std::shared_ptr<arl::robot::Robot> robot;
  std::shared_ptr<arl::robot::Sensor> sensor;
	bool simulation_enabled=false;
	if (simulation_enabled){ // Create a simulated robot, use can use a real robot also
    robot = std::make_shared<arl::robot::RobotSim>(model, 2e-3);
        // Create the FT sensor
    sensor = std::make_shared<arl::robot::Sensor>("Ati");
  }
  else{

    robot = std::make_shared<arl::lwr::Robot>(model, "Kuka");

       }

	//auto thread_L =std::make_shared<testThreads::threadLarge>(nh,100);
	auto  thread_S = std::make_shared<testThreads::threadSmall>(nh,robot,cloud2);

		sched_param param;
		int priority, policy, ret;
std::cout<<"ddsd"<<std::endl;
	std::thread threadS(&arl::robot::Controller::runRT, thread_S);






	pthread_getschedparam(threadS.native_handle(), &policy, &param);
	priority = param.sched_priority;
	param.sched_priority =sched_get_priority_max(SCHED_FIFO);
	ret =pthread_setschedparam(threadS.native_handle(), SCHED_FIFO, &param);
	ret =pthread_getschedparam(threadS.native_handle(), &policy, &param);
  priority = param.sched_priority;
	if (ret != 0) {
	         std::cout << "Couldn't retrieve real-time scheduling paramers" << std::endl;

	     }

	     // Check the correct policy was applied
	     if(policy != SCHED_FIFO) {
	         std::cout << "Scheduling is NOT SCHED_FIFO!" << std::endl;
	     } else {
	         std::cout << "SCHED_FIFO OK" << std::endl;
	     }
	std::cout<<"get threadS PRIORITY="<<priority<<std::endl;

//	std::thread threadL(&testThreads::threadLarge::update, thread_L);
//	pthread_getschedparam(threadL.native_handle(), &policy, &param);
	//param.sched_priority =60;






	//pthread_setschedparam(threadL.native_handle(), policy, &param);
	//pthread_getschedparam(threadL.native_handle(), &policy, &param);
//	priority = param.sched_priority;
//	std::cout<<"get threadL PRIORITY="<<priority<<std::endl;

//	std::thread::id t1_id = threadS.get_id();
	//	std::thread::id t2_id = threadL.get_id();
//	std::cout << "threadS ID associted with thread1= "<< t1_id << std::endl;
	//   std::cout << "threadL ID associted with thread2= "<< t2_id << std::endl;


ros::MultiThreadedSpinner spinner(6);
spinner.spin();
	threadS.join();
//	threadL.join();

	ros::waitForShutdown();
	return 0;
}
