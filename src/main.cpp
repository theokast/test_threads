
#include <autharl_core>
#include <ros/ros.h>
#include <thread>
#include <boost/shared_ptr.hpp>
#include <threadSmall.h>
#include <threadLarge.h>

#include <sched.h>




int main(int argc, char** argv){
    // Initialize the ROS node

	ros::init(argc, argv, "test_threads_node");

	std::cout<<"ddsd"<<std::endl;

	ros::NodeHandle  nh(ros::this_node::getName());
	std::cout<<"ddsd"<<std::endl;

	auto thread_L =std::make_shared<testThreads::threadLarge>(nh,100);
	auto  thread_S = std::make_shared<testThreads::threadSmall>(nh,20);

		sched_param param;
		int priority, policy, ret;
std::cout<<"ddsd"<<std::endl;
	std::thread threadS(&testThreads::threadSmall::update, thread_S);






	pthread_getschedparam(threadS.native_handle(), &policy, &param);
	priority = param.sched_priority;
	param.sched_priority =sched_get_priority_max(SCHED_FIFO);
	ret=pthread_setschedparam(threadS.native_handle(), SCHED_FIFO, &param);
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



	std::thread threadL(&testThreads::threadLarge::update, thread_L);
	pthread_getschedparam(threadL.native_handle(), &policy, &param);
	param.sched_priority =60;






	pthread_setschedparam(threadL.native_handle(), SCHED_FIFO, &param);
	pthread_getschedparam(threadL.native_handle(), &policy, &param);
	priority = param.sched_priority;
	std::cout<<"get threadL PRIORITY="<<priority<<std::endl;

	std::thread::id t1_id = threadS.get_id();
		std::thread::id t2_id = threadL.get_id();
	std::cout << "threadS ID associted with thread1= "<< t1_id << std::endl;
	   std::cout << "threadL ID associted with thread2= "<< t2_id << std::endl;


ros::MultiThreadedSpinner spinner(6);
spinner.spin();
	threadS.join();
//	threadL.join();

	ros::waitForShutdown();
	return 0;
}
