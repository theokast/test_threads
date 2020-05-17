#include <threadSmall.h>
#include <thread>
#define MAX(a,b) ((a)>(b))?(a):(b)

namespace testThreads{

	threadSmall::threadSmall(ros::NodeHandle nh, double sleep_ms){
		sleep_ms_=sleep_ms_;
	}


	threadSmall::~threadSmall(){}



	void threadSmall::update(){

		std::clock_t start;
		double duration=0;
		double duration2=0;

		while (ros::ok()) {
			 start = std::clock();
			 auto t_start = std::chrono::high_resolution_clock::now();

			 std::this_thread::sleep_for(std::chrono::milliseconds(1));

			 duration =MAX(duration,1000.0* ( std::clock() - start ) / (double) CLOCKS_PER_SEC);
			 auto t_end = std::chrono::high_resolution_clock::now();
			 double temp=std::chrono::duration<double,std::milli>(t_end-t_start).count();
	 			 duration2 =MAX(duration2,  temp );


      std::cout <<"threadsmall -------- max CPU time used: "    << duration << " ms "<<"max Wall clock time passed: "    << duration2 << " ms\n"<<std::endl;
		 }

	 }
 }
