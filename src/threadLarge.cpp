#include <threadLarge.h>
#include <vector>
#define MAX(a,b) ((a)>(b))?(a):(b)

namespace testThreads{

	threadLarge::threadLarge(ros::NodeHandle nh, double sleep_ms){
		sleep_ms_=sleep_ms;
	}


	threadLarge::~threadLarge(){}



	void threadLarge::update(){

		std::clock_t start;
		double duration=0;
		double duration2=0;

		while (ros::ok()) {
			std::vector<double> result;

			 start = std::clock();
			 auto t_start = std::chrono::high_resolution_clock::now();

			 std::this_thread::sleep_for(std::chrono::milliseconds(1));
			 for (unsigned int i = 0; i < 600000; i++)
			 {
			 	result.push_back(0.8);
			 }
			 duration =MAX(duration,1000.0* ( std::clock() - start ) / (double) CLOCKS_PER_SEC);
			 auto t_end = std::chrono::high_resolution_clock::now();
			 double temp=std::chrono::duration<double,std::milli>(t_end-t_start).count();
			 duration2 =MAX(duration2,  temp );
     std::cout <<"threadLarge -------- max CPU time used: "    << duration << " ms"<<"max Wall clock time passed: "    << duration2 << " ms\n"<<std::endl;
		 }

	 }
 }
