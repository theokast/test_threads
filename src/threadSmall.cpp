#include <threadSmall.h>
#include <thread>
#define MAX(a,b) ((a)>(b))?(a):(b)

namespace testThreads{

	threadSmall::threadSmall(ros::NodeHandle nh,const std::shared_ptr<arl::robot::Robot>& robot,const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
	:arl::robot::Controller(robot, "sigma_kuka_dora")
,cloud_(pc){
 simulation_enabled_=false;
	}


	threadSmall::~threadSmall(){}

	void threadSmall::init()
	{	std::cout <<"[SigmaKukaDevice::init] start"<< std::endl;
	 duration=0;
	duration2=0;
	  // Init timer

	  t = 0.0;
	  qs_ = arma::zeros<arma::vec> (7);
		dqs_ = arma::zeros<arma::vec> (7);
	  if (!simulation_enabled_){
	  robot->setMode(arl::robot::Mode::TORQUE_CONTROL);
	  }
	  else{
	  robot->setMode(arl::robot::Mode::POSITION_CONTROL);
	  }

	  ros::Duration(5.0).sleep();
	  // calibrate the devices

	  std::cout <<"[threadSmall::init] end"<< std::endl;
	}


	void threadSmall::update(){
	time += 0.002;
		std::clock_t start;

    start = std::clock();
		auto t_start = std::chrono::high_resolution_clock::now();
		//Jac_=robot->getJacobian().toArma();
		//vs_=robot->getTwist().toArma();
		qs_=robot->getJointPosition().toArma();
	  dqs_=robot->getJointVelocity().toArma();

  std::this_thread::sleep_for(std::chrono::milliseconds(1));
	//	Fs_=Ds_*(vs_);
	//u_= Jac_.t()*(-Fs_);

     //arma::vec u_=-0.2*dqs_;
		 arma::vec u_ = arma::zeros<arma::vec> (7);

		if (!simulation_enabled_){
			robot->setJointTorque(u_);

		   }
		  else{
		  robot->setJointPosition(qs_);
		   }
			 duration =MAX(duration,1000.0* ( std::clock() - start ) / (double) CLOCKS_PER_SEC);
			 auto t_end = std::chrono::high_resolution_clock::now();
			 double temp=std::chrono::duration<double,std::milli>(t_end-t_start).count();
			 duration2 =MAX(duration2,  temp );
      if(temp>1.9){
				std::cout <<"threadsmall -------- max CPU time used: "    << duration << " ms "<<"max Wall clock time passed: "    << duration2 << " ms\n"<<std::endl;
}

	 }
 }
