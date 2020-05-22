#include <autharl_core>
#include <thread>
#include <chrono>
#include<ctime>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <mutex>

namespace testThreads{
  class threadSmall : public arl::robot::Controller{
  public:
    threadSmall(ros::NodeHandle nh,const std::shared_ptr<arl::robot::Robot>& robot,const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc);
    ~threadSmall();
    void update();

  private:
    double duration, duration2;
    double time;
    void init();
  arma::vec qs_,dqs_;
  bool simulation_enabled_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    double sleep_ms_;

 };
}
