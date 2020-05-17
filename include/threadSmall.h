#include <autharl_core>
#include <thread>
#include <chrono>
#include<ctime>


namespace testThreads{
  class threadSmall{
  public:
    threadSmall(ros::NodeHandle nh, double sleep_ms);
    ~threadSmall();
    void update();

  private:
  
    double sleep_ms_;
 };
}
