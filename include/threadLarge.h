#include <autharl_core>
#include <thread>
#include <chrono>
#include<ctime>

namespace testThreads{
  class threadLarge{
  public:
    threadLarge(ros::NodeHandle nh, double sleep_ms);
    ~threadLarge();
    void update();

  private:
    double sleep_ms_;
 };
}
