#include <robotcar_bringup/bringup.h>
#include <thread>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotcar_bringup");
  ros::NodeHandle nh;

  BringUp robotcar;

  robotcar.init();

  thread read_task(bind(readSpin, &robotcar));
  read_task.detach();

  thread write_task(bind(writeSpin, &robotcar));
  write_task.join();

  return 0;
}