#include <iostream>
#include <thread>
#include <chrono>

#include <radxa/pwm_driver.h>

int main(){
  radxa::PwmDriver pwm(10, 2, 50);
  pwm.open();
  pwm.setPulseWidthUs(1000);
  pwm.enable();
  std::cout << "frequency is " << pwm.frequencyHz() << std::endl; 
  std::cout << "period is " << pwm.periodUs() << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(10));
  return 0;
}
