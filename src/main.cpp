#include <mbed.h>

// Const variable

// Function prototype

// IO
// CAN can{PA_11, PA_12, (int)1e6};
// CAN can{PB_12, PB_13, (int)1e6};
// CANMessage msg;
Timer timer;

// Struct definition

// Global variable

/// @brief The application entry point.
int main() {
  // put your setup code here, to run once:
  printf("\nsetup\n");
  timer.start();
  auto pre = timer.elapsed_time();
  while(1) {
    // put your main code here, to run repeatedly:
    auto now = timer.elapsed_time();
    if(now - pre > 20ms) {
      printf("hoge\n");
      pre = now;
    }
  }
}

// Function definition
