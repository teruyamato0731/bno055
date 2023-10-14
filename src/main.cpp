#include <mbed.h>

#include "Bno055.hpp"

// Const variable

// Function prototype

// IO
// CAN can{PA_11, PA_12, (int)1e6};
// CAN can{PB_12, PB_13, (int)1e6};

// Struct definition

// Global variable
Bno055 bno055{PA_0, PA_1};

/// @brief The application entry point.
int main() {
  // put your setup code here, to run once:
  printf("\nsetup\n");
  bno055.init();
  while(1) {
    // put your main code here, to run repeatedly:
    auto now = HighResClock::now();
    static auto pre = HighResClock::now();
    if(now - pre > 5ms) {
      printf("%d ", (int)bno055.request_euler_angle());
      printf("% 5.2f ", bno055.get_x_rad());
      printf("% 5.2f ", bno055.get_y_rad());
      printf("% 5.2f ", bno055.get_z_rad());
      printf("\n");
      pre = now;
    }
  }
}

// Function definition
