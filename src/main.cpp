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

static bool gyro_initialized = false;
static int err_count = 0;
bool try_init_gyro() {
  return gyro_initialized || (gyro_initialized = bno055.try_init(10ms));
}
bool update_gyro() {
  if(!try_init_gyro()) return false;
  if(bno055.request_euler_x() == Bno055::Response::WRITE_SUCCESS) {
    return true;
  } else {
    if(++err_count > 10) {
      err_count = 0;
      gyro_initialized = false;
    }
    return false;
  }
}

/// @brief The application entry point.
int main() {
  // put your setup code here, to run once:
  printf("\nsetup\n");
  try_init_gyro();
  while(1) {
    // put your main code here, to run repeatedly:
    auto now = HighResClock::now();
    static auto pre = HighResClock::now();
    if(now - pre > 5ms) {
      printf("%d ", (int)update_gyro());
      printf("% 5.2f ", bno055.get_x_rad());
      printf("% 5.2f ", bno055.get_y_rad());
      printf("% 5.2f ", bno055.get_z_rad());
      printf("\n");
      pre = now;
    }
  }
}

// Function definition
