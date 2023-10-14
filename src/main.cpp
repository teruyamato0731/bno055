#include <mbed.h>

// Const variable

// Function prototype

// IO
// CAN can{PA_11, PA_12, (int)1e6};
// CAN can{PB_12, PB_13, (int)1e6};

// Struct definition
struct Serial {
  void flush_read_buffer() {
    uint8_t buf;
    while(bus_.readable() && bus_.read(&buf, 1) > 0) {}
  }
  template<int N>
  void uart_transmit(const uint8_t (&send)[N]) {
    bus_.write(send, N);
  }
  template<int N>
  bool uart_receive(uint8_t (&buf)[N], const std::chrono::microseconds timeout) {
    const auto pre = HighResClock::now();
    uint8_t* p = std::begin(buf);
    while(HighResClock::now() - pre < timeout) {
      if(bus_.readable() && bus_.read(p, 1) > 0 && ++p == std::end(buf)) return true;
    }
    return false;
  }
  template<class T>
  bool uart_receive(T& buf, const std::chrono::microseconds timeout) {
    return uart_receive(reinterpret_cast<uint8_t(&)[sizeof(buf)]>(buf), timeout);
  }
  // UnbufferedSerial bus_;
  BufferedSerial bus_;
};
struct Bno055 {
  static constexpr auto timeout = 5ms;
  enum class Response : uint8_t {
    NO_RESPONSE = 0x00,  // 独自定義
    WRITE_SUCCESS = 0x01,
    READ_FAIL,
    WRITE_FAIL,
    REGMAP_INVALID_ADDRESS,
    REGMAP_WRITE_DISABLED,
    WRONG_START_BYTE,
    BUS_OVER_RUN_ERROR,
    MAX_LENGTH_ERROR,
    MIN_LENGTH_ERROR,
    RECEIVE_CHARACTER_TIMEOUT,
  };
  enum Register {
    UNIT_SEL = 0x3B,
    OPR_MODE = 0x3D,
    AXIS_MAP_CONFIG = 0x41,
    AXIS_MAP_SIGN = 0x42,
  };
  struct Euler {
    int16_t x;
    int16_t y;
    int16_t z;
  };

  Bno055(PinName tx, PinName rx) : bus_{{tx, rx, 115200}} {}
  void init() {
    while(write(OPR_MODE, {0x00}) != Response::WRITE_SUCCESS) {
      printf("bno055:Operating Mode setting CONFIGMODE\n");
    }
    ThisThread::sleep_for(19ms);
    while(write(UNIT_SEL, {0x04}) != Response::WRITE_SUCCESS) {
      printf("bno055:UNIT_SEL setting\n");
    }
    while(write(OPR_MODE, {0x08}) != Response::WRITE_SUCCESS) {
      printf("bno055:Operating Mode setting IMU\n");
    }
  }
  Response request_euler_angle() {
    return read(0x1A, euler_angle);
  }

  Euler get_euler() {
    return euler_angle;
  }
  float get_x_rad() {
    return to_rad(euler_angle.x);
  }
  float get_y_rad() {
    return to_rad(euler_angle.y);
  }
  float get_z_rad() {
    return to_rad(euler_angle.z);
  }

 private:
  template<int N>
  Response write(const uint8_t addr, const uint8_t (&data)[N]) {
    static_assert(N <= 128);
    bus_.flush_read_buffer();
    bus_.uart_transmit({0xAA, 0x00, addr, N});
    bus_.uart_transmit(data);

    if(uint8_t buf[2] = {}; bus_.uart_receive(buf, timeout) && buf[0] == 0xEE) {
      return Response{buf[1]};
    }
    return Response{};
  }
  template<int N>
  Response read(const uint8_t addr, uint8_t (&buf)[N]) {
    static_assert(N <= 128);
    bus_.flush_read_buffer();
    bus_.uart_transmit({0xAA, 0x01, addr, N});
    if(uint8_t st_byte; bus_.uart_receive(st_byte, timeout)) {
      switch(st_byte) {
        case 0xBB:
          if(bus_.uart_receive(st_byte, timeout) && st_byte == N && bus_.uart_receive(buf, timeout))
            return Response::WRITE_SUCCESS;
          break;
        case 0xEE: {
          if(bus_.uart_receive(st_byte, timeout)) return Response{st_byte};
          break;
        }
      }
    }
    return Response{};
  }
  template<class T>
  Response read(const uint8_t addr, T& buf) {
    return read(addr, reinterpret_cast<uint8_t(&)[sizeof(buf)]>(buf));
  }
  static float to_rad(const int16_t val) {
    constexpr auto radian_representation = 900;
    constexpr float k = 1.0f / radian_representation;
    return val * k;
  }
  Serial bus_;
  Euler euler_angle;
};

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
