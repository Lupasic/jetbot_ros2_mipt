#include <iostream>
#include <unistd.h>
#include "four_ch_motor_drive_comms.hpp"

int main()
{
  try
  {
    // Создаем объект
    FourChMotorDriveComms motor_comm;

    // Инициализируем соединение — предположим, есть метод `setup()` или `connect()`
    // Замените, если метод называется иначе
    motor_comm.connect("/dev/ttyUSB0", 115200,50);  // или motor_comm.setup(...)

    // Читаем значения энкодеров
    double left_ticks = 0.0;
    double right_ticks = 0.0;

    motor_comm.read_encoder_data(left_ticks, right_ticks,1);

    std::cout << "Left encoder 1: " << static_cast<int>(left_ticks) << ", Right encoder: " << static_cast<int>(right_ticks) << std::endl;

    std::cout <<"send speed 100,100" << std::endl;
    motor_comm.set_motor_speed_values(100,100);
    sleep(5);
    motor_comm.read_encoder_data(left_ticks, right_ticks,1);
    std::cout << "Left encoder 1: " << static_cast<int>(left_ticks) << ", Right encoder: " << static_cast<int>(right_ticks) << std::endl;
    motor_comm.read_encoder_data(left_ticks, right_ticks,2);
    std::cout << "Left encoder/s: " << static_cast<int>(left_ticks) << ", Right encoder/s: " << static_cast<int>(right_ticks) << std::endl;
    motor_comm.read_encoder_data(left_ticks, right_ticks,3);
    std::cout << "Left spd: " << left_ticks << ", Right spd: " << right_ticks << std::endl;
    motor_comm.set_motor_speed_values(0,0);
    motor_comm.set_motor_pwm_values(0,0);

    motor_comm.disconnect();

  }
  catch (const std::exception &e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
