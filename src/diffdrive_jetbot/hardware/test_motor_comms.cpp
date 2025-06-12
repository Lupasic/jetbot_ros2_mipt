#include <iostream>
#include <unistd.h>
#include <cmath>
#include "four_ch_motor_drive_comms.hpp"

int main()
{
  try
  {
    // Создаем объект
    FourChMotorDriveComms motor_comm;

    // Инициализируем соединение — предположим, есть метод `setup()` или `connect()`
    // Замените, если метод называется иначе
    motor_comm.connect("/dev/ttyUSB0", 115200,200);  // или motor_comm.setup(...)

    // Читаем значения энкодеров
    double init_l_ticks =0;
    double init_r_ticks =0;
    double left_ticks = 0.0;
    double right_ticks = 0.0;

    motor_comm.set_motor_mphase(40);
    motor_comm.set_motor_mline(11);
    motor_comm.read_flash_settings();
    motor_comm.read_encoder_data(left_ticks, right_ticks,1);
    int counts_per_rev = 1760;

    std::cout << "Left encoder 1: " << static_cast<int>(left_ticks) << ", Right encoder: " << static_cast<int>(right_ticks) << std::endl;

    init_l_ticks = left_ticks;
    init_r_ticks = right_ticks;

    std::cout <<"send speed 100,100" << std::endl;
    motor_comm.set_motor_speed_values(100,100);
    sleep(5);
    motor_comm.read_encoder_data(left_ticks, right_ticks,1);
    // std::cout << "Left angle 1: " << (2*M_PI)/counts_per_rev*(left_ticks-init_l_ticks) << ", Right angle: " << (2*M_PI)/counts_per_rev*(right_ticks-init_r_ticks) << std::endl;
    std::cout << "Left angle 1: " << left_ticks-init_l_ticks << ", Right angle: " << (right_ticks-init_r_ticks) << std::endl;
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
