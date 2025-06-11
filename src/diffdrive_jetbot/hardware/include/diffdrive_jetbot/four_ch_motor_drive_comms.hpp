#ifndef DIFFDRIVE_JETBOT_COMMS_HPP
#define DIFFDRIVE_JETBOT_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include <chrono>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 115200" << std::endl;
      return LibSerial::BaudRate::BAUD_115200;
  }
}

class FourChMotorDriveComms
{

public:

  FourChMotorDriveComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  void send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << std::endl;
    }
  }

std::string read_last_line() {
    std::string all_data;
    std::string last_line;
    auto start_time = std::chrono::steady_clock::now();

    // read_all_data_from_buffer
    while (true) {
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
          
      if (elapsed >= timeout_ms_) {
              break;
          }
      if (serial_conn_.IsDataAvailable()) {
          char byte;
          serial_conn_.ReadByte(byte, timeout_ms_);
          all_data += byte;
        }
    }

    // find last `\r`
size_t last_dollar_pos = all_data.rfind('$');
size_t last_hash_pos = all_data.rfind('#', last_dollar_pos); // find # after $

if (last_dollar_pos != std::string::npos && last_hash_pos != std::string::npos) {
    last_line = all_data.substr(last_dollar_pos, last_hash_pos - last_dollar_pos + 1);
} else {
    last_line = all_data; // fallback
}
    // std::cout << "thats my boy" << last_line <<std::endl;
    return last_line;
}

 void read_encoder_data(double &val_1, double &val_2, int data_type)
  {
    //if data_type 1 - total encoder data, 2, encoder ticks per time, 3 - speed
    // Start data
    std::string response = "";
    std::string send_text = "";
    switch (data_type)
    {
      case 1: send_text = "$upload:1,0,0#"; break;
      case 2: send_text = "$upload:0,1,0#"; break;
      case 3: send_text = "$upload:0,0,1#"; break;
      default:
        std::cout << "Error! Number not 1, 2 or 3!  if data_type 1 - total encoder data, 2, encoder ticks per time, 3 - speed" << "\n By default all are zeros" << std::endl;
        send_text = "$upload:0,0,0#";
    }
    send_msg(send_text);

    response = read_last_line();

    // Stop encoder data
    send_msg("$upload:0,0,0#");

    // Data looks like $MAll:-216,354,0,-2#

    size_t prefix_pos;
    switch (data_type)
    {
      case 1: prefix_pos = response.find("$MAll:"); break;
      case 2: prefix_pos = response.find("$MTEP:"); break;
      case 3: prefix_pos = response.find("$MSPD:"); break;
      default:
        std::cout << "Error! Number not 1, 2 or 3!  if data_type 1 - total encoder data, 2, encoder ticks per time, 3 - speed" << "\n By default all are zeros" << std::endl; 
    }
    if (prefix_pos != std::string::npos) {
      std::string data = response.substr(prefix_pos + 6); // skip "$MAll:"
      size_t hash_pos = data.find('#');
      if (hash_pos != std::string::npos) {
        data = data.substr(0, hash_pos);  // cut off #

        std::stringstream ss(data);
        std::string val_str;
        std::getline(ss, val_str, ',');
        val_1 = std::stod(val_str.c_str());

        std::getline(ss, val_str, ',');
        val_2 = -std::stod(val_str.c_str());

        // Ignoring other 2 values
      }
    }
  }

  void set_motor_speed_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "$spd:" << val_1 << "," << -val_2 << ",0,0" << "#\r";
    send_msg(ss.str());
  }

  void set_motor_pwm_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "$pwm:" << val_1 << "," << -val_2 << ",0,0" << "#\r";
    send_msg(ss.str());
  }

  void set_pwm_pid_values(float k_p, float k_d, float k_i)
  {
    std::stringstream ss;
    ss << "$MPID:" << k_p << "," << k_i << "," << k_d << "#\r";
    send_msg(ss.str());
  }

  void set_motor_type(int motor_type)
  {
    std::stringstream ss;
    ss << "$mtype:" << motor_type << "#\r";
    send_msg(ss.str());
  }

  void set_motor_mline(int mline)
  {
    std::stringstream ss;
    ss << "$mline:" << mline << "#\r";
    send_msg(ss.str());
  }

  void set_motor_mphase(int mphase)
  {
    std::stringstream ss;
    ss << "$mphase:" << mphase << "#\r";
    send_msg(ss.str());
  }

  void set_wheel_diam(int wdiameter)
  {
    std::stringstream ss;
    ss << "$wdiameter:" << wdiameter << "#\r";
    send_msg(ss.str());
  }

  void set_motor_deadzone(int deadzone)
  {
    std::stringstream ss;
    ss << "$deadzone:" << deadzone << "#\r";
    send_msg(ss.str());
  }


private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_JETBOT_COMMS_HPP