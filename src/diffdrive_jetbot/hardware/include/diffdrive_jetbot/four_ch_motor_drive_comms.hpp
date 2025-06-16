#ifndef DIFFDRIVE_JETBOT_COMMS_HPP
#define DIFFDRIVE_JETBOT_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <chrono>
#include <thread>

class FourChMotorDriveComms
{
public:
  FourChMotorDriveComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    
    // Clear any existing data and wait for connection to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    serial_conn_.FlushIOBuffers();
  }

  void disconnect()
  {
    if (serial_conn_.IsOpen())
    {
      serial_conn_.Close();
    }
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  void read_encoder_data(double &val_1, double &val_2, int data_type)
  {
    if (!connected()) 
    {
      val_1 = val_2 = 0.0;
      return;
    }

    // Clear buffers before sending command
    serial_conn_.FlushIOBuffers();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::string send_text = get_encoder_command(data_type);
    std::string expected_prefix = get_encoder_prefix(data_type);
    
    send_command_and_wait(send_text);
    std::string response = read_specific_response(expected_prefix);
    
    // Stop data stream
    send_command_and_wait("$upload:0,0,0#");
    
    parse_encoder_response(response, expected_prefix, val_1, val_2);
  }

  void set_motor_speed_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "$spd:" << val_1 << "," << -val_2 << ",0,0#\r";
    send_command_and_wait(ss.str());
  }

  void set_motor_pwm_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "$pwm:" << val_1 << "," << -val_2 << ",0,0#\r";
    send_command_and_wait(ss.str());
  }

  void set_pwm_pid_values(float k_p, float k_d, float k_i)
  {
    std::stringstream ss;
    ss << "$MPID:" << k_p << "," << k_i << "," << k_d << "#\r";
    send_command_and_wait(ss.str());
  }

  void set_motor_type(int motor_type)
  {
    std::stringstream ss;
    ss << "$mtype:" << motor_type << "#\r";
    send_command_and_wait(ss.str());
    std::string response = read_acknowledgment();
    std::cout << response << std::endl;
  }

  void set_motor_mline(int mline)
  {
    std::stringstream ss;
    ss << "$mline:" << mline << "#\r";
    send_command_and_wait(ss.str());
    std::string response = read_acknowledgment();
    std::cout << response << std::endl;
  }

  void set_motor_mphase(int mphase)
  {
    std::stringstream ss;
    ss << "$mphase:" << mphase << "#\r";
    send_command_and_wait(ss.str());
    std::string response = read_acknowledgment();
    std::cout << response << std::endl;
  }

  void set_wheel_diam(int wdiameter)
  {
    std::stringstream ss;
    ss << "$wdiameter:" << wdiameter << "#\r";
    send_command_and_wait(ss.str());
    std::string response = read_acknowledgment();
    std::cout << response << std::endl;
  }

  void set_motor_deadzone(int deadzone)
  {
    std::stringstream ss;
    ss << "$deadzone:" << deadzone << "#\r";
    send_command_and_wait(ss.str());
    std::string response = read_acknowledgment();
    std::cout << response << std::endl;
  }

  void set_encoder_reset()
  {
    std::stringstream ss;
    ss << "$flash_reset" << "#\r";
    send_command_and_wait(ss.str());
    std::string response = read_acknowledgment();
    std::cout << response << std::endl;
  }

  void setup_motor_parameters(int mtype, int deadzone, int mline, int mphase, int wdiameter, float pid_p, float pid_i, float pid_d)
  {
    // set_encoder_reset();
    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // std::cout << "Reset cmu" << std::endl;
    set_motor_type(mtype);
    set_motor_deadzone(deadzone);
    set_motor_mline(mline);
    set_motor_mphase(mphase);
    set_wheel_diam(wdiameter);
    set_pwm_pid_values(pid_p, pid_d, pid_i);
  }

  void read_flash_settings()
  {
    serial_conn_.FlushIOBuffers();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    send_command_and_wait("$read_flash#");
    std::string response = read_all_data_from_buffer();
    std::cout << "Flash settings: " << std::endl << response << std::endl;
  }

  // Legacy functions for backward compatibility
  void send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    send_command_and_wait(msg_to_send, print_output);
  }

  std::string read_all_data_from_buffer()
  {
    std::string all_data;
    auto start_time = std::chrono::steady_clock::now();

    while (true)
    {
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time).count();
          
      if (elapsed >= timeout_ms_)
      {
        break;
      }
      
      if (serial_conn_.IsDataAvailable())
      {
        char byte;
        serial_conn_.ReadByte(byte, 1); // Very short timeout for individual byte
        all_data += byte;
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    return all_data;
  }

  std::string read_last_line()
  {
    std::string all_data = read_all_data_from_buffer();
    if (all_data.empty())
    {
      return "";
    }

    size_t last_dollar = all_data.rfind('$');
    if (last_dollar == std::string::npos)
    {
      return all_data;
    }
    
    size_t last_hash = all_data.find('#', last_dollar);
    if (last_hash == std::string::npos)
    {
      return all_data;
    }
    
    return all_data.substr(last_dollar, last_hash - last_dollar + 1);
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_ = 50;

  LibSerial::BaudRate convert_baud_rate(int baud_rate) const
  {
    switch (baud_rate)
    {
      case 115200: return LibSerial::BaudRate::BAUD_115200;
      case 57600: return LibSerial::BaudRate::BAUD_57600;
      case 38400: return LibSerial::BaudRate::BAUD_38400;
      case 19200: return LibSerial::BaudRate::BAUD_19200;
      case 9600: return LibSerial::BaudRate::BAUD_9600;
      default:
        std::cout << "Warning: Baud rate " << baud_rate << " not supported! Using 115200" << std::endl;
        return LibSerial::BaudRate::BAUD_115200;
    }
  }

  void send_command_and_wait(const std::string &command, bool print_debug = false)
  {
    if (!connected())
    {
      std::cerr << "Error: Serial connection not established" << std::endl;
      return;
    }
    
    // Ensure clean state before sending
    serial_conn_.FlushIOBuffers();
    
    // Send command
    serial_conn_.Write(command);
    
    // Small delay to ensure command is sent
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    if (print_debug)
    {
      std::cout << "Sent: " << command << std::endl;
    }
  }

  std::string read_specific_response(const std::string &expected_prefix, int max_attempts = 5)
  {
    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
      std::string data = read_all_data_from_buffer();
      
      if (data.find(expected_prefix) != std::string::npos)
      {
        // Found expected response, extract the complete message
        size_t start_pos = data.find('$');
        size_t end_pos = data.find('#', start_pos);
        
        if (start_pos != std::string::npos && end_pos != std::string::npos)
        {
          return data.substr(start_pos, end_pos - start_pos + 1);
        }
      }
      
      // If we didn't get the expected response, wait a bit and try again
      if (attempt < max_attempts - 1)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    
    return ""; // Failed to get expected response
  }

  std::string read_acknowledgment()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    return read_all_data_from_buffer();
  }

  std::string get_encoder_command(int data_type) const
  {
    switch (data_type)
    {
      case 1: return "$upload:1,0,0#";
      case 2: return "$upload:0,1,0#";
      case 3: return "$upload:0,0,1#";
      default:
        std::cerr << "Error: Invalid data_type " << data_type << std::endl;
        return "$upload:0,0,0#";
    }
  }

  std::string get_encoder_prefix(int data_type) const
  {
    switch (data_type)
    {
      case 1: return "$MAll:";
      case 2: return "$MTEP:";
      case 3: return "$MSPD:";
      default: return "$MAll:";
    }
  }

  void parse_encoder_response(const std::string &response, const std::string &prefix, 
                             double &left_val, double &right_val)
  {
    left_val = right_val = 0.0;
    
    size_t prefix_pos = response.find(prefix);
    if (prefix_pos == std::string::npos)
    {
      return;
    }

    std::string data = response.substr(prefix_pos + prefix.length());
    size_t hash_pos = data.find('#');
    if (hash_pos != std::string::npos)
    {
      data = data.substr(0, hash_pos);
    }

    try
    {
      std::stringstream ss(data);
      std::string val_str;
      
      if (std::getline(ss, val_str, ','))
      {
        left_val = std::stod(val_str);
      }
      
      if (std::getline(ss, val_str, ','))
      {
        right_val = -std::stod(val_str); // Invert right wheel
      }
    }
    catch (const std::exception &e)
    {
      std::cerr << "Error parsing encoder data: " << e.what() << std::endl;
      left_val = right_val = 0.0;
    }
  }
};

#endif // DIFFDRIVE_JETBOT_COMMS_HPP