#ifndef DIFFDRIVE_JETBOT_COMMS_HPP
#define DIFFDRIVE_JETBOT_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>

class FourChMotorDriveComms
{
public:
  FourChMotorDriveComms() : serial_conn_(std::make_unique<LibSerial::SerialPort>()), keep_reading_(false) {}

  ~FourChMotorDriveComms()
  {
    if (connected())
    {
      disconnect();
    }
  }

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    if (connected()) return;

    serial_device_ = serial_device;
    baud_rate_ = baud_rate;
    timeout_ms_ = timeout_ms;
    serial_conn_->Open(serial_device);
    serial_conn_->SetBaudRate(convert_baud_rate(baud_rate));
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    serial_conn_->FlushIOBuffers();

    // Start continuous data streaming
    send_command("$upload:1,0,1#"); // Request both position and speed

    // Start the reader thread
    keep_reading_ = true;
    reader_thread_ = std::thread(&FourChMotorDriveComms::reader_thread_func, this);
  }

  void disconnect()
  {
    if (!connected()) return;

    // Stop continuous data stream
    try {
      send_command("$upload:0,0,0#");
    } catch (...) {
      // Ignore exceptions on disconnect
    }
    
    keep_reading_ = false;
    if (reader_thread_.joinable())
    {
      reader_thread_.join();
    }

    if (serial_conn_->IsOpen())
    {
      serial_conn_->Close();
    }
  }

  bool connected() const
  {
    return serial_conn_->IsOpen() && keep_reading_;
  }

  void read_encoder_data(double &val_1, double &val_2, int data_type)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (data_type == 1) // MAll: Encoder Ticks
    {
      val_1 = encoder_pos_[0];
      val_2 = encoder_pos_[1];
    }
    else if (data_type == 3) // MSPD: Wheel Speeds
    {
      val_1 = wheel_speeds_[0];
      val_2 = wheel_speeds_[1];
    }
    else
    {
      val_1 = val_2 = 0.0;
    }
  }

  void set_motor_speed_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "$spd:" << val_1 << "," << -val_2 << ",0,0#\r";
    send_command(ss.str());
  }

  void set_motor_pwm_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "$pwm:" << val_1 << "," << -val_2 << ",0,0#\r";
    send_command(ss.str());
  }

  void set_pwm_pid_values(float k_p, float k_d, float k_i)
  {
    std::stringstream ss;
    ss << "$MPID:" << k_p << "," << k_i << "," << k_d << "#\r";
    send_command_and_wait_for_ack(ss.str());
    handle_reboot_and_reconnect("PID values");
  }

  void set_motor_type(int motor_type)
  {
    std::stringstream ss;
    ss << "$mtype:" << motor_type << "#\r";
    send_command_and_wait_for_ack(ss.str());
  }

  void set_motor_mline(int mline)
  {
    std::stringstream ss;
    ss << "$mline:" << mline << "#\r";
    send_command_and_wait_for_ack(ss.str());
  }

  void set_motor_mphase(int mphase)
  {
    std::stringstream ss;
    ss << "$mphase:" << mphase << "#\r";
    send_command_and_wait_for_ack(ss.str());
  }

  void set_wheel_diam(int wdiameter)
  {
    std::stringstream ss;
    ss << "$wdiameter:" << wdiameter << "#\r";
    send_command_and_wait_for_ack(ss.str());
  }

  void set_motor_deadzone(int deadzone)
  {
    std::stringstream ss;
    ss << "$deadzone:" << deadzone << "#\r";
    send_command_and_wait_for_ack(ss.str());
  }

  void setup_motor_parameters(int mtype, int deadzone, int mline, int mphase, int wdiameter)
  {
    set_motor_type(mtype);
    set_motor_deadzone(deadzone);
    set_motor_mline(mline);
    set_motor_mphase(mphase);
    set_wheel_diam(wdiameter);
  }

  void read_flash_settings()
  {
    if (!connected()) return;
    
    clear_response_queue();
    send_command("$read_flash#");

    std::vector<std::string> responses;
    std::string line;

    // Wait for the first line containing "flash"
    if (get_response(line, std::chrono::milliseconds(500)))
    {
      if (line.find("read_flash") != std::string::npos)
      {
        responses.push_back(line);
        // Read the next 8 lines
        for (int i = 0; i < 8; ++i)
        {
          if (get_response(line, std::chrono::milliseconds(100)))
          {
            responses.push_back(line);
          }
          else
          {
            std::cerr << "Timeout waiting for flash setting line " << i + 2 << std::endl;
            break;
          }
        }
      }
    } else {
      std::cerr << "Timeout waiting for initial flash response." << std::endl;
    }

    std::cout << "Flash settings: " << std::endl;
    for(const auto& r : responses)
    {
      std::cout << r << std::endl;
    }
  }

private:
  std::unique_ptr<LibSerial::SerialPort> serial_conn_;
  int timeout_ms_ = 50;
  std::string serial_device_;
  int32_t baud_rate_;

  // Threading and data synchronization
  std::thread reader_thread_;
  std::atomic<bool> keep_reading_;
  
  std::mutex data_mutex_;
  double encoder_pos_[2] = {0.0, 0.0};
  double wheel_speeds_[2] = {0.0, 0.0};

  std::mutex response_mutex_;
  std::queue<std::string> response_queue_;
  std::condition_variable response_cv_;


  void reader_thread_func()
  {
    std::string current_line;
    while (keep_reading_)
    {
      try
      {
        if (serial_conn_->IsDataAvailable())
        {
          char next_char;
          serial_conn_->ReadByte(next_char, 5); // Short timeout
          if (next_char == '\n' || next_char == '\r')
          {
            if (!current_line.empty())
            {
              parse_line(current_line);
              current_line.clear();
            }
          }
          else
          {
            current_line += next_char;
          }
        }
        else
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }
      catch (const LibSerial::ReadTimeout&)
      {
        // This is expected, just continue
      }
      catch (const std::exception& e)
      {
        std::cerr << "Error in reader thread: " << e.what() << std::endl;
        keep_reading_ = false; // Stop thread on error
      }
    }
  }

  void parse_line(const std::string& line)
  {
    if (line.rfind("$MAll:", 0) == 0)
    {
      parse_encoder_response(line, "$MAll:", encoder_pos_);
    }
    else if (line.rfind("$MSPD:", 0) == 0)
    {
      parse_encoder_response(line, "$MSPD:", wheel_speeds_);
    }
    else
    {
      // General response, push to queue
      std::lock_guard<std::mutex> lock(response_mutex_);
      response_queue_.push(line);
      response_cv_.notify_one();
    }
  }

  void handle_reboot_and_reconnect(const std::string& reason)
  {
    std::cout << "Configuration updated (" << reason << "). The board needs to be rebooted." << std::endl;
    disconnect();

    std::cout << "\nPlease manually reboot the motor controller (unplug and plug it back in)." << std::endl;
    std::cout << "Press ENTER when you are ready to reconnect..." << std::flush;
    
    // Clear the input buffer and wait for the user to press Enter.
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    std::cout << "Attempting to reconnect..." << std::endl;

    bool reconnected = false;
    for (int i = 0; i < 10; ++i)
    {
      try
      {
        // Re-create the serial port object to ensure a clean state.
        serial_conn_ = std::make_unique<LibSerial::SerialPort>();
        connect(serial_device_, baud_rate_, timeout_ms_);
        if (connected())
        {
          std::cout << "Successfully reconnected." << std::endl;
          reconnected = true;
          read_flash_settings();
          break;
        }
      }
      catch(const std::exception& e)
      {
        std::cerr << "Reconnect attempt " << i + 1 << " failed. Retrying in 1 second..." << std::endl;
        disconnect(); // Ensure port is closed before next attempt
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (!reconnected)
    {
      throw std::runtime_error("Failed to reconnect to motor driver after " + reason);
    }
  }

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

  void send_command(const std::string &command)
  {
    if (!serial_conn_->IsOpen())
    {
      throw std::runtime_error("Serial connection not established");
    }
    serial_conn_->Write(command);
    serial_conn_->DrainWriteBuffer(); // Block until all data is written to the serial port.
  }

  void send_command_and_wait_for_ack(const std::string& command)
  {
    clear_response_queue();
    send_command(command);
    std::string response;
    if (get_response(response, std::chrono::milliseconds(200)))
    {
      std::cout << "Response: " << response << std::endl;
    }
    else
    {
      std::cout << "Warning: Timeout waiting for acknowledgment for command: " << command << std::endl;
    }
  }

  void clear_response_queue()
  {
    std::lock_guard<std::mutex> lock(response_mutex_);
    std::queue<std::string> empty;
    std::swap(response_queue_, empty);
  }

  bool get_response(std::string& response, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(response_mutex_);
    if (response_cv_.wait_for(lock, timeout, [this]{ return !response_queue_.empty(); }))
    {
      response = response_queue_.front();
      response_queue_.pop();
      return true;
    }
    return false; // Timeout
  }

  void parse_encoder_response(const std::string &response, const std::string &prefix, 
                             double* values)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
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
      
      // Left wheel
      if (std::getline(ss, val_str, ','))
      {
        values[0] = std::stod(val_str);
      }
      
      // Right wheel
      if (std::getline(ss, val_str, ','))
      {
        values[1] = -std::stod(val_str); // Invert right wheel
      }
    }
    catch (const std::exception &e)
    {
      std::cerr << "Error parsing encoder data: " << e.what() << " for line: " << response << std::endl;
      values[0] = values[1] = 0.0;
    }
  }
};

#endif // DIFFDRIVE_JETBOT_COMMS_HPP