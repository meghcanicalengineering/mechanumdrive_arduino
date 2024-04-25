#ifndef MECHANUMDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define MECHANUMDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
    switch (baud_rate)
    {
        case 1200: return LibSerial::BaudRate::BAUD_1200;
        case 1800: return LibSerial::BaudRate::BAUD_1800;
        case 2400: return LibSerial::BaudRate::BAUD_2400;
        case 4800: return LibSerial::BaudRate::BAUD_4800;
        case 9600: return LibSerial::BaudRate::BAUD_9600;
        case 19200: return LibSerial::BaudRate::BAUD_19200;
        case 38400: return LibSerial::BaudRate::BAUD_38400;
        case 57600: return LibSerial::BaudRate::BAUD_57600;
        case 115200: return LibSerial::BaudRate::BAUD_115200;
        case 230400: return LibSerial::BaudRate::BAUD_230400;
        default:
            std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
            return LibSerial::BaudRate::BAUD_57600;
    }
}

class ArduinoComms
{
public:
    ArduinoComms() = default;

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

    std::string send_msg(const std::string &msg_to_send, bool print_output = false)
    {
        serial_conn_.FlushIOBuffers(); // Clear buffers
        serial_conn_.Write(msg_to_send);

        std::string response = "";
        try
        {
            serial_conn_.ReadLine(response, '\n', timeout_ms_);
        }
        catch (const LibSerial::ReadTimeout&)
        {
            std::cerr << "Read timeout." << std::endl;
        }

        if (print_output)
        {
            std::cout << "Sent: " << msg_to_send << " Received: " << response << std::endl;
        }

        return response;
    }

    void read_encoder_values(int &fl_val, int &fr_val, int &rl_val, int &rr_val)
    {
        std::string response = send_msg("e\r");

        std::istringstream iss(response);
        iss >> fl_val >> fr_val >> rl_val >> rr_val;
    }

    void set_motor_values(int fl_val, int fr_val, int rl_val, int rr_val)
    {
        std::stringstream ss;
        ss << "m " << fl_val << " " << fr_val << " " << rl_val << " " << rr_val << "\r";
        send_msg(ss.str());
    }

    void set_pid_values(int k_p, int k_d, int k_i, int k_o)
    {
        std::stringstream ss;
        ss << "p " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
        send_msg(ss.str());
    }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // MECHANUMDRIVE_ARDUINO_ARDUINO_COMMS_HPP
