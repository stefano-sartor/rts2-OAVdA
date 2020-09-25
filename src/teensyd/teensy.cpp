#include <ctime>
#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <msgpack.hpp>

#define CMD_READ_DATA 0x01
#define CMD_READ_SETTINGS 0x02 /*TODO*/

#define CMD_WRITE_QUEUE 0x10
#define CMD_WRITE_NOW 0x20
#define CMD_WRITE_GOTO 0x30
#define CMD_WRITE_STOP 0x40
#define CMD_WRITE_SETTINGS 0x50 /*TODO*/
#define CMD_WRITE_POSITION 0x60

#define MOTOR_RA 0x00
#define MOTOR_DEC 0x01
#define MOTOR_DOME 0x02
#define MOTOR_FOCUS 0x03

#define ERROR_SUCCESS 0x00
#define ERROR_OUT_RANGE 0x01
#define ERROR_BAD_MSG 0x02
#define ERROR_BAD_ARG 0x03
#define ERROR_UNKNOWN_MSG 0x04

#define ERROR_QUEUE_FULL 0x10
#define ERROR_QUEUE_BAD_REQ 0x20

#define ERROR_COMM 0xFF

using boost::asio::ip::tcp;

#include <mutex>
#include <thread>
#include <deque>
#include "server_tcp.h"


int main()
{
  std::string path = "/dev/ttyACM0";
  int baud = 2000000;
  try
  {
    boost::asio::io_service io_service;
    boost::asio::serial_port ser(io_service);
    ser.open(path);
    ser.set_option(boost::asio::serial_port_base::baud_rate(baud));
    ServerTCP server(io_service, ser);
    io_service.run();
  }
  catch (std::exception &e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}