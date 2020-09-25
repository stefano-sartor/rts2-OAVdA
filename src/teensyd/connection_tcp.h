#ifndef OAVDA_CONNECTION_TCP_H
#define OAVDA_CONNECTION_TCP_H
#include <ctime>
#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <msgpack.hpp>
#include <mutex>
#include <thread>
#include <deque>
#include <boost/random/mersenne_twister.hpp>

class ConnectionTCP
    : public boost::enable_shared_from_this<ConnectionTCP>
{
public:
  typedef boost::shared_ptr<ConnectionTCP> pointer;

  static pointer create(boost::asio::io_service &io_service, boost::asio::serial_port &s, std::mutex &m);

  boost::asio::ip::tcp::socket &socket();

  void start();

private:
  bool try_sync();

  void do_loop();
  ConnectionTCP(boost::asio::io_service &io_service, boost::asio::serial_port &s, std::mutex &m);

  void handle_tcp_read(const boost::system::error_code &error, size_t bytes_transferred);

  void handle_tcp_write(const boost::system::error_code &error, size_t bytes_transferred);

  void prepare_error(uint8_t err);

  void handle_request(msgpack::object obj);

  msgpack::object decode_serial(uint8_t& err);

  static constexpr size_t BUFF_LEN = 4096;
  size_t _off;
  char _data[BUFF_LEN];
  boost::asio::ip::tcp::socket _socket;
  std::string _message;

  boost::asio::serial_port &_ser;
  std::mutex &_ser_mtx;

  size_t _req_count;
  boost::random::mt19937 _rng;
};


#endif