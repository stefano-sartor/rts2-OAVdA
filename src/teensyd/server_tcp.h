#ifndef OAVDA_SERVER_TCP_H
#define OAVDA_SERVER_TCP_H

#include "connection_tcp.h"

class ServerTCP
{
public:
  ServerTCP(boost::asio::io_service &io_service, boost::asio::serial_port &s);

private:
  void start_accept();

  void handle_accept(ConnectionTCP::pointer new_connection,
                     const boost::system::error_code &error);

  boost::asio::ip::tcp::acceptor acceptor_;
  boost::asio::serial_port &_ser;
  std::mutex  _ser_mtx;
};

#endif