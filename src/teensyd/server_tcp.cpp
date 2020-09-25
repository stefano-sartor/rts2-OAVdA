#include "server_tcp.h"

using boost::asio::ip::tcp;

ServerTCP::ServerTCP(boost::asio::io_service &io_service, boost::asio::serial_port &s)
    : acceptor_(io_service, tcp::endpoint(tcp::v4(), 1500)),
      _ser(s)
{
  start_accept();
}

void ServerTCP::start_accept()
{
  ConnectionTCP::pointer new_connection =
      ConnectionTCP::create(acceptor_.get_io_service(), _ser, _ser_mtx);

  acceptor_.async_accept(new_connection->socket(),
                         boost::bind(&ServerTCP::handle_accept, this, new_connection,
                                     boost::asio::placeholders::error));
}

void ServerTCP::handle_accept(ConnectionTCP::pointer new_connection,
                              const boost::system::error_code &error)
{
  std::cout << "HANDLE ACCEPT" << std::endl;
  if (!error)
  {
    std::cout << "no error" << std::endl;
    new_connection->start();
    std::cout << "new connection started" << std::endl;
  }

  start_accept();
}