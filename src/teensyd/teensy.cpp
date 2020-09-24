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

class ConnectionTCP
    : public boost::enable_shared_from_this<ConnectionTCP>
{
public:
  typedef boost::shared_ptr<ConnectionTCP> pointer;

  static pointer create(boost::asio::io_service &io_service, boost::asio::serial_port &s, std::mutex &m)
  {
    return pointer(new ConnectionTCP(io_service, s, m));
  }

  tcp::socket &socket()
  {
    return _socket;
  }

  void start()
  {
    _socket.async_read_some(boost::asio::buffer(_data + _off, 4096 - _off),
                            boost::bind(&ConnectionTCP::handle_read, shared_from_this(),
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));
  }

private:
  ConnectionTCP(boost::asio::io_service &io_service, boost::asio::serial_port &s, std::mutex &m)
      : _off(0), _socket(io_service), _ser(s), _ser_mtx(m)
  {
  }

  void handle_read(const boost::system::error_code &error,
                   size_t bytes_transferred)
  {
    if (error)
      return;
    _off += bytes_transferred;
    if (_off == 0) // should never happen
      return start();

    msgpack::object_handle res;
    try
    {
      res = msgpack::unpack(_data, _off);
      return handle_request(res.get());
    }
    catch (const msgpack::insufficient_bytes &b)
    {
      return start();
    }
    catch (const std::exception &ex)
    {
      std::cout << "error msgpack: " << ex.what() << std::endl;
      _off = 0;
      return start();
    }
  }

  void handle_write(const boost::system::error_code &error,
                    size_t /*bytes_transferred*/)
  {
    if (!error)
    {
      _off = 0;
      start();
    }
  }

  void send_error(uint8_t err)
  {
    //DBG std::cout << "RETURNING ERROR " << int(err) << std::endl;
    std::stringstream buffer;
    auto msg = msgpack::type::tuple<uint8_t>(err);
    msgpack::pack(buffer, msg);
    _response = buffer.str();

    boost::asio::async_write(_socket, boost::asio::buffer(_response),
                             boost::bind(&ConnectionTCP::handle_write, shared_from_this(),
                                         boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
    return;
  }

  void handle_request(msgpack::object obj)
  {
    if (obj.type != msgpack::type::ARRAY && obj.via.array.size < 2)
    {
      return send_error(ERROR_BAD_MSG);
    }

    try
    {
      std::unique_lock<std::mutex> l(_ser_mtx);
      _ser.write_some(boost::asio::buffer(_data, _off));

      bool ok = false;
      auto start = std::chrono::system_clock::now();
      auto end = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = end - start;
      _off = 0;

      //FIXME  SUPERBAD use select... read_some does not return until at least 1 byte is red
      while (diff.count() <= 1 && _off < BUFF_LEN) // we require asnswer in 1 second time
      {

        auto asiobuff = boost::asio::buffer(_data + _off, 1);
        _off += _ser.read_some(asiobuff);
        ///DBG       std::cout << "READ " << s << " bytes" << std::endl;

        end = std::chrono::system_clock::now();
        diff = end - start;

        if (_off == 0)
          continue;

        try
        {
          msgpack::unpack(_data, _off);
          ok = true;
          break;
        }
        catch (const msgpack::insufficient_bytes &b)
        {
          continue;
        }
        catch (const std::exception &ex)
        {
          std::cout << "error msgpack: " << ex.what() << std::endl;
          break;
        }
      }
      if (ok)
      {
        boost::asio::async_write(_socket, boost::asio::buffer(_data,_off),
                                 boost::bind(&ConnectionTCP::handle_write, shared_from_this(),
                                             boost::asio::placeholders::error,
                                             boost::asio::placeholders::bytes_transferred));
        return;
      }
      if (diff.count() > 1)
      {
        std::cout << "TIMEOUT PDD" << std::endl;
        return send_error(ERROR_COMM);
      }
      else
        return send_error(ERROR_BAD_MSG);
    }
    catch (const std::exception &ex)
    {
      std::cout << "error handling obj " << ex.what() << std::endl;
      return send_error(ERROR_UNKNOWN_MSG);
    }
  }

  static constexpr size_t BUFF_LEN = 4096;
  std::string _response;
  size_t _off;
  char _data[BUFF_LEN];
  tcp::socket _socket;
  std::string _message;

  boost::asio::serial_port &_ser;
  std::mutex &_ser_mtx;
};

class ServerTCP
{
public:
  ServerTCP(boost::asio::io_service &io_service, boost::asio::serial_port &s)
      : acceptor_(io_service, tcp::endpoint(tcp::v4(), 1500)),
        _ser(s)
  {
    start_accept();
  }

private:
  void start_accept()
  {
    ConnectionTCP::pointer new_connection =
        ConnectionTCP::create(acceptor_.get_io_service(), _ser, _ser_mtx);

    acceptor_.async_accept(new_connection->socket(),
                           boost::bind(&ServerTCP::handle_accept, this, new_connection,
                                       boost::asio::placeholders::error));
  }

  void handle_accept(ConnectionTCP::pointer new_connection,
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

  tcp::acceptor acceptor_;

  boost::asio::serial_port &_ser;
  std::mutex _ser_mtx;
};

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