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

using boost::asio::ip::tcp;

#include <mutex>
#include <thread>
#include <deque>

class Motor
{
public:
  Motor();
  void init(const std::string &name);
  uint8_t write_queue(int32_t steps, uint32_t speed);
  uint8_t write_now(int32_t steps, uint32_t speed);
  uint8_t write_goto(int32_t pos, uint32_t speed);

  void set_position(int32_t pos);

  void stop();

  void read_data(int8_t &dir, uint32_t &speed, int32_t &pos, uint8_t &buff_free);

private:
  typedef std::pair<int32_t, uint32_t> instruction;

  std::mutex _m;
  std::thread _th;
  void update();

  int32_t _target;
  int8_t _dir;
  uint32_t _speed;
  int32_t _pos;
  uint8_t _buff_free;

  std::string _name;
  std::deque<instruction> _q;
  int32_t _ticks;
};

class MotorDome
{
public:
  enum Status
  {
    STOP,
    MOVE_EAST,
    MOVE_WEST
  };
  MotorDome();
  void init();

  int32_t get_position();
  void set_position(int32_t pos);

  void go_to(int32_t pos);
  void go_rel(int32_t counts);
  void stop();

  void read_data(int8_t &dir, uint32_t &speed, int32_t &pos, uint8_t &buff_free);

private:
  std::mutex _m;
  std::thread _th;
  void update();

  int32_t _target;
  bool _stop;
  int8_t _dir;
  uint32_t _speed;
  int32_t _pos;
  uint8_t _buff_free;
};

class MotorFocus
{
public:
  enum Status
  {
    STOP = 0,
    SLOW_MOVE = 1,
    FAST_MOVE
  };
  MotorFocus();
  void init();

  int32_t get_position();
  void set_position(int32_t pos);

  void go_to(int32_t pos);
  void go_rel(int32_t counts);
  void stop();

  void read_data(int8_t &dir, uint32_t &speed, int32_t &pos, uint8_t &buff_free);

private:
  std::mutex _m;
  std::thread _th;
  void update();

  int32_t _target;
  bool _stop;
  int8_t _dir;
  uint32_t _speed;
  int32_t _pos;
  uint8_t _buff_free;
};

Motor motor_ra;
Motor motor_dec;

MotorDome motor_dome;
MotorFocus motor_focus;

class ConnectionTCP
    : public boost::enable_shared_from_this<ConnectionTCP>
{
public:
  typedef boost::shared_ptr<ConnectionTCP> pointer;

  static pointer create(boost::asio::io_service &io_service)
  {
    return pointer(new ConnectionTCP(io_service));
  }

  tcp::socket &socket()
  {
    return _socket;
  }

  void start()
  {
    _socket.async_read_some( boost::asio::buffer(_data + _off, 4096 - _off),
                            boost::bind(&ConnectionTCP::handle_read, shared_from_this(),
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));
  }

private:
  ConnectionTCP(boost::asio::io_service &io_service)
      : _off(0), _socket(io_service)
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
    if (!error){
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
      uint8_t op_code = obj.via.array.ptr[0].as<uint8_t>();
      uint8_t motor = obj.via.array.ptr[1].as<uint8_t>();
      std::stringstream buffer;
      //DBG std::cout << "REQ HEAD opcode[" << int(op_code) << "] motor["<<int(motor) << "]" << std::endl;
      switch (op_code)
      {
      case CMD_READ_DATA:
      {
        int8_t dir;
        uint32_t speed;
        int32_t pos;
        uint8_t buff_free;

        switch (motor)
        {
        case MOTOR_RA:
          motor_ra.read_data(dir, speed, pos, buff_free);
          break;
        case MOTOR_DEC:
          motor_dec.read_data(dir, speed, pos, buff_free);
          break;
        case MOTOR_DOME:
          motor_dome.read_data(dir, speed, pos, buff_free);
          break;
        case MOTOR_FOCUS:
          motor_focus.read_data(dir, speed, pos, buff_free);
          break;
        default:
          return send_error(ERROR_BAD_ARG);
          break;
        }
        msgpack::pack(buffer, msgpack::type::tuple<int8_t, uint32_t, int32_t, uint8_t>(dir, speed, pos, buff_free));
        _response = buffer.str();

        boost::asio::async_write(_socket, boost::asio::buffer(_response),
                                 boost::bind(&ConnectionTCP::handle_write, shared_from_this(),
                                             boost::asio::placeholders::error,
                                             boost::asio::placeholders::bytes_transferred));
        return;
      }
      break;
      case CMD_WRITE_QUEUE:
      {
        int32_t steps = obj.via.array.ptr[2].as<int32_t>();
        uint32_t speed = obj.via.array.ptr[3].as<uint32_t>();
        uint8_t res = ERROR_SUCCESS;
        switch (motor)
        {
        case MOTOR_RA:
          res = motor_ra.write_queue(steps, speed);
          break;
        case MOTOR_DEC:
          res = motor_dec.write_queue(steps, speed);
          break;
        default:
          res = ERROR_BAD_ARG;
          break;
        }
        return send_error(res); // might not be an error
      }
      break;
      case CMD_WRITE_NOW:
      {
        int32_t steps = obj.via.array.ptr[2].as<int32_t>();
        uint32_t speed = obj.via.array.ptr[3].as<uint32_t>();
        uint8_t res = ERROR_SUCCESS;
        switch (motor)
        {
        case MOTOR_RA:
          res = motor_ra.write_now(steps, speed);
          break;
        case MOTOR_DEC:
          res = motor_dec.write_now(steps, speed);
          break;
        case MOTOR_DOME:
          motor_dome.go_rel(steps);
          break;
        case MOTOR_FOCUS:
          motor_focus.go_rel(steps);
          break;
        default:
          res = ERROR_BAD_ARG;
          break;
        }
        return send_error(res); // might not be an error
      }
      break;
      case CMD_WRITE_GOTO:
      {
        int32_t pos = obj.via.array.ptr[2].as<int32_t>();
        uint32_t speed = obj.via.array.ptr[3].as<uint32_t>();
        uint8_t res = ERROR_SUCCESS;
        switch (motor)
        {
        case MOTOR_RA:
          res = motor_ra.write_goto(pos, speed);
          break;
        case MOTOR_DEC:
          res = motor_dec.write_goto(pos, speed);
          break;
        case MOTOR_DOME:
          motor_dome.go_to(pos);
          break;
        case MOTOR_FOCUS:
          motor_focus.go_to(pos);
          break;
        default:
          res = ERROR_BAD_ARG;
          break;
        }
        return send_error(res); // might not be an error
      }
      break;
      case CMD_WRITE_STOP:
      {
        uint8_t res = ERROR_SUCCESS;
        switch (motor)
        {
        case MOTOR_RA:
          motor_ra.stop();
          break;
        case MOTOR_DEC:
          motor_dec.stop();
          break;
        case MOTOR_DOME:
          motor_dome.stop();
          break;
        case MOTOR_FOCUS:
          motor_focus.stop();
          break;
        default:
          res = ERROR_BAD_ARG;
          break;
        }
        return send_error(res); // might not be an error
      }
      break;
      case CMD_WRITE_POSITION:
      {
        int32_t pos = obj.via.array.ptr[2].as<int32_t>();
        uint8_t res = ERROR_SUCCESS;
        switch (motor)
        {
        case MOTOR_RA:
          motor_ra.set_position(pos);
          break;
        case MOTOR_DEC:
          motor_dec.set_position(pos);
          break;
        case MOTOR_DOME:
          motor_dome.set_position(pos);
          break;
        case MOTOR_FOCUS:
          motor_focus.set_position(pos);
          break;
        default:
          res = ERROR_BAD_ARG;
          break;
        }
        return send_error(res); // might not be an error
      }
      break;
      default:
        return send_error(ERROR_BAD_MSG);
        break;
      }
    }
    catch (const std::exception &ex)
    {
      std::cout << "error handling obj " << ex.what() << std::endl;
      return send_error(ERROR_UNKNOWN_MSG);
    }
  }

  std::string _response;
  size_t _off;
  char _data[4096];
  tcp::socket _socket;
  std::string _message;
};

class ServerTCP
{
public:
  ServerTCP(boost::asio::io_service &io_service)
      : acceptor_(io_service, tcp::endpoint(tcp::v4(), 1500))
  {
    start_accept();
  }

private:
  void start_accept()
  {
    ConnectionTCP::pointer new_connection =
        ConnectionTCP::create(acceptor_.get_io_service());

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
};

int main()
{
  motor_ra.init("RA");
  motor_dec.init("Dec");
  motor_dome.init();
  motor_focus.init();
  try
  {
    boost::asio::io_service io_service;
    ServerTCP server(io_service);
    io_service.run();
  }
  catch (std::exception &e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *               teensy simulator
 *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include <functional> // std::bind
#include <chrono>     // std::chrono::seconds
#include <iostream>

void Motor::init(const std::string &name)
{
  _name = name;
  _th = std::thread(std::bind(&Motor::update, this));
}

void MotorDome::init()
{
  _th = std::thread(std::bind(&MotorDome::update, this));
}

void MotorFocus::init()
{
  _th = std::thread(std::bind(&MotorFocus::update, this));
}

Motor::Motor() : _dir(0),
                 _speed(0),
                 _pos(1500000),
                 _buff_free(0),
                 _ticks(0) {}

MotorDome::MotorDome() : _target(0),
                         _stop(true),
                         _dir(0),
                         _speed(0),
                         _pos(0),
                         _buff_free(0) {}

MotorFocus::MotorFocus() : _target(0),
                           _stop(true),
                           _dir(0),
                           _speed(0),
                           _pos(0),
                           _buff_free(0) {}

void Motor::update()
{
  while (true)
  {
    auto t0 = std::chrono::system_clock::now();
    std::this_thread::sleep_for(std::chrono::microseconds(35));
    {
      std::unique_lock<std::mutex> l(_m);
      if (_q.empty())
      {
        _dir = 0;
        _speed = 0;
        _ticks = 0;
        continue;
      }

      auto &is = _q.front();
      _speed = is.second;
      _dir = is.first > 0 ? 1 : 0;
      if (_ticks <= 1)
      {
        _ticks = is.second;
        if (is.first < 0)
        {
          _pos--;
          is.first++;
        }
        else
        {
          _pos++;
          is.first--;
        }
        if (is.first == 0)
          _q.pop_front();
      }
      else
      {
        _ticks--;
      }
    }

    auto t1 = std::chrono::system_clock::now();
    std::this_thread::sleep_for(std::chrono::microseconds(35 - (t1-t0).count()*1000000));


  }
}

uint8_t Motor::write_queue(int32_t steps, uint32_t speed)
{
  std::unique_lock<std::mutex> l(_m);
  _q.emplace_back(steps, speed);
  std::cout << _name << "[WRITE_QUEUE] " << steps << " " << speed << std::endl;
  return 0;
}

uint8_t Motor::write_now(int32_t steps, uint32_t speed)
{
  std::unique_lock<std::mutex> l(_m);
  _q.clear();
  _q.emplace_back(steps, speed);
  std::cout << _name << "[WRITE_NOW] " << steps << " " << speed << std::endl;
  return 0;
}

uint8_t Motor::write_goto(int32_t pos, uint32_t speed)
{
  std::unique_lock<std::mutex> l(_m);
  _q.clear();
  _q.emplace_back(pos - _pos, speed);
  std::cout << _name << "[WRITE_GOTO] " << pos << " " << speed << std::endl;
  return 0;
}

void Motor::set_position(int32_t pos)
{
  _pos = pos;
}

void Motor::stop()
{
  std::cout << _name << "[STOP start] " << std::endl;
  std::unique_lock<std::mutex> l(_m);
  _q.clear();
  _q.emplace_back(4929, 2);
  _q.emplace_back(1408, 3);
  _q.emplace_back(704, 4);
  _q.emplace_back(282, 5);
  _q.emplace_back(235, 6);
  _q.emplace_back(201, 7);
  _q.emplace_back(156, 8);
  _q.emplace_back(100, 14);
  _q.emplace_back(33, 28);

  while (!_q.empty() && _q.front().second > _speed)
    _q.pop_front();
  if (_dir == 0)
  {
    for (auto &i : _q)
    {
      i.first *= -1;
    }
  }
  std::cout << _name << "[STOP] " << _q.size() << " decel steps" << std::endl;
}

void Motor::read_data(int8_t &dir, uint32_t &speed, int32_t &pos, uint8_t &buff_free)
{
  std::unique_lock<std::mutex> l(_m);
  dir = _dir;
  speed = _speed;
  pos = _pos;
  buff_free = _q.size();
  /* //DBG std::cout << _name << "[READ_DATA] "
            << int(_dir) << " "
            << speed << " "
            << pos << " "
            << int(buff_free) << " "
            << std::endl;*/
}

void MotorFocus::read_data(int8_t &dir, uint32_t &speed, int32_t &pos, uint8_t &buff_free)
{
  std::unique_lock<std::mutex> l(_m);
  dir = _dir;
  speed = _speed;
  pos = _pos;
  buff_free = _buff_free;
}

void MotorDome::read_data(int8_t &dir, uint32_t &speed, int32_t &pos, uint8_t &buff_free)
{
  std::unique_lock<std::mutex> l(_m);
  dir = _dir;
  speed = _speed;
  pos = _pos;
  buff_free = _buff_free;
}

void MotorFocus::update()
{
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    {
      std::unique_lock<std::mutex> l(_m);
      if (_stop)
        continue;
      int32_t dist = _target - _pos;
      if (dist > 0)
      {
        _dir = 1;
        _speed = dist > 50 ? 50 : dist;
        _pos += _speed;
      }
      else if (dist < 0)
      {
        _dir = 0;
        _speed = dist < -50 ? 50 : dist * -1;
        _pos -= _speed;
      }
      else
      { // dist 0
        _stop = true;
        _speed = 0;
        _dir = 0;
      }
    }
  }
}

void MotorDome::update()
{
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    {
      std::unique_lock<std::mutex> l(_m);
      if (_stop)
        continue;
      int32_t dist = _target - _pos;
      if (dist > 0)
      {
        _dir = 1;
        _speed = 1;
        _pos += _speed;
      }
      else if (dist < 0)
      {
        _dir = 0;
        _speed = 1;
        _pos -= _speed;
      }
      else
      { // dist 0
        _stop = true;
        _speed = 0;
        _dir = 0;
      }
    }
  }
}

int32_t MotorFocus::get_position()
{
  std::unique_lock<std::mutex> l(_m);
  return _pos;
}

void MotorFocus::set_position(int32_t pos)
{
  std::unique_lock<std::mutex> l(_m);
  _stop = true;
  _speed = 0;
  _dir = 0;
  _pos = pos;
  _target = pos;
}

void MotorFocus::go_to(int32_t pos)
{
  std::unique_lock<std::mutex> l(_m);
  _stop = false;
  _target = pos;
}

void MotorFocus::go_rel(int32_t counts)
{
  std::unique_lock<std::mutex> l(_m);
  _stop = false;
  _target = _pos + counts;
}

void MotorFocus::stop()
{
  std::unique_lock<std::mutex> l(_m);
  _stop = true;
  _speed = 0;
  _dir = 0;
}

int32_t MotorDome::get_position()
{
  std::unique_lock<std::mutex> l(_m);
  return _pos;
}

void MotorDome::set_position(int32_t pos)
{
  std::unique_lock<std::mutex> l(_m);
  _stop = true;
  _speed = 0;
  _dir = 0;
  _pos = pos;
  _target = pos;
}

void MotorDome::go_to(int32_t pos)
{
  std::unique_lock<std::mutex> l(_m);
  _stop = false;
  _target = pos;
}

void MotorDome::go_rel(int32_t counts)
{
  std::unique_lock<std::mutex> l(_m);
  _stop = false;
  _target = _pos + counts;
}

void MotorDome::stop()
{
  std::unique_lock<std::mutex> l(_m);
  _stop = true;
  _speed = 0;
  _dir = 0;
}