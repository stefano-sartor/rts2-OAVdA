#include "connection_tcp.h"
#include <sys/select.h>
#include <sys/time.h>

#define CMD_READ_DATA 0x01
#define CMD_READ_SETTINGS 0x02 /*TODO*/

#define CMD_ECHO 0xFF

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

#define SYNC_AFTER 50

using boost::asio::ip::tcp;

ConnectionTCP::pointer ConnectionTCP::create(boost::asio::io_service &io_service, boost::asio::serial_port &s, std::mutex &m)
{
    return pointer(new ConnectionTCP(io_service, s, m));
}

tcp::socket &ConnectionTCP::socket()
{
    return _socket;
}

bool ConnectionTCP::try_sync()
{
    uint8_t reference = _rng();
    std::stringstream buffer;
    auto msg = msgpack::type::tuple<uint8_t, uint8_t>(CMD_ECHO, reference);
    msgpack::pack(buffer, msg);
    _off = buffer.str().copy(_data, BUFF_LEN, 0);
    {
        std::unique_lock<std::mutex> l(_ser_mtx);
        boost::system::error_code ec;
        _ser.write_some(boost::asio::buffer(_data, _off), ec);
        if (ec)
        {
            std::cout << "SERIAL ERROR " << ec.message() << std::endl;
            return false;
        }
        else
        {
            for (auto i = 0; i < 10; i++)
            {
                uint8_t err = 0;
                msgpack::object obj = decode_serial(err);
                if (err)
                    return false;
                if (obj.type == msgpack::type::ARRAY &&
                    obj.via.array.size == 2)
                {
                    try
                    {
                        uint8_t opc = obj.via.array.ptr[0].as<uint8_t>();
                        uint8_t chl = obj.via.array.ptr[1].as<uint8_t>();
                        if (opc == CMD_ECHO && chl == reference) //we've found the sync message, start listen for TCP requests
                        {
                            return true;
                        }
                    }
                    catch (const std::exception &ex)
                    {
                        continue;
                    }
                }
            }
        }
    }
    return false;
}

void ConnectionTCP::start()
{
    if (try_sync())
    {
        _off = 0;
        do_loop();
    }
    else
    {
        std::cout << "FAILED SYNC" << std::endl;
    }
}

void ConnectionTCP::do_loop()
{
    _socket.async_read_some(boost::asio::buffer(_data + _off, BUFF_LEN - _off),
                            boost::bind(&ConnectionTCP::handle_tcp_read, shared_from_this(),
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));
}

ConnectionTCP::ConnectionTCP(boost::asio::io_service &io_service, boost::asio::serial_port &s, std::mutex &m)
    : _off(0), _socket(io_service), _ser(s), _ser_mtx(m), _req_count(SYNC_AFTER)
{
}

void ConnectionTCP::handle_tcp_read(const boost::system::error_code &error,
                                    size_t bytes_transferred)
{
    if (error)
        return; // let the connection die
    _off += bytes_transferred;
    if (_off == 0) // should never happen
        return do_loop();

    msgpack::object_handle res;
    try
    {
        res = msgpack::unpack(_data, _off);
        return handle_request(res.get());
    }
    catch (const msgpack::insufficient_bytes &b)
    {
        return do_loop();
    }
    catch (const std::exception &ex)
    {
        std::cout << "error msgpack: " << ex.what() << std::endl;
        return; //there was an encoding error, let the connection die
    }
}

void ConnectionTCP::handle_tcp_write(const boost::system::error_code &error,
                                     size_t /*bytes_transferred*/)
{
    --_req_count;
    if (!error)
    {
        if (_req_count == 0)
        {
            if (try_sync())
                _req_count = SYNC_AFTER;
            else
                return;
        }
        _off = 0;
        do_loop();
    }
}

void ConnectionTCP::prepare_error(uint8_t err)
{
    std::stringstream buffer;
    auto msg = msgpack::type::tuple<uint8_t>(err);
    msgpack::pack(buffer, msg);
    _off = buffer.str().copy(_data, BUFF_LEN, 0);
}

msgpack::object ConnectionTCP::decode_serial(uint8_t &err)
{
    msgpack::object obj;
    _off = 0;

    auto s = _ser.native_handle();
    fd_set set;
    struct timeval timeout;
    FD_ZERO(&set);      /* clear the set */
    FD_SET(s, &set);    /* add our file descriptor to the set */
    timeout.tv_sec = 1; // we require asnswer in 1 second time
    timeout.tv_usec = 0;

    while (true)
    {
        int rv = select(s + 1, &set, NULL, NULL, &timeout);
        if (rv == -1)
        { // error occured
            std::cout << "ERROR SELECT" << std::endl;
            err = ERROR_COMM;
            break;
        }
        if (rv == 0) // timeout
        {
            std::cout << "SELECT TIMEOUT" << std::endl;
            err = ERROR_COMM;
            break;
        }

        auto asiobuff = boost::asio::buffer(_data + _off, BUFF_LEN - _off);
        _off += _ser.read_some(asiobuff);
        try
        {
            auto handle = msgpack::unpack(_data, _off);
            obj = handle.get();
            break;
        }
        catch (const msgpack::insufficient_bytes &b)
        {
            continue;
        }
        catch (const std::exception &ex)
        {
            std::cout << "error msgpack: " << ex.what() << std::endl;
            err = ERROR_BAD_MSG;
            break;
        }
        if (_off >= BUFF_LEN)
        {
            std::cout << "buffer full" << std::endl;
            err = ERROR_COMM;
            break;
        }
    }
    return obj;
}

void ConnectionTCP::handle_request(msgpack::object obj)
{
    if (obj.type != msgpack::type::ARRAY && obj.via.array.size < 2)
    {
        prepare_error(ERROR_BAD_MSG);
    }
    else
    {
        std::unique_lock<std::mutex> l(_ser_mtx);
        boost::system::error_code ec;
        _ser.write_some(boost::asio::buffer(_data, _off), ec);
        if (ec)
        {
            std::cout << "SERIAL ERROR " << ec.message() << std::endl;
            prepare_error(ERROR_COMM);
        }
        else
        {
            uint8_t err = 0;
            decode_serial(err);
            if (err)
            {
                prepare_error(err);
            }
        }
    }
    boost::asio::async_write(_socket, boost::asio::buffer(_data, _off),
                             boost::bind(&ConnectionTCP::handle_tcp_write, shared_from_this(),
                                         boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
}