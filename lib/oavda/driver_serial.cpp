#include "oavda/driver_serial.h"
#include <msgpack.hpp>
#include <mutex>
#include <iostream>
#include <sstream>
#include <boost/asio.hpp>
#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds

#define CMD_READ_DATA 0x01
#define CMD_READ_SETTINGS 0x02 /*TODO*/

#define CMD_WRITE_QUEUE 0x10
#define CMD_WRITE_NOW 0x20
#define CMD_WRITE_GOTO 0x30
#define CMD_WRITE_STOP 0x40
#define CMD_WRITE_SETTINGS 0x50 /*TODO*/
#define CMD_WRITE_POSITION 0x60

#define STEPS_RA 4160342
#define TICK_US (71 / 2)
#define US 1000000
#define TICK_S (US / TICK_US)

#define MAX_SPEED (US / 40)
#define TRACK_SPEED (STEPS_RA / 86400)
#define STOP_SPEED (TRACK_SPEED)

#define LIMIT_STEPS 1000000

namespace oavda
{
    std::unique_ptr<boost::asio::serial_port> ser;
    std::mutex mtx_ser;

    template <typename T>
    msgpack::object send_no_block(T &msg, boost::system::error_code &ec)
    {

        msgpack::object obj;
        std::stringstream buffer;
        msgpack::pack(buffer, msg);
        boost::asio::write(*ser, boost::asio::buffer(buffer.str()), ec);
        if (ec)
            return obj;

        constexpr size_t BUFF_LEN = 1024;
        char buff[BUFF_LEN];
        size_t s = 0;
        msgpack::object_handle res;
        bool ok = false;
        while (true)
        {
            auto asiobuff = boost::asio::buffer(buff + s, BUFF_LEN);
            s += ser->read_some(asiobuff, ec);

            if (s == 0)
                continue;

            if (ec)
                return obj;

            try
            {
                res = msgpack::unpack(buff, s);
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
            obj = res.get();
        else
            ec = boost::system::error_code(boost::system::errc::bad_message, boost::system::posix_category);

        return obj;
    }

    template <typename T>
    msgpack::object send(T &msg, boost::system::error_code &ec)
    {
        std::unique_lock<std::mutex> lck(mtx_ser);
        return send_no_block(msg, ec);
    }

    error_t decode_1(msgpack::object res, boost::system::error_code ec)
    {
        if (ec)
            return ERROR_COMM;

        if (res.type != msgpack::type::ARRAY)
            return ERROR_COMM;

        if (res.via.array.size != 1)
            return ERROR_COMM;

        try
        {
            return res.via.array.ptr[0].as<error_t>();
        }
        catch (const std::exception &)
        {
        }

        return ERROR_COMM;
    }

    //TODO
    int serial_init(const char *serial, int boud)
    {
        return 0;
    }

    error_t AxisBase::stop()
    {
        auto msg = msgpack::type::tuple<uint8_t, uint8_t>(CMD_WRITE_STOP, _motor);
        boost::system::error_code ec;

        msgpack::object res = send(msg, ec);

        return decode_1(res, ec);
    }

    error_t AxisBase::set_position(int32_t pos)
    {
        auto msg = msgpack::type::tuple<uint8_t, uint8_t, int32_t>(CMD_WRITE_POSITION, _motor, pos);
        boost::system::error_code ec;

        msgpack::object res = send(msg, ec);

        return decode_1(res, ec);
    }

    error_t AxisBase::go_rel(int32_t steps, uint32_t speed)
    {
        auto msg = msgpack::type::tuple<uint8_t, uint8_t, int32_t, int32_t>(CMD_WRITE_NOW, _motor, steps, speed);
        boost::system::error_code ec;

        msgpack::object res = send(msg, ec);

        return decode_1(res, ec);
    }

    error_t AxisBase::go_abs(int32_t pos, uint32_t speed)
    {
        auto msg = msgpack::type::tuple<uint8_t, uint8_t, int32_t, int32_t>(CMD_WRITE_GOTO, _motor, pos, speed);
        boost::system::error_code ec;

        msgpack::object res = send(msg, ec);

        return decode_1(res, ec);
    }

    error_t AxisAdv::append(int32_t steps, uint32_t speed)
    {
        auto msg = msgpack::type::tuple<uint8_t, uint8_t, int32_t, int32_t>(CMD_WRITE_QUEUE, _motor, steps, speed);
        boost::system::error_code ec;

        msgpack::object res = send(msg, ec);

        return decode_1(res, ec);
    }

    error_t AxisBase::read_hk(int8_t &dir, uint32_t &speed, int32_t &pos, uint8_t &buff_free)
    {
        auto msg = msgpack::type::tuple<uint8_t, uint8_t>(CMD_READ_DATA, _motor);
        boost::system::error_code ec;

        msgpack::object res = send(msg, ec);
        if (ec)
            return ERROR_COMM;

        if (res.type != msgpack::type::ARRAY)
            return ERROR_COMM;

        if (res.via.array.size != 4)
            return ERROR_COMM;

        try
        {
            dir = res.via.array.ptr[0].as<int8_t>();
            speed = res.via.array.ptr[1].as<uint32_t>();
            pos = res.via.array.ptr[2].as<int32_t>();
            buff_free = res.via.array.ptr[3].as<uint8_t>();
        }
        catch (const std::exception &)
        {
        }

        return ERROR_COMM;
    }

    int8_t AxisBase::get_dir(error_t &err)
    {
        int8_t dir;
        uint32_t speed;
        int32_t pos;
        uint8_t buff_free;

        err = read_hk(dir, speed, pos, buff_free);
        return dir;
    }

    uint32_t AxisBase::get_speed(error_t &err)
    {
        int8_t dir;
        uint32_t speed;
        int32_t pos;
        uint8_t buff_free;

        err = read_hk(dir, speed, pos, buff_free);
        return speed;
    }

    int32_t AxisBase::get_pos(error_t &err)
    {
        int8_t dir;
        uint32_t speed;
        int32_t pos;
        uint8_t buff_free;

        err = read_hk(dir, speed, pos, buff_free);
        return pos;
    }

    uint8_t AxisAdv::get_buff_free(error_t &err)
    {
        int8_t dir;
        uint32_t speed;
        int32_t pos;
        uint8_t buff_free;

        err = read_hk(dir, speed, pos, buff_free);
        return buff_free;
    }

    inline float tiks2speed(const uint32_t &speed_tiks)
    {
        if (speed_tiks == 0)
            return 0;
        else
            return float(TICK_S) / float(speed_tiks);
    }

    inline uint32_t speed2ticks(const float &speed)
    {
        return round(TICK_S / speed);
    }

    inline float ticks2s(uint32_t ticks)
    {
        return float(ticks * TICK_US) / float(US);
    }

    inline float commands2s(const std::deque<std::pair<int32_t, uint32_t>> &c){
        float s = 0;
        for (const auto& i : c){
            s += ticks2s(i.second) * abs(i.first);
        }
        return s;
    }

    error_t AxisStepper::bulk(bool start_now)
    {
        error_t err = 0;
        _time = commands2s(_command_array);

        if(start_now && !_command_array.empty()){
            err = go_rel(_command_array.front().first,_command_array.front().second);
            _command_array.pop_front();
        }
        for(auto& c : _command_array){
            if(err) return err;
            err = append(c.first,c.second);
        }
        return err;
    }

    inline error_t AxisStepper::hk()
    {
        error_t err = read_hk(_dir, _speed_us, _position, _buff_free);
        if (err)
            return err;

        if (_dir == 0)
            _dir = -1;

        _speed_sps = tiks2speed(_speed_us) * _dir;
        return 0;
    }

    int32_t AxisStepper::get_position(error_t &err)
    {
        err = hk();
        return _position;
    }

    error_t AxisStepper::set_position(int32_t pos)
    {
        error_t err = AxisAdv::set_position(pos);
        _position = pos;
        return err;
    }

    void AxisStepper::compute_acc(float i, float &f, std::deque<uint32_t> &a, size_t max_steps)
    {
        size_t stop_steps = a.size() + max_steps;
        float v0 = i;
        float v = v0;
        float t = ticks2s(speed2ticks(v0));
        while (v <= f && a.size() <= stop_steps)
        {
            v = (_acc * t) + v0;
            uint32_t ticks = speed2ticks(v);
            v = speed2ticks(ticks);
            t += ticks2s(ticks);
            a.push_back(ticks);
        }
        f = v;
    }

    template <typename IT>
    void AxisStepper::compress(IT b, IT e, command_t &acc, int sgn)
    {

        for (auto it = b; it != e; it++)
        {
            if (acc.empty() || acc.back().second != *it)
                acc.emplace_back(sgn, *it);
            else
                acc.back().first += sgn;
        }
    }

    void AxisStepper::enque_stop(int32_t &position)
    {
        if (abs(_speed_sps) <= STOP_SPEED)
        { // safe speed to stop
            _speed_sps = 0;
            return;
        }
        int sgn = _speed_sps < 0 ? -1 : 1;
        speed_t a;
        float speed = abs(_speed_sps);
        compute_acc(STOP_SPEED, speed, a, LIMIT_STEPS);
        position += a.size() * sgn;
        compress(a.rbegin(), a.rend(), _command_array, sgn);
    }

    float AxisStepper::go_to(int32_t pos, float max_speed, error_t &err)
    {
        err = hk();
        if (err)
            return 0;

        _command_array.clear(); //start fresh
        _time = 0;              //start fresh

        int32_t steps = pos - _position;
        if ((_speed_sps < 0 && steps > 0) ||                   // do we need to reverse movement?
            (_speed_sps > 0 && steps < 0) ||                   // as above
            abs(steps) < (max_speed * max_speed) / (2 * _acc)) // is the target too close to brake at current speed?
        {                                                      // we need to reverse the movement
            enque_stop(_position);
            steps = pos - _position; // abs(steps) should increase and not change sign since we stop in the same movement direction
        }

        speed_t speed_acc, speed_decel;
        command_t commands;
        int sgn = _speed_sps < 0 ? -1 : 1;

        float final_speed = max_speed;
        compute_acc(abs(_speed_sps), final_speed, speed_acc, abs(steps) / 2);
        int32_t remain_steps = abs(steps) - speed_acc.size();
        compute_acc(STOP_SPEED, final_speed, speed_decel, LIMIT_STEPS);
        remain_steps -= speed_decel.size();
        while (remain_steps < 0)
        { //not enough steps to stop -> erase half ecced steps form acc and half from decel
            if (!speed_acc.empty())
            {
                speed_acc.pop_back();
                remain_steps++;
            }
            if (!speed_decel.empty() && remain_steps < 0)
            {
                speed_decel.pop_back();
                remain_steps++;
            }
        }
        compress(speed_acc.begin(), speed_acc.end(), _command_array, sgn); // append acceleration
        if (remain_steps > 0)                                              // append constant speed
        {
            if (!speed_acc.empty())
                _command_array.emplace_back(remain_steps * sgn, speed_acc.back());
            else if (!speed_decel.empty())
                _command_array.emplace_back(remain_steps * sgn, speed_decel.back());
            else
                _command_array.emplace_back(remain_steps * sgn, speed2ticks(max_speed));
        }
        compress(speed_decel.rbegin(), speed_decel.rend(), _command_array, sgn); // append deceleration

        err = bulk(true);
        _target = pos;
        return _time;
    }

} // namespace oavda