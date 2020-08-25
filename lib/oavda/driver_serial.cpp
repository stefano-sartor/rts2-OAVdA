#include "oavda/driver_serial.h"
#include <mutex>
#include <iostream>
#include <sstream>
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
#define TRACK_END STEPS_RA
#define TICK_US (71 / 2)
#define US 1000000
#define TICK_S (US / TICK_US)

#define MAX_SPEED (US / 40)
//#define TRACK_SPEED (STEPS_RA / 86400)
#define STOP_SPEED ((STEPS_RA / 86400)*0.5)

#define LIMIT_STEPS 1000000

using boost::asio::ip::tcp;

namespace oavda
{

    template <typename T>
    msgpack::object_handle AxisBase::send(T &msg, boost::system::error_code &ec)
    {
        msgpack::object_handle res;
        std::stringstream buffer;
        msgpack::pack(buffer, msg);
        _skt.write_some(boost::asio::buffer(buffer.str()), ec);
        if (ec)
        {
            reconnect();
            return res;
        }
        constexpr size_t BUFF_LEN = 1024;
        char buff[BUFF_LEN];
        size_t s = 0;

        bool ok = false;
        auto start = std::chrono::system_clock::now();
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end - start;

        //FIXME  SUPERBAD use select... read_some does not return until at least 1 byte is red
        while (diff.count() <= 1 && s < BUFF_LEN) // we require asnswer in 1 second time
        {

            auto asiobuff = boost::asio::buffer(buff + s, 1);
            s += _skt.read_some(asiobuff, ec);
            ///DBG       std::cout << "READ " << s << " bytes" << std::endl;

            end = std::chrono::system_clock::now();
            diff = end - start;

            if (s == 0)
                continue;

            if (ec)
            {
                reconnect();
                return res;
            }

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
        {
            ///DBG   if (res.get().type == msgpack::type::ARRAY &&
            ///DBG       res.get().via.array.size > 1)
            ///DBG       std::cout << "read res  [" << int(res.get().via.array.ptr[0].type) << "] " << std::endl;

            return res;
        }
        if (diff.count() > 1)
        {
            std::cout << "TIMEOUT PDD" << std::endl;
            ec = boost::system::errc::make_error_code(boost::system::errc::timed_out);
        }
        else
            ec = boost::system::errc::make_error_code(boost::system::errc::bad_message);

        return res;
    }

    error_t decode_1(msgpack::object_handle& res, boost::system::error_code ec)
    {
        if (ec)
            return ERROR_COMM;

        if (res.get().type != msgpack::type::ARRAY)
            return ERROR_COMM;

        if (res.get().via.array.size != 1)
        {
            std::cout << "decode_1  wrong array size, got " << int(res.get().via.array.size) << std::endl;
            return ERROR_COMM;
        }

        try
        {
            return res.get().via.array.ptr[0].as<error_t>();
        }
        catch (const std::exception &ex)
        {
            std::cout << "type decode_1  [" << int(res.get().via.array.ptr[0].type) << "] " << std::endl;
            std::cout << "decode_1 exception " << ex.what() << std::endl;
        }

        return ERROR_COMM;
    }

    int AxisBase::init(const std::string &hostname, uint16_t port)
    {

        tcp::resolver resolver(_io_service);
        try
        {
            _ep = boost::asio::ip::tcp::endpoint(resolver.resolve({ hostname, "" })->endpoint().address(), port);
        }
        catch (const std::exception &ex)
        {
            std::cout << ex.what() << std::endl;
            return -1;
        }

        boost::system::error_code error = reconnect();

        if (error) {
            std::cout << error.message() << std::endl;
            return -1;
        }
        return 0;
    }

    boost::system::error_code AxisBase::reconnect()
    {
        boost::system::error_code error;
        _skt.close();
        _skt.connect(_ep, error);
        return error;
    }

    error_t AxisBase::stop()
    {
        auto msg = msgpack::type::tuple<uint8_t, uint8_t>(CMD_WRITE_STOP, _motor);
        boost::system::error_code ec;

        msgpack::object_handle res = send(msg, ec);

        return decode_1(res, ec);
    }

    error_t AxisBase::set_position(int32_t pos)
    {
        auto msg = msgpack::type::tuple<uint8_t, uint8_t, int32_t>(CMD_WRITE_POSITION, _motor, pos);
        boost::system::error_code ec;

        msgpack::object_handle res = send(msg, ec);

        return decode_1(res, ec);
    }

    error_t AxisBase::go_rel(int32_t steps, uint32_t speed)
    {
        auto msg = msgpack::type::tuple<uint8_t, uint8_t, int32_t, int32_t>(CMD_WRITE_NOW, _motor, steps, speed);
        boost::system::error_code ec;

        msgpack::object_handle res = send(msg, ec);

        return decode_1(res, ec);
    }

    error_t AxisBase::go_abs(int32_t pos, uint32_t speed)
    {
        auto msg = msgpack::type::tuple<uint8_t, uint8_t, int32_t, int32_t>(CMD_WRITE_GOTO, _motor, pos, speed);
        boost::system::error_code ec;

        msgpack::object_handle res = send(msg, ec);

        return decode_1(res, ec);
    }

    error_t AxisAdv::append(int32_t steps, uint32_t speed)
    {
        auto msg = msgpack::type::tuple<uint8_t, uint8_t, int32_t, int32_t>(CMD_WRITE_QUEUE, _motor, steps, speed);
        boost::system::error_code ec;

        msgpack::object_handle res = send(msg, ec);

        return decode_1(res, ec);
    }

    error_t AxisBase::read_hk(int8_t &dir, uint32_t &speed, int32_t &pos, uint8_t &buff_free)
    {
        auto msg = msgpack::type::tuple<uint8_t, uint8_t>(CMD_READ_DATA, _motor);
        boost::system::error_code ec;

        msgpack::object_handle res = send(msg, ec);
        if (ec)
            return ERROR_COMM;

        if (res.get().type != msgpack::type::ARRAY)
            return ERROR_COMM;

        if (res.get().via.array.size != 4)
            return ERROR_COMM;

        try
        {
            ///DBG    std::cout << "type hk"
            ///DBG              << " [" << int(res.get().via.array.ptr[0].type) << "] "
            ///DBG              << " [" << int(res.get().via.array.ptr[1].type) << "] "
            ///DBG              << " [" << int(res.get().via.array.ptr[2].type) << "] "
            ///DBG              << " [" << int(res.get().via.array.ptr[3].type) << "] " << std::endl;

            dir = res.get().via.array.ptr[0].as<int8_t>();
            speed = res.get().via.array.ptr[1].as<uint32_t>();
            pos = res.get().via.array.ptr[2].as<int32_t>();
            buff_free = res.get().via.array.ptr[3].as<uint8_t>();
        }
        catch (const std::exception &ex)
        {
            std::cout << "[read_hk] " << ex.what() << std::endl;
            return ERROR_COMM;
        }

        return ERROR_SUCCESS;
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

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *                               AxisStepper
 *
 *  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    inline float tiks2speed(const uint32_t &speed_tiks)
    {
        if (speed_tiks == 0)
            return 0;
        else
            return float(TICK_S) / float(speed_tiks);
    }

    inline uint32_t speed2ticks(const float &speed)
    {
        uint32_t s = round(TICK_S / speed);
        return s>0?s:1;
    }

    inline float ticks2s(uint32_t ticks)
    {
        return float(ticks * TICK_US) / float(US);
    }

    inline float commands2s(const std::deque<std::pair<int32_t, uint32_t>> &c)
    {
        float s = 0;
        for (const auto &i : c)
        {
            s += ticks2s(i.second) * abs(i.first);
        }
        return s;
    }

    error_t AxisStepper::bulk(move_t &m, bool start_now)
    {
        error_t err = 0;

        std::cout << "++++++++++++++++++++++++++++++++++++++++++++++ " << int(_motor) <<std::endl;
        if (start_now && !m.buffer.empty())
        {
            std::cout.width(10);
            std::cout << "GO_R: " << m.buffer.front().first << " ";
            std::cout.width(10);
            std::cout << m.buffer.front().second << std::endl;
            err = go_rel(m.buffer.front().first, m.buffer.front().second);
            m.buffer.pop_front();
        }
        for (auto &c : m.buffer)
        {
            if (err)
                return err;

            std::cout.width(10);
            std::cout << "GO_A: " << c.first << " ";
            std::cout.width(10);
            std::cout << c.second << std::endl;
            err = append(c.first, c.second);
        }
        std::cout << "++++++++++++++++++++++++++++++++++++++++++++++ " << int(_motor) <<std::endl;
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

    void AxisStepper::compute_acc(float i, float &f, std::deque<uint32_t> &a, size_t max_steps,const float& smooth_factor)
    {
        size_t stop_steps = a.size() + max_steps;
        float v0 = i > STOP_SPEED? i : STOP_SPEED;
        float v = v0;
        float t = ticks2s(speed2ticks(v0));
        std::cout << "compute_acc " << v0 << " ["<<speed2ticks(v0) <<"]  "<< f << " ["<<speed2ticks(f)<<"] ";
        while (v < f && a.size() < stop_steps)
        {
            //std::cout << v << " " << t << std::endl; //DBG
            v = (_acc * t) + v0;
            v = round(v/smooth_factor)*smooth_factor;
            uint32_t ticks = speed2ticks(v);
            v = tiks2speed(ticks);
            t += ticks2s(ticks);
            a.push_back(ticks);
        }
        std::cout << "{" << a.size() << "} vf "<< f << " [" << speed2ticks(f) << "]   SF"<<smooth_factor<<std::endl;
        
        if(!a.empty())
            std::cout << "a.front: "<< tiks2speed(a.front()) << " ["<<a.front()<<"] " << tiks2speed(a.back())<< " ["<< a.back()<<"]   end compute ------" << std::endl;
        f = v;
    }

    template <typename IT>
    void AxisStepper::compress(IT b, IT e, command_t &acc, int sgn)
    {

        for (auto it = b; it != e; it++)
        {
            if (acc.empty() || // accumulator is empty
                acc.back().second != *it || // speeds differ
                acc.back().first * sgn < 0) // direction differs
                acc.emplace_back(sgn, *it);
            else
                acc.back().first += sgn;
        }
    }

    void AxisStepper::append_stop(move_t &m)
    {
        if (abs(m.curr_speed) <= STOP_SPEED)
        { // safe speed to stop
            m.curr_speed = 0;
            return;
        }
        int sgn = m.curr_speed < 0 ? -1 : 1;
        speed_t a;
        float speed = abs(m.curr_speed);
        compute_acc(STOP_SPEED, speed, a, LIMIT_STEPS);
        m.curr_position += a.size() * sgn;
        compress(a.rbegin(), a.rend(), m.buffer, sgn);
        m.curr_speed = 0;
    }

    float AxisStepper::go_to(int32_t pos, float max_speed,const float& smooth_factor,bool then_track, error_t &err)
    {
        err = hk();
        if (err)
            return 0;

        move_t m ={ .curr_speed = _speed_sps, .curr_position = _position };

        append_goto(m, pos, max_speed, STOP_SPEED,smooth_factor);
        float time = commands2s(m.buffer);

        if(then_track){
            std::cout << "********  compute track *******" << std::endl;
            append_goto(m, TRACK_END , _track_speed, STOP_SPEED,smooth_factor);
        }

        err = bulk(m, true);
        _target = m.curr_position;
        return time;
    }

    float AxisStepper::jerk(int32_t steps, error_t &err)
    {
        err = hk();
        if (err)
            return 0;

        move_t m ={ .curr_speed = _speed_sps, .curr_position = _position };

        append_goto(m, m.curr_position + steps, MAX_SPEED, STOP_SPEED,1);
        float time = commands2s(m.buffer); // we return just the jerk time

        if (_position != _target) // we were traveling (or tracking)
            append_goto(m, _target, _speed_sps, STOP_SPEED,1);

        err = bulk(m, true);

        _target = m.curr_position;
        return time;
    }

    void AxisStepper::append_goto(move_t &m, int32_t pos, float max_speed, float final_speed,const float& smooth_factor)
    {
        int32_t steps = pos - m.curr_position;
        if ((m.curr_speed < 0 && steps > 0) ||                 // do we need to reverse movement?
            (m.curr_speed > 0 && steps < 0) ||                 // as above
            abs(steps) < (max_speed * max_speed) / (2 * _acc)) // is the target too close to brake at current speed?
        {                                                      // we need to reverse the movement
            append_stop(m);
            steps = pos - m.curr_position; // abs(steps) should increase and not change sign since we stop in the same movement direction
        }

        std::cout <<"steps to target: " << steps << std::endl;
        speed_t speed_acc, speed_decel;
        int sgn = steps >0? 1 : -1;

        float reached_speed = abs(max_speed);
        compute_acc(abs(m.curr_speed), reached_speed, speed_acc, abs(steps) / 2,smooth_factor);
        int32_t remain_steps = abs(steps) - speed_acc.size();
        compute_acc(final_speed, reached_speed, speed_decel, LIMIT_STEPS,smooth_factor);
        remain_steps -= speed_decel.size();
        std::cout << "remain steps: " << remain_steps << std::endl;

        std::cout << "+++ acc steps: " << speed_acc.size() << "  decel steps: " << speed_decel.size() << std::endl;
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

        std::cout << "--- acc steps: " << speed_acc.size() << "  decel steps: " << speed_decel.size() << std::endl;

        compress(speed_acc.begin(), speed_acc.end(), m.buffer, sgn); // append acceleration
        if (remain_steps > 0)                                        // append constant speed
        {
            uint32_t speed_ticks = speed2ticks(max_speed);
            if (!m.buffer.empty() && m.buffer.back().second == speed_ticks)
                m.buffer.back().first += remain_steps * sgn;
            else if (!speed_decel.empty())
                m.buffer.emplace_back(remain_steps * sgn, speed_ticks);
            else
                m.buffer.emplace_back(remain_steps * sgn, speed_ticks);

        }
        compress(speed_decel.rbegin(), speed_decel.rend(), m.buffer, sgn); // append deceleration

        m.curr_position = pos;
        m.curr_speed = final_speed * sgn;
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *                               AxisPWM
 *
 *  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    inline error_t AxisPWM::hk()
    {
        error_t err = read_hk(_dir, _speed_raw, _position, _buff_free);
        if (err)
            return err;

        _speed_pwm = tiks2speed(_speed_raw) * _dir <= 0 ? -1 : 1;
        return 0;
    }

    int32_t AxisPWM::get_position(error_t &err)
    {
        err = hk();
        return _position;
    }

    error_t AxisPWM::set_position(int32_t pos)
    {
        error_t err = AxisBase::set_position(pos);
        _position = pos;
        return err;
    }

    float AxisPWM::go_to(int32_t pos, error_t &err)
    {
        err = hk();
        if (err)
            return 0;

        int32_t steps = pos - _position;
        if ((_speed_pwm < 0 && steps > 0) || // do we need to reverse movement?
            (_speed_pwm > 0 && steps < 0))   // as above
        {                                    // we need to reverse the movement
            err = stop();
            if (err)
                return 0;
        }
        err = go_abs(pos, 1); // speed does not count in PID PWM
        return steps;         //FIXME: return meaningful value
    }

    float AxisPWM::jerk(int32_t steps, error_t &err)
    {
        err = hk();
        if (err)
            return 0;

        if ((_speed_pwm < 0 && steps > 0) || // do we need to reverse movement?
            (_speed_pwm > 0 && steps < 0))   // as above
        {                                    // we need to reverse the movement
            err = stop();
            if (err)
                return 0;
        }
        err = go_rel(steps, 1); // speed does not count in PID PWM
        return steps;           //FIXME: return meaningful value
    }
} // namespace oavda