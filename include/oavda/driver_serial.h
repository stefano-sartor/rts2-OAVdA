#ifndef OAVDA_DRIVER_SERIAL
#define OAVDA_DRIVER_SERIAL

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

#include <inttypes.h>
#include <deque>
#include <string>
#include <boost/asio.hpp>
#include <msgpack.hpp>

namespace oavda
{

    typedef uint8_t error_t;

    class AxisBase
    {
    public:
        AxisBase(uint8_t motor) :  _motor(motor), _skt(_io_service){}
        int init(const std::string &ip_addr, uint16_t port);

        error_t stop();

        error_t set_position(int32_t pos);

        int8_t get_dir(error_t &err);
        uint32_t get_speed(error_t &err);
        int32_t get_pos(error_t &err);

        error_t go_rel(int32_t steps, uint32_t speed);
        error_t go_abs(int32_t pos, uint32_t speed);

    protected:
        template <typename T>
        msgpack::object_handle send(T &msg, boost::system::error_code &ec);
        error_t read_hk(int8_t &dir, uint32_t &speed, int32_t &pos, uint8_t& status, uint8_t &buff_free);

        uint8_t _motor;
    private:
        boost::system::error_code reconnect();
        boost::asio::io_service _io_service;
        boost::asio::ip::tcp::socket _skt;
        boost::asio::ip::tcp::endpoint _ep;
    };

    class AxisAdv : public AxisBase
    {
    public:
        AxisAdv(uint8_t motor) : AxisBase(motor) {}

        uint8_t get_buff_free(error_t &err);

        error_t append(int32_t steps, uint32_t speed);
    };

    class AxisStepper : private AxisAdv
    {
    public:
        enum class axis
        {
            RA = MOTOR_RA,
            DEC = MOTOR_DEC
        };
        AxisStepper(axis ax,float track_speed = 0) : AxisAdv(uint8_t(ax)), _acc(20000),_track_speed(track_speed) {}
        int init(const std::string &ip_addr, uint16_t port = 1550) { return AxisAdv::init(ip_addr, port); }
        error_t stop() { return AxisAdv::stop(); }
        float get_speed(error_t &err)
        {
            err = hk();
            return _speed_sps;
        }
        int32_t get_position(error_t &err);
        error_t set_position(int32_t pos);
        float go_to(int32_t pos, float max_speed,const float& smooth_factor,bool then_track, error_t &err);
        float jerk(int32_t steps, error_t &err);
        int32_t target() { return _target; }

    private:
        typedef std::deque<uint32_t> speed_t;
        typedef std::deque<std::pair<int32_t, uint32_t>> command_t;
        typedef struct
        {
            float curr_speed;
            int32_t curr_position;
            command_t buffer;
        } move_t;

        error_t hk();
        void compute_acc(float min_speed, float &max_speed, speed_t &array, size_t max_steps,const float& smooth_factor = 100);
        void append_stop(move_t &m);
        void append_goto(move_t &m, int32_t pos, float max_speed, float final_speed,const float& smooth_factor);
        error_t bulk(move_t &m, bool start_now);

        template <typename IT>
        void compress(IT b, IT e, command_t &acc, int sgn = 1);
        float _acc;  // steps * s^-2
        //bool _track; // usefull to restore tracking speed after correction

        int8_t _dir;
        uint32_t _speed_us;
        int32_t _position;
        uint8_t _status;
        uint8_t _buff_free;
        float _speed_sps;
        float _track_speed;
        int32_t _target;
    };

    class AxisPWM : private AxisBase
    {
    public:
        enum class axis
        {
            DOME = MOTOR_DOME,
            FOCUS = MOTOR_FOCUS
        };
        AxisPWM(axis ax) : AxisBase(uint8_t(ax)) {}
        int init(const std::string &ip_addr, uint16_t port = 1550) { return AxisBase::init(ip_addr, port); }
        error_t stop() { return AxisBase::stop(); }
        float get_speed(error_t &err)
        {
            err = hk();
            return _speed_pwm;
        }
        int32_t get_position(error_t &err);
        error_t set_position(int32_t pos);
        float go_to(int32_t pos, error_t &err);
        float jerk(int32_t steps, error_t &err);

    private:
        error_t hk();

        int8_t _dir;
        uint32_t _speed_raw;
        int32_t _position;
        uint8_t _status;
        uint8_t _buff_free;

        float _speed_pwm;
    };
} // namespace oavda
#endif