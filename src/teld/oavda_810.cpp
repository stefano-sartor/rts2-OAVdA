#include "oavda/modbus.h"
#include "oavda/modbus_address.h"
#include "teld.h"
#include "configuration.h"

#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds

#define MICRO 1000000
#define INIT_SPEED_us 10000
#define FLUSH_ms 100

#define AXIS_ACC 4000000.f

#define RA_MIN_STEPS 0
#define RA_MAX_STEPS 4160342
#define RA_ZERO_STEPS (RA_MAX_STEPS / 2)
#define RA_ACC AXIS_ACC
#define SIDER_SPEED (double(RA_MAX_STEPS) / 86164.1)

#define DEC_MIN_STEPS 0
#define DEC_MAX_STEPS 4292250
#define DEC_ZERO_STEPS (DEC_MAX_STEPS / 2)
#define DEC_ACC AXIS_ACC

#define OPT_CRIO_ADDR (OPT_LOCAL + 300)
#define OPT_CRIO_PORT (OPT_LOCAL + 301)

class AxisDriver : private oavda::Modbus
{
public:
	enum type
	{
		RA,
		DEC
	};
	AxisDriver(type);

	int init(const std::string &ip_addr, uint16_t port = 502);
	int info();

	/**
	 * incemenental move, initial speed=current_speed
	 * @param speed     speed in steps per second > 0
	 * @param target    target in steps
	 * 
	 * @return -1 on failure, -2 on stop needed to reverse motion, otherwise the time in ms to complete the movement
	 */
	int move(double speed, double target);

	/**
	 * stop the axis.
	 * 
	 * @return 0 on succes, -1 on failure
	 */
	int stop();

	/**
	 * enable the axis movement after a stop() call.
	 * 
	 * @return 0 on succes, -1 on failure
	 */
	int enable_axis();

	/**
	 * set zero position to axis.
	 * 
	 * @return 0 on succes, -1 on failure
	 */
	int set_zero();

	/**
	 * the following methods return the parameters of the axis,
	 * you need to call info() first in order to get the most updated values;
	 */
	int32_t position();
	double speed();
	int32_t steps_360();

private:
	uint32_t _min;
	uint32_t _max;
	uint32_t _zero;
	double _acc;

	bulk_coils _status_coils_r;

	bulk_coils _init_coils_rw;
	bulk_coils _reset_pos_coils_rw;
	bulk_coils _reset_steps_360_coils_rw;
	bulk_coils _goto_coils_rw;

	bulk_coils _stop_coils_rw;

	bulk_registers _status_regs_r;
	bulk_registers _steps_360_regs_r;
	bulk_registers _encoder_regs_r;

	bulk_registers _set_pos_motor_rw;
	bulk_registers _move_rw;
	bulk_registers _set_steps_360_rw;
};

namespace rts2teld
{
class Oavda810 : public Telescope
{
public:
	/*WIP*/ Oavda810(int in_argc, char **in_argv);
	/*DONE*/ ~Oavda810(){};

	virtual int idle();

	//virtual void postEvent(rts2core::Event *event); /* serve? per i timer? */

	//virtual int commandAuthorized(rts2core::Connection *conn); /* serve? si, se abbiamo comandi persolanizzati tipo inizializzazione posizione in cielo*/

protected:
	/*DONE*/ virtual void usage();
	/*DONE*/ virtual int processOption(int opt);
	/*DONE*/ virtual int init();
	/*WIP*/ virtual int info();

	/**
	 * Send telescope to requested coordinates. This function does not
	 * have any parameters, as they are various ways how to obtain
	 * telescope coordinates.
	 *
	 * If you want to get raw, J2000 target coordinates, without any offsets, check with
	 *
	 *
	 * @return 0 on success, -1 on error.
	 *
	 * @see originChangedFromLastResync
	 * @see getOrigin()
	 */
	virtual int startResync();

	/**
	 * Check if telescope is moving. Called during telescope
	 * movement to detect if the target destination was reached.
	 *
	 * @return -2 when destination was reached, -1 on failure, >= 0
	 * return value is number of milliseconds for next isMoving
	 * call.
	 */
	virtual int isMoving();
	//	virtual int endMove(); /* forse non serve */

	/**
	 * Stop telescope movement. It is called in two cases. Either when new
	 * target is entered and telescope should stop movement to current target,
	 * or when some failure of telescope is detected and telescope should stop
	 * current movement in order to prevent futher damage to the hardware.
	 *
	 * @return 0 on success, -1 on failure
	 */
	virtual int stopMove();

	/**
	 * Set telescope to match given coordinates
	 *
	 * This function is mainly used to tell the telescope, where it
	 * actually is at the beggining of observation
	 *
	 * @param ra		setting right ascennation
	 * @param dec		setting declination
	 *
	 * @return -1 on error, otherwise 0
	 */
	virtual int setTo(double set_ra, double set_dec);

	/**
	 * Set telescope to park position.
	 *
	 * @return -1 on error, otherwise 0
	 */
	virtual int setToPark();

	/**
	 * Called when park command is issued. Moves telescope to park position. Target
	 * positions are set to NAN after startPark returns 0. If driver needs to retain
	 * set target position (e.g. it is using moveAltAz to move to predefined AltAz pozition),
	 * startPark must reuturn 1.
	 *
	 * @return 0 on success, 1 on success if target value reset is not needed, -1 on failure
	 */
	virtual int startPark();

	/**
	 * Check if telescope is parking. Called during telescope
	 * park to detect if parking position was reached.
	 *
	 * @return -2 when destination was reached, -1 on failure, >= 0
	 * return value is number of milliseconds for next isParking
	 * call.
	 */
	virtual int isParking();

	/**
	 * Called when parking of the telescope is finished. Can do various
	 * important thinks - ussually switch of mount tracking, but can
	 * also switch of some power supply etc..
	 *
	 * @return 0 on success, -1 on failure
	 */
	virtual int endPark() { return 0; }

	/*per ora non ci serve */ //virtual void setDiffTrack(double dra, double ddec);

	virtual int setValue(rts2core::Value *old_value, rts2core::Value *new_value);

	/* BOH */ /*virtual int sky2counts(const double uct1, const double utc2, struct ln_equ_posn *pos,
									 struct ln_hrz_posn *hrz_out, int32_t &ac, int32_t &dc, bool writeValue,
									 double haMargin, bool forceShortest);
									 */

private:
	std::string _crio_ip;
	uint16_t _crio_port;

	AxisDriver _motor_ra;
	AxisDriver _motor_dec;
};

} // namespace rts2teld

/* AxisDriver implementation */
AxisDriver::AxisDriver(AxisDriver::type t)
{
	switch (t)
	{
	case RA:
		_status_coils_r = oavda::ra_status_coil_r;

		_init_coils_rw = oavda::ra_init_coil_rw;
		_reset_pos_coils_rw = oavda::ra_reset_position_coil_rw;
		_reset_steps_360_coils_rw = oavda::ra_reset_steps_360_coil_rw;
		_goto_coils_rw = oavda::ra_goto_coil_rw;

		_stop_coils_rw = oavda::ra_stop_coil_rw;

		_status_regs_r = oavda::ra_status_reg_r;
		_steps_360_regs_r = oavda::ra_steps_360_reg_r;
		_encoder_regs_r = oavda::ra_encoder_reg_r;

		_set_pos_motor_rw = oavda::ra_set_position_motor_reg_rw;
		_move_rw = oavda::ra_move_reg_rw;
		_set_steps_360_rw = oavda::ra_set_steps_360_reg_rw;

		_min = RA_MIN_STEPS;
		_max = RA_MAX_STEPS;
		_zero = RA_ZERO_STEPS;
		_acc = RA_ACC;
		break;
	case DEC:
		_status_coils_r = oavda::dec_status_coil_r;

		_init_coils_rw = oavda::dec_init_coil_rw;
		_reset_pos_coils_rw = oavda::dec_reset_position_coil_rw;
		_reset_steps_360_coils_rw = oavda::dec_reset_steps_360_coil_rw;
		_goto_coils_rw = oavda::dec_goto_coil_rw;

		_stop_coils_rw = oavda::dec_stop_coil_rw;

		_status_regs_r = oavda::dec_status_reg_r;
		_steps_360_regs_r = oavda::dec_steps_360_reg_r;
		_encoder_regs_r = oavda::dec_encoder_reg_r;

		_set_pos_motor_rw = oavda::dec_set_position_motor_reg_rw;
		_move_rw = oavda::dec_move_reg_rw;
		_set_steps_360_rw = oavda::dec_set_steps_360_reg_rw;
		_min = DEC_MIN_STEPS;
		_max = DEC_MAX_STEPS;
		_zero = DEC_ZERO_STEPS;
		_acc = DEC_ACC;
		break;
	default:
		break;
	}
}

int AxisDriver::init(const std::string &ip_addr, uint16_t port)
{
	if (oavda::Modbus::init(ip_addr, port))
		return -1;

	if (set_slave(1))
		return -2;

	return 0;
}

int AxisDriver::info()
{
	bool good = true;

	good = read_bits(_status_coils_r) != _status_coils_r.len ? false : good;

	good = read_bits(_init_coils_rw) != _init_coils_rw.len ? false : good;
	good = read_bits(_reset_pos_coils_rw) != _reset_pos_coils_rw.len ? false : good;
	good = read_bits(_reset_steps_360_coils_rw) != _reset_steps_360_coils_rw.len ? false : good;
	good = read_bits(_goto_coils_rw) != _goto_coils_rw.len ? false : good;

	good = read_bits(_stop_coils_rw) != _stop_coils_rw.len ? false : good;

	good = read_input_registers(_status_regs_r) != _status_regs_r.len ? false : good;
	good = read_input_registers(_steps_360_regs_r) != _steps_360_regs_r.len ? false : good;
	good = read_input_registers(_encoder_regs_r) != _encoder_regs_r.len ? false : good;

	good = read_registers(_set_pos_motor_rw) != _set_pos_motor_rw.len ? false : good;
	good = read_registers(_move_rw) != _move_rw.len ? false : good;
	good = read_registers(_set_steps_360_rw) != _set_steps_360_rw.len ? false : good;

	return good ? 0 : -1;
}

int32_t AxisDriver::position()
{
	return _status_regs_r.at("Position_Motor");
}

double AxisDriver::speed()
{
	int32_t us_speed = _status_regs_r.at("Speed_microSec");
	if (us_speed == 0)
		return 0;

	return us_speed;
	//return 1.f/us_speed * MICRO;
}

int32_t AxisDriver::steps_360()
{
	return _steps_360_regs_r.at("Steps_360");
}

inline double space_from_time(const double &a, const double &t, const double &v0)
{
	return 0.5 * a * t * t + v0 * t;
}

inline double time_from_acc(const double &v0, const double &v1, const double &a)
{
	return (v1 - v0) / a;
}

inline double space_from_acc(const double &v0, const double &v1, const double &a)
{
	return space_from_time(a, time_from_acc(v0, v1, a), v0);
}

inline double speed_usPs2stepsPs(const int32_t &us)
{
	return 1.f / double(us) * MICRO;
}

inline int32_t speed_stepsPs2usPs(const double &sps)
{
	return 1.f / sps * MICRO;
}

int AxisDriver::move(double speed, double target)
{
	if (speed <= 0)
		return -1;
	if (speed > 50000.0)
		return -1; // too fast

	double steps_decel = space_from_acc(speed, 0, -_acc);

	if (read_registers(_move_rw) != _move_rw.len)
		return -1;
	if (read_input_registers(_status_regs_r) != _status_regs_r.len)
		return -1;

	int32_t curr_pos = _status_regs_r.at("Position_Motor");
	int32_t old_sig = curr_pos - _move_rw.at("Target_Position_Motor");
	int32_t new_sig = curr_pos - target;

	if (((old_sig < 0 && new_sig > 0) ||
		(old_sig > 0 && new_sig < 0)) && 
		_status_regs_r.at("Speed_microSec") != 0)
		return -2; // we first need to stop, and there reverse

	int32_t us_per_step_current = _status_regs_r.at("Speed_microSec");
	if (us_per_step_current == 0) // axis stopped, set to 10000
		us_per_step_current = INIT_SPEED_us;

	double curr_speed = speed_usPs2stepsPs(us_per_step_current);

	double steps_acc = std::abs(space_from_acc(speed, curr_speed, _acc));

	int32_t us_per_step_final = speed_stepsPs2usPs(speed);

	_move_rw.at("Speed_init_microSec") = us_per_step_current;
	_move_rw.at("Speed_final_microSec") = us_per_step_final;
	_move_rw.at("Steps_accel") = 500;//steps_acc;
	_move_rw.at("Steps_decel") = 500;//steps_decel;
	_move_rw.at("Target_Position_Motor") = target;
	_goto_coils_rw.at("GOTO_Remote") = 1;

	if (write_registers(_move_rw) != _move_rw.len)
		return -1;
	if (write_bit("GOTO_Remote", _goto_coils_rw) != 1)
		return -1;

	std::cout << _move_rw; //DBG

	return abs(target - curr_pos) * speed * 1000;
}

int AxisDriver::stop()
{
	if (read_input_registers(_status_regs_r) != _status_regs_r.len)
		return -1;

	if (read_registers(_move_rw) != _move_rw.len)
		return -1;

	int32_t us_per_step_current = _status_regs_r.at("Speed_microSec");
	if (us_per_step_current == 0)
		return 0; //already stopped

	double curr_speed = speed_usPs2stepsPs(us_per_step_current);
	double steps_decel = space_from_acc(curr_speed, 0, -_acc);

	_move_rw.at("Speed_init_microSec") = us_per_step_current;
	_move_rw.at("Speed_final_microSec") = INIT_SPEED_us;
	_move_rw.at("Steps_accel") = 0;
	_move_rw.at("Steps_decel") = steps_decel;
	// we leave "Target_Position_Motor" to avoid changing motor direction (since we'll call stop anyways)
	_goto_coils_rw.at("GOTO_Remote") = 1;

	if (write_registers(_move_rw) != _move_rw.len)
		return -1;
	if (write_bit("GOTO_Remote", _goto_coils_rw) != 1)
		return -1;

	int64_t ms_wait = time_from_acc(curr_speed, 0, -_acc) * 1000;
	std::this_thread::sleep_for(std::chrono::milliseconds(ms_wait)); // wait to slow down;

	//FIXME issue stop axis command here
	_stop_coils_rw.at("Enable_STOP") = true;
	_stop_coils_rw.at("STOP") = true;

	if (write_bits(_stop_coils_rw) != _stop_coils_rw.len)
		return -1;

	return 0;
}

int AxisDriver::set_zero()
{
	_set_pos_motor_rw.at("Set_Position_Motor") = _zero;
	_reset_pos_coils_rw.at("Reset_Position_Motor") = 1;
	if (write_registers(_set_pos_motor_rw) != _set_pos_motor_rw.len)
		return -1;
	if (write_bit("Reset_Position_Motor", _reset_pos_coils_rw) != 1)
		return -1;

	//wait FLUSH_ms milliseconds to let the command to be exececuted
	std::this_thread::sleep_for(std::chrono::milliseconds(FLUSH_ms));

	return 0;
}

int AxisDriver::enable_axis()
{
	if (read_bits(_status_coils_r) != _status_coils_r.len)
		return -1;
	if (read_bits(_stop_coils_rw) != _stop_coils_rw.len)
		return -1;

	std::cout << _status_coils_r << std::endl;
	std::cout << _stop_coils_rw << std::endl;

	if (_status_coils_r.at("Status_STOP") == 1)
	{
		_stop_coils_rw.at("Enable_STOP") = 1;
		_stop_coils_rw.at("STOP") = 0;
		if (write_bits(_stop_coils_rw) != _stop_coils_rw.len)
			return -1;
		//wait FLUSH_ms milliseconds to let the command to be exececuted
		std::this_thread::sleep_for(std::chrono::milliseconds(FLUSH_ms));
		_stop_coils_rw.at("Enable_STOP") = 0;
		if (write_bits(_stop_coils_rw) != _stop_coils_rw.len)
			return -1;
	}
	else
	{
		_stop_coils_rw.at("Enable_STOP") = 0;
		_stop_coils_rw.at("STOP") = 0;
		if (write_bits(_stop_coils_rw) != _stop_coils_rw.len)
			return -1;
	}

	std::cout << " STO USCENDO" << std::endl;
	if (read_bits(_status_coils_r) != _status_coils_r.len)
		return -1;
	if (read_bits(_stop_coils_rw) != _stop_coils_rw.len)
		return -1;

	std::cout << _status_coils_r << std::endl;
	std::cout << _stop_coils_rw << std::endl;

	std::this_thread::sleep_for(std::chrono::milliseconds(FLUSH_ms));
	return 0;
}

/* Telescope implementation */

namespace rts2teld
{

void Oavda810::usage()
{
	std::cout << "\t" << getAppName() << " --crio-ip 169.254.181.2 --crio--port 502" << std::endl;
}

Oavda810::Oavda810(int in_argc, char **in_argv) : Telescope(in_argc, in_argv),
												  _crio_ip(""),
												  _crio_port(502),
												  _motor_ra(AxisDriver::RA),
												  _motor_dec(AxisDriver::DEC)
{
	logStream(MESSAGE_DEBUG) << "Oavda810::Oavda810" << sendLog;
	/* VARIABLES HERE */
	addOption(OPT_CRIO_ADDR, "crio-ip", 1, "ip address of the C-RIO controller");
	addOption(OPT_CRIO_PORT, "crio-port", 1, "port of the C-RIO controller");
}

int Oavda810::idle()
{
	return Telescope::idle();
}

int Oavda810::init()
{
	int ret;
	ret = Telescope::init();
	if (ret)
		return ret;

	ret = _motor_ra.init(_crio_ip, _crio_port);
	if (ret)
		return ret;

	ret = _motor_dec.init(_crio_ip, _crio_port);
	if (ret)
		return ret;

	return 0;
}

int Oavda810::processOption(int in_opt)
{
	switch (in_opt)
	{
	case OPT_CRIO_ADDR:
		_crio_ip = std::string(optarg);
		break;
	case OPT_CRIO_PORT:
		_crio_port = atoi(optarg);
		break;
	default:
		return Telescope::processOption(in_opt);
	}
	return 0;
}

int Oavda810::info()
{
	int ret;
	ret = _motor_ra.info();
	if (ret)
		return ret;

	ret = _motor_dec.info();
	if (ret)
		return ret;

	/* TODO aggiorna qui le variabili interne*/
	return 0;
}

int Oavda810::startResync()
{
	return 0;
}

int Oavda810::isMoving()
{
	return 0;
}

int Oavda810::stopMove()
{
	return 0;
}

int Oavda810::setTo(double set_ra, double set_dec)
{
	return 0;
}

int Oavda810::setToPark()
{
	return 0;
}

int Oavda810::startPark()
{
	return 0;
}

int Oavda810::isParking()
{
	return isMoving();
}

/**
 * Set value. This is the function that get called when user want to change some value, either interactively through
 * rts2-mon, XML-RPC or from the script. You can overwrite this function in descendants to allow additional variables
 * beiing overwritten. If variable has flag RTS2_VALUE_WRITABLE, default implemetation returns sucess. If setting variable
 * involves some special commands being send to the device, you most probably want to overwrite setValue, and provides
 * set action for your values in it.
 *
 * Suppose you have variables var1 and var2, which you would like to be settable by user. When user set var1, system will just change
 * value and pick it up next time it will use it. If user set integer value var2, method setVar2 should be called to communicate
 * the change to the underliing hardware. Then your setValueMethod should looks like:
 *
 * @code
 * class MyClass:public MyParent
 * {
 *   ....
 *   protected:
 *       virtual int setValue (Value * old_value, Value *new_value)
 *       {
 *             if (old_value == var1)
 *                   return 0;
 *             if (old_value == var2)
 *                   return setVar2 (new_value->getValueInteger ()) == 0 ? 0 : -2;
 *             return MyParent::setValue (old_value, new_value);
     *       }
 *   ...
 * };
 *
 * @endcode
 *
 * @param  old_value	Old value (pointer), can be directly
 *        accesed with the pointer stored in object.
 * @param new_value	New value.
 *
 * @return 1 when value can be set, but it will take longer
 * time to perform, 0 when value can be se immediately, -1 when
 * value set was queued and -2 on an error.
 */
int Oavda810::setValue(rts2core::Value *old_value, rts2core::Value *new_value)
{
	/*
    if(old_value == _move_CCW){
        //TODO
        return 0;
    }
*/
	return Telescope::setValue(old_value, new_value);
}

} // namespace rts2teld

#define OAVDA_TEST 1
#ifndef OAVDA_TEST

int main(int argc, char **argv)
{
	rts2teld::Oavda810 device(argc, argv);
	return device.run();
}

#else

int main()
{
	double zero = 2080171;
	AxisDriver d(AxisDriver::RA);
	if (d.init("169.254.181.2"))
	{
		std::cout << "error connection" << std::endl;
		return -1;
	}
	d.set_zero();
	d.enable_axis();

	auto t = d.move(50000, zero + 80000);
	std::cout << "ETA " << t << std::endl;

	std::this_thread::sleep_for(std::chrono::seconds(1));
	d.info();
	std::cout << d.steps_360() << "," << d.speed() << "," << d.position() << std::endl;

	std::this_thread::sleep_for(std::chrono::seconds(3));
	/*d.move(15000, zero - 120000);
	std::cout << "ETA " << t << std::endl;
	*/
	/*while (d.stop())
	{
		d.info();
		std::cout << d.steps_360() << "," << d.speed() << "," << d.position() << std::endl;
		//std::cout << "not stopped" << std::endl;
	}
	std::cout << "STOPPED" << std::endl;
	/*
	d.move(20000,zero-80000);
	std::this_thread::sleep_for (std::chrono::seconds(3));
	d.move(50000,zero-120000);
*/
	while (true)
	{
		d.info();
		std::cout << d.steps_360() << "," << d.speed() << "," << d.position() << std::endl;

		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	return 0;
}
#endif