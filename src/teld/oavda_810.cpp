#include "oavda/driver_serial.h"
#include "teld.h"
#include "configuration.h"
#include "libnova_cpp.h"

#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds

#define MICRO 1000000
#define INIT_SPEED_us 10000
#define FLUSH_ms 100

#define AXIS_ACC 4000000.f

#define RA_MIN_STEPS 0
#define RA_TICKS 4160342
#define RA_ZERO_STEPS (RA_TICKS / 2)
#define RA_ACC AXIS_ACC
#define TRACK_SPEED (double(RA_TICKS) / 86400)
#define RA_REPOINT_SPEED 10000

#define DEC_MIN_STEPS 0
#define DEC_TICKS 4292250
#define DEC_ZERO_STEPS (DEC_TICKS / 2)
#define DEC_ACC AXIS_ACC
#define DEC_REPOINT_SPEED 5000

#define OPT_CRIO_ADDR (OPT_LOCAL + 300)
#define OPT_CRIO_PORT (OPT_LOCAL + 301)

namespace oavda
{
class AxisDriver : private Modbus
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
	 * set zero position to axis and counts per 360 degs.
	 * 
	 * @return 0 on succes, -1 on failure
	 */
	int set_zero();

	/**
	 * the following methods return the parameters of the axis,
	 * you need to call info() first in order to get the most updated values;
	 */
	int32_t position();
	int32_t target();
	const int32_t &min() const { return _min; }
	const int32_t &max() const { return _max; }
	double speed();
	int32_t steps_360();

	friend std::ostream &operator<<(std::ostream &stream, const AxisDriver &a)
	{
		stream << a._status_coils_r << "----" << std::endl
			   << a._init_coils_rw << "----" << std::endl
			   << a._reset_pos_coils_rw << "----" << std::endl
			   << a._reset_steps_360_coils_rw << "----" << std::endl
			   << a._goto_coils_rw << "----" << std::endl
			   << a._stop_coils_rw << "----" << std::endl
			   << a._status_regs_r << "----" << std::endl
			   << a._steps_360_regs_r << "----" << std::endl
			   << a._encoder_regs_r << "----" << std::endl
			   << a._set_pos_motor_rw << "----" << std::endl
			   << a._move_rw << "----" << std::endl
			   << a._set_steps_360_rw << "----" << std::endl
			   << "----" << std::endl;
		return stream;
	}

private:
	std::string _ip;
	uint16_t _port;

	type _t;
	int32_t _min;
	int32_t _max;
	int32_t _zero;
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
} // namespace oavda

namespace rts2teld
{
class Oavda810 : public Telescope
{
public:
	Oavda810(int in_argc, char **in_argv);
	~Oavda810(){};

	virtual int idle();

	//virtual void postEvent(rts2core::Event *event); /* serve? per i timer? */

	//virtual int commandAuthorized(rts2core::Connection *conn); /* serve? si, se abbiamo comandi persolanizzati tipo inizializzazione posizione in cielo*/

	friend class OavdaTest;

protected:
	virtual void usage();
	virtual int processOption(int opt);
	virtual int init();
	virtual int info();

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
	/* TODO */ virtual int setToPark();

	/**
	 * Called when park command is issued. Moves telescope to park position. Target
	 * positions are set to NAN after startPark returns 0. If driver needs to retain
	 * set target position (e.g. it is using moveAltAz to move to predefined AltAz pozition),
	 * startPark must reuturn 1.
	 *
	 * @return 0 on success, 1 on success if target value reset is not needed, -1 on failure
	 */
	/* TODO */ virtual int startPark();

	/**
	 * Check if telescope is parking. Called during telescope
	 * park to detect if parking position was reached.
	 *
	 * @return -2 when destination was reached, -1 on failure, >= 0
	 * return value is number of milliseconds for next isParking
	 * call.
	 */
	/* TODO */ virtual int isParking();

	/**
	 * Called when parking of the telescope is finished. Can do various
	 * important thinks - ussually switch of mount tracking, but can
	 * also switch of some power supply etc..
	 *
	 * @return 0 on success, -1 on failure
	 */
	/* TODO */ virtual int endPark() { return 0; }

	/*per ora non ci serve */ //virtual void setDiffTrack(double dra, double ddec);

	/* TODO */ virtual int setValue(rts2core::Value *old_value, rts2core::Value *new_value);

	/**
 	 * Transform sky coordinates to axis coordinates. Implemented in classes
	 * commanding directly the telescope axes in counts.
	 *
	 * @param JD		date for which transformation will be valid
	 * @param pos		target position (sky position, excluding precession, refraction, and corections ...)
	 * @param hrz_out       modelled horizontal coordinates
	 * @param ac		current (input) and target (output) HA axis counts value
	 * @param dc		current (input) and target (output) DEC axis counts value
	 * @param writeValues   when true, RTS2 values will be updated to reflect new target values
	 * @param haMargin      ha value (in degrees), for which mount must be allowed to move
	 * @param forceShortest if true, shortest path will be taken - desired flipping will be ignored
	 */
	virtual int sky2counts(const double uct1, const double utc2, struct ln_equ_posn *pos, struct ln_hrz_posn *hrz_out, int32_t &ac, int32_t &dc, bool writeValue, double haMargin, bool forceShortest);

	/**
	 * Convert counts to RA&Dec coordinates.
	 *
	 * @param JD	  date for which transformation will be valid
	 * @param ac      Alpha counts
	 * @param dc      Delta counts
	 * @param ra      Telescope RA (output)
	 * @param dec     Telescope declination (output)
	 */
	int counts2sky(const double uct1, const double utc2, int32_t ac, int32_t dc, double &ra, double &dec);

private:
	std::string _crio_ip;
	uint16_t _crio_port;

	::oavda::AxisDriver _motor_ra;
	::oavda::AxisDriver _motor_dec;

	double _min_alt = 10;
	double _max_alt = 80;

	/**
	 * motor parameters, in degrees (HA/Dec coordinates of hw-zero positions,
	 *  decZero with inverted sign on south hemisphere).
	 */
	rts2core::ValueDouble *haZero;
	rts2core::ValueDouble *decZero;

	rts2core::ValueDouble *haCpd;
	rts2core::ValueDouble *decCpd;

	/*	rts2core::ValueLong *acMin;
	rts2core::ValueLong *acMax;
	rts2core::ValueLong *dcMin;
	rts2core::ValueLong *dcMax;
*/
	// ticks per revolution
	rts2core::ValueLong *ra_ticks;
	rts2core::ValueLong *dec_ticks;

	rts2core::ValueDouble *moveTolerance;
};

} // namespace rts2teld

namespace oavda
{
/* AxisDriver implementation */

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

AxisDriver::AxisDriver(AxisDriver::type t) : _t(t)
{

	std::cout << "-- Axis[" << (_t == RA ? "RA ]" : "Dec]") << std::endl;
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
		_max = RA_TICKS;
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
		_max = DEC_TICKS;
		_zero = DEC_ZERO_STEPS;
		_acc = DEC_ACC;
		break;
	default:
		break;
	}
}

int AxisDriver::init(const std::string &ip_addr, uint16_t port)
{
	std::cout << "-- init[" << (_t == RA ? "RA ]" : "Dec]") << " " << ip_addr << " " << port << std::endl;
	_ip = ip_addr;
	_port = port;
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

	if (!good)
	{ // probably lost connection, reinit
		std::cout << "!! RE-INIT CONNECTION !!" << std::endl;
		oavda::Modbus::init(_ip, _port);
	}
	return good ? 0 : -1;
}

int32_t AxisDriver::position()
{
	std::cout << "-- position[" << (_t == RA ? "RA ]" : "Dec]") << std::endl;
	return _status_regs_r.at("Position_Motor");
}

int32_t AxisDriver::target()
{
	std::cout << "-- target[" << (_t == RA ? "RA ]" : "Dec]") << std::endl;
	return _move_rw.at("Target_Position_Motor");
}
double AxisDriver::speed()
{
	std::cout << "-- speed[" << (_t == RA ? "RA ]" : "Dec]") << std::endl;
	int32_t us_speed = _status_regs_r.at("Speed_microSec");
	if (us_speed == 0)
		return 0;

	return speed_usPs2stepsPs(us_speed);
}

int32_t AxisDriver::steps_360()
{
	std::cout << "-- steps_360[" << (_t == RA ? "RA ]" : "Dec]") << std::endl;
	return _steps_360_regs_r.at("Steps_360");
}

int AxisDriver::move(double speed, double target)
{
	std::cout << "-- move[" << (_t == RA ? "RA ]" : "Dec]") << " " << speed << " " << target << std::endl;
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
	int32_t us_per_step_current = _status_regs_r.at("Speed_microSec");

	int32_t old_sig = curr_pos - _move_rw.at("Target_Position_Motor");
	int32_t new_sig = curr_pos - target;

	if ((old_sig < 0 && new_sig > 0) ||
		(old_sig > 0 && new_sig < 0))
	{
		if (us_per_step_current < INIT_SPEED_us &&
			us_per_step_current != 0)
			return -2; // we first need to stop, and there reverse
	}

	if (us_per_step_current == 0) // axis stopped, set to 10000
		us_per_step_current = INIT_SPEED_us;

	double curr_speed = speed_usPs2stepsPs(us_per_step_current);

	double steps_acc = std::abs(space_from_acc(speed, curr_speed, _acc));

	int32_t us_per_step_final = speed_stepsPs2usPs(speed);

	_move_rw.at("Speed_init_microSec") = us_per_step_current;
	_move_rw.at("Speed_final_microSec") = us_per_step_final;
	_move_rw.at("Steps_accel") = 500; //steps_acc;
	_move_rw.at("Steps_decel") = 500; //steps_decel;
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
	std::cout << "-- stop[" << (_t == RA ? "RA ]" : "Dec]") << std::endl;
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

	_stop_coils_rw.at("Enable_STOP") = 1;
	_stop_coils_rw.at("STOP") = 1;

	if (write_bits(_stop_coils_rw) != _stop_coils_rw.len)
		return -1;

	return 0;
}

int AxisDriver::set_zero()
{
	std::cout << "-- set_zero[" << (_t == RA ? "RA ]" : "Dec]") << std::endl;
	_set_pos_motor_rw.at("Set_Position_Motor") = _zero;
	_set_steps_360_rw.at("Set_Steps_360") = _max;

	_reset_pos_coils_rw.at("Reset_Position_Motor") = 1;
	_reset_steps_360_coils_rw.at("Reset_Steps_360") = 1;

	if (write_registers(_set_pos_motor_rw) != _set_pos_motor_rw.len)
		return -1;
	if (write_registers(_set_steps_360_rw) != _set_steps_360_rw.len)
		return -1;

	//wait FLUSH_ms milliseconds to let the command to be exececuted
	std::this_thread::sleep_for(std::chrono::milliseconds(FLUSH_ms));

	if (write_bit("Reset_Position_Motor", _reset_pos_coils_rw) != 1)
		return -1;
	if (write_bit("Reset_Steps_360", _reset_steps_360_coils_rw) != 1)
		return -1;

	return 0;
}

int AxisDriver::enable_axis()
{
	std::cout << "-- enable_axis[" << (_t == RA ? "RA ]" : "Dec]") << std::endl;
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

	if (read_bits(_status_coils_r) != _status_coils_r.len)
		return -1;
	if (read_bits(_stop_coils_rw) != _stop_coils_rw.len)
		return -1;

	std::this_thread::sleep_for(std::chrono::milliseconds(FLUSH_ms));
	return 0;
}
} //namespace oavda

/* Telescope implementation */

namespace rts2teld
{

void Oavda810::usage()
{
	std::cout << "\t" << getAppName() << " --crio-ip 169.254.181.2 --crio--port 502" << std::endl;
}

Oavda810::Oavda810(int in_argc, char **in_argv) : Telescope(in_argc, in_argv, true, true),
												  _crio_ip(""),
												  _crio_port(502),
												  _motor_ra(::oavda::AxisDriver::RA),
												  _motor_dec(::oavda::AxisDriver::DEC)
{
	logStream(MESSAGE_DEBUG) << "Oavda810::Oavda810" << sendLog;

	createValue(haZero, "_ha_zero", "HA zero offset", false);
	createValue(decZero, "_dec_zero", "DEC zero offset", false);

	createValue(haCpd, "_ha_cpd", "HA counts per degree", false);
	createValue(decCpd, "_dec_cpd", "DEC counts per degree", false);

	/*
	createValue (acMin, "_ac_min", "HA minimal count value", false);
	createValue (acMax, "_ac_max", "HA maximal count value", false);
	createValue (dcMin, "_dc_min", "DEC minimal count value", false);
	createValue (dcMax, "_dc_max", "DEC maximal count value", false);
*/
	createValue(ra_ticks, "_ra_ticks", "RA ticks per full loop (no effect)", false);
	createValue(dec_ticks, "_dec_ticks", "DEC ticks per full loop (no effect)", false);

	createValue(moveTolerance, "move_tolerance", "[deg] minimal movement distance", false, RTS2_DT_DEGREES | RTS2_VALUE_WRITABLE);

	/* INIT rts2 variables*/
	ra_ticks->setValueLong(RA_TICKS);
	dec_ticks->setValueLong(DEC_TICKS);

	haCpd->setValueDouble(RA_TICKS / 360.0);
	decCpd->setValueDouble(DEC_TICKS / 360.0);

	haZero->setValueDouble(180);
	decZero->setValueDouble(-90);

	moveTolerance->setValueDouble(4.0 / 60.0);

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
	std::cout << "++ init" << std::endl;
	int ret;
	ret = Telescope::init();
	if (ret)
		return ret;

	rts2core::Configuration *config = rts2core::Configuration::instance();
	ret = config->loadFile();
	if (ret)
		return -1;

	setTelLongLat(config->getObserver()->lng, config->getObserver()->lat);
	setTelAltitude(config->getObservatoryAltitude());

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
	std::cout << "++ processOption: " << in_opt << std::endl;
	switch (in_opt)
	{
	case OPT_CRIO_ADDR:
		_crio_ip = std::string(optarg);
		std::cout << "IP " << _crio_ip;
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
	std::cout << "++ info" << std::endl;
	int ret;
	ret = _motor_ra.info();
	if (ret)
		return ret;

	ret = _motor_dec.info();
	if (ret)
		return ret;

	double t_telRa;
	double t_telDec;
	int32_t raPos = _motor_ra.position();
	int32_t decPos = _motor_dec.position();
	double utc1, utc2;
#ifdef RTS2_LIBERFA
	getEraUTC(utc1, utc2);
#else
	utc1 = ln_get_julian_from_sys();
	utc2 = 0;
#endif
	counts2sky(utc1, utc2, raPos, decPos, t_telRa, t_telDec);
	setTelRaDec(t_telRa, t_telDec);
	/* TODO aggiorna qui le variabili interne*/

	return Telescope::info();
}

int Oavda810::startResync()
{
	std::cout << "++ startResync" << std::endl;
	if (_motor_ra.stop())
		return -1;
	if (_motor_dec.stop())
		return -1;

	if (_motor_ra.enable_axis())
		return -1;
	if (_motor_dec.enable_axis())
		return -1;

	if (_motor_ra.info())
		return -1;
	if (_motor_dec.info())
		return -1;

	int32_t rac = _motor_ra.position();
	int32_t dc = _motor_dec.position();
	double utc1, utc2;
#ifdef RTS2_LIBERFA
	getEraUTC(utc1, utc2);
#else
	utc1 = ln_get_julian_from_sys();
	utc2 = 0;
#endif
	struct ln_equ_posn tar;
	struct ln_hrz_posn hrz;
	int ret = calculateTarget(utc1, utc2, &tar, &hrz, rac, dc, true, 360, false);
	if (ret)
		return -1;
	if (_motor_ra.move(RA_REPOINT_SPEED, rac) < 0)
		return -1;
	if (_motor_dec.move(DEC_REPOINT_SPEED, dc) < 0)
		return -1;

	return 0;
}

int Oavda810::isMoving()
{
	std::cout << "++ siMoving" << std::endl;
	if (_motor_ra.info())
		return -1;
	if (_motor_dec.info())
		return -1;

	int ra_move = 0;
	int dec_move = 0;

	if ((getState() & TEL_MASK_TRACK) == TEL_TRACKING)
	{
		int32_t ac = _motor_ra.position();
		int32_t dc = _motor_dec.position();
		double utc1, utc2;
#ifdef RTS2_LIBERFA
		getEraUTC(utc1, utc2);
#else
		utc1 = ln_get_julian_from_sys();
		utc2 = 0;
#endif
		struct ln_equ_posn tar;
		struct ln_hrz_posn hrz;
		int ret = calculateTarget(utc1, utc2, &tar, &hrz, ac, dc, true, 360, false);
		if (ret)
			return -1;
		int32_t diff_ac = ac - _motor_ra.position();
		int32_t diff_dc = dc - _motor_dec.position();
		// if difference in H is greater then moveTolerance..
		if (abs(diff_ac) > haCpd->getValueDouble() * moveTolerance->getValueDouble())
		{
			ra_move = _motor_ra.move(RA_REPOINT_SPEED, ac);
		}
		else if (_motor_ra.target() != _motor_ra.max() && _motor_ra.target() != _motor_ra.min())
		{
			ra_move = _motor_ra.move(TRACK_SPEED, ac); //FIXME diff tracking???
		}
		// if difference in Dec is greater then moveTolerance...
		if (abs(diff_dc) > decCpd->getValueDouble() * moveTolerance->getValueDouble())
		{
			dec_move = _motor_dec.move(DEC_REPOINT_SPEED, dc);
		}
		if (dec_move < 0 || ra_move < 0)
			return -1;
		else
			return std::min(dec_move, ra_move);
	}
	else
	{
		if (_motor_ra.target() == _motor_ra.position() &&
			_motor_dec.target() == _motor_dec.position())
			return -2; // target reached
		else
		{
			double eta_ra = abs(_motor_ra.target() - _motor_ra.position()) * _motor_ra.speed();
			double eta_dec = abs(_motor_dec.target() - _motor_dec.position()) * _motor_dec.speed();

			// return min eta, excluding 0
			eta_ra = eta_ra == 0 ? eta_dec : eta_ra;
			eta_dec = eta_dec == 0 ? eta_ra : eta_dec;
			return std::min(eta_ra, eta_dec);
		}
	}
	return 0; // never reached
}

int Oavda810::stopMove()
{
	std::cout << "++ stopMove" << std::endl;
	auto stop_ra = _motor_ra.stop();
	auto stop_dec = _motor_dec.stop();
	return std::min(stop_ra, stop_dec);
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
	std::cout << "++ setValue" << std::endl;
	/*
    if(old_value == _move_CCW){
        //TODO
        return 0;
    }
*/
	return Telescope::setValue(old_value, new_value);
}

/**
 	 * Transform sky coordinates to axis coordinates. Implemented in classes
	 * commanding directly the telescope axes in counts.
	 *
	 * @param JD		date for which transformation will be valid
	 * @param pos		target position (sky position, excluding precession, refraction, and corections ...)
	 * @param hrz_out       modelled horizontal coordinates
	 * @param ac		current (input) and target (output) HA axis counts value
	 * @param dc		current (input) and target (output) DEC axis counts value
	 * @param writeValues   when true, RTS2 values will be updated to reflect new target values
	 * @param haMargin      ha value (in degrees), for which mount must be allowed to move
	 * @param forceShortest if true, shortest path will be taken - desired flipping will be ignored
	 */
int Oavda810::sky2counts(const double utc1, const double utc2,
						 struct ln_equ_posn *pos, struct ln_hrz_posn *hrz_out,
						 int32_t &ac, int32_t &dc, bool writeValues, double haMargin, bool forceShortest)
{
	std::cout << "++ sky2counts" << std::endl;
	double ls, ha, dec;
	struct ln_equ_posn tar_pos;
	int ret;

	if (std::isnan(pos->ra) || std::isnan(pos->dec))
	{
		logStream(MESSAGE_ERROR) << "sky2counts called with nan ra/dec" << sendLog;
		return -1;
	}
	ls = getLstDeg(utc1, utc2);

	tar_pos.ra = pos->ra;
	tar_pos.dec = pos->dec;

	// apply corrections
	applyCorrections(&tar_pos, utc1, utc2, hrz_out, writeValues);

	if (hrz_out->alt < _min_alt)
	{
		logStream(MESSAGE_ERROR) << "object is below min alt, azimuth is "
								 << hrz_out->az << " and altitude " << hrz_out->alt << " RA/DEC targets was " << LibnovaRaDec(pos)
								 << ", check observatory time and location (long & latitude)"
								 << sendLog;
		return -1;
	}

	if (hrz_out->alt > _max_alt)
	{
		logStream(MESSAGE_ERROR) << "object is above max alt, azimuth is "
								 << hrz_out->az << " and altitude " << hrz_out->alt << " RA/DEC targets was " << LibnovaRaDec(pos)
								 << ", check observatory time and location (long & latitude)"
								 << sendLog;
		return -1;
	}

	// get hour angle
	ha = ln_range_degrees(ls - tar_pos.ra);
	// we do not want -180 : +180 ha
	/*if (ha > 180.0)
		ha -= 360.0;
	*/
	// pretend we are at north hemispehere.. at least for dec
	dec = tar_pos.dec;

	// convert to count values
	int32_t tn_ac = (int32_t)((ha - haZero->getValueDouble()) * haCpd->getValueDouble());
	int32_t tn_dc = (int32_t)((dec - decZero->getValueDouble()) * decCpd->getValueDouble());

	/* APPLY MODEL?*/


    // apply modulo wich works also with negative numbers 
	ac = tn_ac % RA_TICKS;
	dc = tn_dc % DEC_TICKS;

	ac = ac >= 0 ? ac : RA_TICKS - ac;
	dc = dc >= 0 ? dc : DEC_TICKS - dc;

	return 0;
}

int Oavda810::counts2sky(const double utc1, const double utc2,
						 int32_t ac, int32_t dc,
						 double &ra, double &dec)
{
	double ls, ha;

	ls = getLstDeg(utc1, utc2);

	ha = (double)(ac / haCpd->getValueDouble()) + haZero->getValueDouble();
	dec = (double)(dc / decCpd->getValueDouble()) + decZero->getValueDouble();

	ra = ls - ha;

	dec = ln_range_degrees(dec);
	if (dec > 180.0)
		dec -= 360.0;

	ra = ln_range_degrees(ra);
	std::cout << "++ counts2sky:" << std::endl
			  << "ac : " << ac << std::endl
			  << "dc : " << dc << std::endl
			  << "ha : " << ha << std::endl
			  << "ls : " << ls << std::endl
			  << "ra : " << ra << std::endl
			  << "dec: " << dec << std::endl;
	return 0;
}

int Oavda810::setTo(double set_ra, double set_dec)
{
	std::cout << "++ setTo" << std::endl;
	int ret;
	ret = _motor_ra.info();
	if (ret)
		return ret;

	ret = _motor_dec.info();
	if (ret)
		return ret;

	double t_telRa;
	double t_telDec;
	int32_t raPos = _motor_ra.position();
	int32_t decPos = _motor_dec.position();

	double utc1, utc2;
#ifdef RTS2_LIBERFA
	getEraUTC(utc1, utc2);
#else
	utc1 = ln_get_julian_from_sys();
	utc2 = 0;
#endif
	counts2sky(utc1, utc2, raPos, decPos, t_telRa, t_telDec);

	double ra_delta = t_telRa - raPos;
	double dec_delta = t_telDec - decPos;
	haZero->setValueDouble(haZero->getValueDouble() + ra_delta);
	decZero->setValueDouble(decZero->getValueDouble() + dec_delta);
	return 0;
}

class OavdaTest
{
public:
	OavdaTest(rts2teld::Oavda810 &tel) : _tel(tel) {}

	int init(){
		_tel.setTelLongLat(7.4781454, 45.7891243);
		_tel.setTelAltitude(1650);

	}
	int counts2sky(const double utc1, const double utc2,
				   int32_t ac, int32_t dc,
				   double &ra, double &dec)
	{
		return _tel.counts2sky(utc1, utc2, ac, dc, ra, dec);
	}
	int sky2counts(const double utc1, const double utc2,
				   struct ln_equ_posn *pos, struct ln_hrz_posn *hrz_out,
				   int32_t &ac, int32_t &dc, bool writeValues, double haMargin, bool forceShortest)
	{
		return _tel.sky2counts(utc1, utc2, pos, hrz_out, ac, dc, writeValues, haMargin, forceShortest);
	}
	void getEraUTC(double &utc1, double &utc2)
	{
#ifdef RTS2_LIBERFA
		_tel.getEraUTC(utc1, utc2);
#else
		utc1 = ln_get_julian_from_sys();
		utc2 = 0;
#endif
	}

private:
	rts2teld::Oavda810 &_tel;
};

} // namespace rts2teld

//#define OAVDA_TEST 1
#ifndef OAVDA_TEST

int main(int argc, char **argv)
{
	rts2teld::Oavda810 device(argc, argv);
	return device.run();
}

#else
#include <string.h>

using namespace rts2teld;
int main(int argc, char **argv)
{	
	int32_t zero = 2080171;

	int32_t ac = 0;
	int32_t dc = 0;
	double utc1, utc2;
	struct ln_equ_posn pos;
	struct ln_hrz_posn hrz_out;
	hrz_out.alt = 0;
	hrz_out.az  = 0;

	Oavda810 device(argc, argv);
	OavdaTest test(device);
	test.init();

	//Altair
	pos.ra  = 285.8463884861111;
	pos.dec = 8.868321194444444;

	// Sirrah
	pos.ra = 0.13985277777777777;
	pos.dec = 29.089499999999997;

	test.getEraUTC(utc1,utc2);
	test.sky2counts(utc1,utc2,&pos,&hrz_out,ac,dc,false,360,false);

	std::cout << ac << " " << dc << std::endl;
	std::cout << hrz_out.alt << " " << hrz_out.az << std::endl;

	std::cout << "-------" << std::endl;
	ac = atoi(argv[1]);
	dc = atoi(argv[2]);

	double ra = 0;
	double dec = 0;

	test.counts2sky(utc1,utc2,ac,dc,ra,dec);
	std::cout << "-------" << std::endl;
	std::cout <<"RA: " << ra << "  Dec: " << dec << std::endl;
	pos.ra = ra;
	pos.dec = dec;
	test.sky2counts(utc1,utc2,&pos,&hrz_out,ac,dc,false,360,false);
	std::cout <<"alt: " << hrz_out.alt << "  az: " << hrz_out.az << std::endl;
	std::cout << ac << " " << dc << std::endl;
		/*
	std::string ip = "169.254.181.2";
	double zero = 2080171;
	oavda::AxisDriver d(oavda::AxisDriver::RA);
	if (d.init(ip))
	{
		std::cout << "error connection" << std::endl;
		return -1;
	}
	d.stop();
	d.enable_axis();
	d.set_zero();
	d.move(RA_REPOINT_SPEED,zero -10000);
	while (true)
	{
		if (d.info())
		{
			std::cout << "ERROR getting info: " << strerror(errno) << std::endl;
			errno = 0;
			std::this_thread::sleep_for(std::chrono::seconds(1));
			continue;
		}
		std::cout << d << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
*/
		return 0;
}
#endif