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
#define RA_MAX_STEPS RA_TICKS
#define RA_ACC AXIS_ACC
#define TRACK_SPEED (double(RA_TICKS) / 86400)
#define RA_REPOINT_SPEED 10000

#define DEC_MIN_STEPS 0
#define DEC_TICKS 4292250
#define DEC_ZERO_STEPS (DEC_TICKS / 2)
#define DEC_MAX_STEPS DEC_TICKS
#define DEC_ACC AXIS_ACC
#define DEC_REPOINT_SPEED 5000

#define MOTOR_SMOOTH_FACTOR 150

#define OPT_TEENSYD_ADDR (OPT_LOCAL + 300)
#define OPT_TEENSYD_PORT (OPT_LOCAL + 301)

namespace rts2teld
{
	class Oavda810 : public Telescope
	{
	public:
		Oavda810(int in_argc, char **in_argv);
		~Oavda810() {};

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
		/* TODO */ virtual int endPark() {
			return 0;
		}


		/**
 * Called to run tracking. It is up to driver implementation
 * to send updated position to telescope driver.
 *
 * If tracking timer shall not be called agin, call setTracking (false).
 * Run every trackingInterval seconds.
 *
 * @see setTracking
 * @see trackingInterval
 */
		virtual void runTracking();

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
		std::string _teensyd_ip;
		uint16_t _teensyd_port;

		::oavda::AxisStepper _motor_ra;
		::oavda::AxisStepper _motor_dec;

		bool _ra_sideral_tracking;

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

/* DEBUG UTILS */
#include <iomanip>

std::ostream& print_hour(std::ostream& stream, const double& angle) {
	auto w = stream.width();
	auto f = stream.fill();
	auto p = stream.precision();


	int32_t hours = angle / 360 * 24;
	double rem = angle - double(int(angle));

	rem *= 60;
	int32_t min = rem;

	double sec = (rem - min) *60;

	stream.width(2);
	stream.fill('0');
	stream << hours << ":";
	stream << min << ":";
	stream.precision(3);
	stream << sec;

	stream.width(w);
	stream.fill(f);
	stream.precision(p);

	return stream;
}


std::ostream& print_deg(std::ostream& stream, const double& angle) {
	auto w = stream.width();
	auto f = stream.fill();
	auto p = stream.precision();


	int32_t degs = angle;
	double rem = abs(angle - double(int(angle)));

	rem *= 60;
	int32_t min = rem;

	double sec = (rem - min) *60;

	stream.width(2);
	stream.fill('0');
	stream << degs << "d ";
	stream << min << "' ";
	stream.precision(3);
	stream << sec << "\"";

	stream.width(w);
	stream.fill(f);
	stream.precision(p);

	return stream;
}


/* Telescope implementation */

namespace rts2teld
{

	void Oavda810::usage()
	{
		std::cout << "\t" << getAppName() << " --teensyd-ip localhost --teensyd-port 502" << std::endl;
	}

	Oavda810::Oavda810(int in_argc, char **in_argv) : Telescope(in_argc, in_argv, true, true),
		_teensyd_ip("localhost"),
		_teensyd_port(1500),
		_motor_ra(::oavda::AxisStepper::axis::RA,TRACK_SPEED),
		_motor_dec(::oavda::AxisStepper::axis::DEC),
		_ra_sideral_tracking(false)
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
		addOption(OPT_TEENSYD_ADDR, "teensyd-ip", 1, "ip address of the C-RIO controller");
		addOption(OPT_TEENSYD_PORT, "teensyd-port", 1, "port of the C-RIO controller");
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

		ret = _motor_ra.init(_teensyd_ip, _teensyd_port);
		std::cout << "RA MOTOR returned " << int(ret) << std::endl;
		if (ret)
			return ret;

		ret = _motor_dec.init(_teensyd_ip, _teensyd_port);
		std::cout << "Dec MOTOR returned " << int(ret) << std::endl;
		if (ret)
			return ret;

		return 0;
	}

	int Oavda810::processOption(int in_opt)
	{
		std::cout << "++ processOption: " << in_opt << std::endl;
		switch (in_opt)
		{
		case OPT_TEENSYD_ADDR:
			_teensyd_ip = std::string(optarg);
			std::cout << "IP " << _teensyd_ip;
			break;
		case OPT_TEENSYD_PORT:
			_teensyd_port = atoi(optarg);
			break;
		default:
			return Telescope::processOption(in_opt);
		}
		return 0;
	}

	int Oavda810::info()
	{
		//DBG std::cout << "++ info" << std::endl;
		oavda::error_t err = 0;
		int32_t raPos = _motor_ra.get_position(err);
		if (err) {
			std::cout << "RA MOTOR returned " << int(err) << std::endl;
			return -1;
		}

		int32_t decPos = _motor_dec.get_position(err);

		if (err) {
			std::cout << "Dec MOTOR returned " << int(err) << std::endl;
			return -1;
		}

		double t_telRa;
		double t_telDec;
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
		_ra_sideral_tracking = false;
		oavda::error_t err = 0;

		std::cout << "++ startResync" << std::endl;
		if (_motor_ra.stop()) {
			std::cout << "ra stop returned error" << std::endl;
			return -1;
		}
		if (_motor_dec.stop()) {
			std::cout << "dec stop returned error" << std::endl;
			return -1;
		}

		int32_t rac = _motor_ra.get_position(err);
		if (err)
		{
			std::cout << "[GET POS] RA MOTOR returned " << int(err) << std::endl;
			return -1;
		}
		int32_t dc = _motor_dec.get_position(err);
		if (err)
		{
			std::cout << "[GET POS] Dec MOTOR returned " << int(err) << std::endl;
			return -1;
		}
		double utc1, utc2;
		#ifdef RTS2_LIBERFA
		getEraUTC(utc1, utc2);
		#else
		utc1 = ln_get_julian_from_sys();
		utc2 = 0;
		#endif
		struct ln_equ_posn tar;
		struct ln_hrz_posn hrz;
		int ret = calculateTarget(utc1, utc2, &tar, &hrz, rac, dc, true, 360, true);
		if (ret)
			return -1;

		_motor_ra.go_to(rac, RA_REPOINT_SPEED, MOTOR_SMOOTH_FACTOR,false, err);
		if (err)
		{
			std::cout << "[GOTO] RA MOTOR returned " << int(err) << std::endl;
			return -1;
		}

		_motor_dec.go_to(dc, DEC_REPOINT_SPEED, MOTOR_SMOOTH_FACTOR,false, err);
		if (err)
		{
			std::cout << "[GOTO] Dec MOTOR returned " << int(err) << std::endl;
			return -1;
		}

		std::cout << "GOT ra_counts: " << rac << " dec_counts: " << dc << std::endl;
		return 0;
	}

	int Oavda810::isMoving()
	{
		//DBG std::cout << "++ isMoving" << std::endl;
		oavda::error_t err;
		int32_t pos_ra = _motor_ra.get_position(err);
		if (err)
			return -1;
		int32_t pos_dec = _motor_dec.get_position(err);
		if (err)
			return -1;

		int ra_move = 0;
		int dec_move = 0;

		if ((getState() & TEL_MASK_TRACK) == TEL_TRACKING)
		{
			int32_t ac = pos_ra;
			int32_t dc = pos_dec;
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
			int32_t diff_ac = ac - pos_ra;
			int32_t diff_dc = dc - pos_dec;
			std::cout << "diff_ac: " << diff_ac << std::endl; //DBG
			std::cout << "diff_dc: " << diff_dc << std::endl; //DBG
			// if difference in H is greater then moveTolerance..
			if (abs(diff_ac) > haCpd->getValueDouble() * moveTolerance->getValueDouble())
			{
				ra_move = _motor_ra.go_to(ac, RA_REPOINT_SPEED, MOTOR_SMOOTH_FACTOR,false, err)*1000; // s to ms
				if (err)
					return -1;
			}
			else { // RA reached, start tracking
				_motor_ra.go_to(RA_MAX_STEPS, TRACK_SPEED, 1,false, err); // 1 means no smooth factor
				ra_move = -2; //target reached
				if (err)
					return -1;
			}

			// if difference in Dec is greater then moveTolerance...
			if (abs(diff_dc) > decCpd->getValueDouble() * moveTolerance->getValueDouble())
			{
				dec_move = _motor_dec.go_to(dc, DEC_REPOINT_SPEED, MOTOR_SMOOTH_FACTOR,false, err)*1000; // s to ms
				if (err)
					return -1;
			}
			else {
				dec_move = -2; // target reached
			}

			if (dec_move == -2 && ra_move == -2) // target reached
				return -2;
			else {
				if (dec_move >0 && ra_move >0)
					return std::min(dec_move, ra_move); // return min time to check
				else if (dec_move > 0)
					return dec_move;
				else if (ra_move > 0)
					return ra_move;
				else // both 0, extrimely unlikely, return 10 to check position in 10 ms
					return 10;
			}
		}
		else
		{				//FIXME ricalculate target!!!!!
				/*
/////////////////////////////////////////////////////////////////////////////////
meglio ricalcolare il target per arrivare più precisi su repointing lunghi
/////////////////////////////////////////////////////////////////////////////////
				*/
			if (abs(_motor_ra.target()  - pos_ra ) <  haCpd->getValueDouble() * moveTolerance->getValueDouble() &&
				abs(_motor_dec.target() - pos_dec) < decCpd->getValueDouble() * moveTolerance->getValueDouble())
				return -2; // target reached
			else
			{

				float speed_ra = _motor_ra.get_speed(err);
				if (err)
					return -1;

				float speed_dec = _motor_dec.get_speed(err);
				if (err)
					return -1;

				double eta_ra = abs(_motor_ra.target() - pos_ra) * speed_ra;
				double eta_dec = abs(_motor_dec.target() - pos_dec) * speed_dec;

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
		//DBG std::cout << "++ stopMove" << std::endl;
		auto stop_ra = _motor_ra.stop();
		auto stop_dec = _motor_dec.stop();
		if (stop_ra || stop_dec) {
			return -1;
			std::cout << "++ stopMove " << int(stop_ra) << " " << int(stop_dec) << std::endl;
		}
		else return 0;
	}
	/**
	 * Called to run tracking. It is up to driver implementation
	 * to send updated position to telescope driver.
	 *
	 * If tracking timer shall not be called agin, call setTracking (false).
	 * Run every trackingInterval seconds.
	 *
	 * @see setTracking
	 * @see trackingInterval
	 */
	void Oavda810::runTracking() {
		if (_ra_sideral_tracking) {
			return Telescope::runTracking();
		}
		else {
			std::cout << "++ First runTracking" << std::endl;
			//////////////////////////////////////////////////////////////////////////////////////////
			oavda::error_t err;
			int32_t pos_ra = _motor_ra.get_position(err);
			if (err) return Telescope::runTracking();
			int32_t pos_dec = _motor_dec.get_position(err);
			if (err)return Telescope::runTracking();


			int32_t ac = pos_ra;
			int32_t dc = pos_dec;
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
			if (ret)return Telescope::runTracking();

			int32_t diff_ac = ac - pos_ra;
			int32_t diff_dc = dc - pos_dec;
			std::cout << "diff_ac: " << diff_ac << std::endl; //DBG
			std::cout << "diff_dc: " << diff_dc << std::endl; //DBG
			// if difference in H is greater then moveTolerance..
			_motor_ra.go_to(ac, RA_REPOINT_SPEED, 1, true, err); // s to ms
			if (err)return Telescope::runTracking();

			_motor_dec.go_to(dc, DEC_REPOINT_SPEED, 1, false, err); // s to ms
			if (err)return Telescope::runTracking();


			_ra_sideral_tracking = true;
			return Telescope::runTracking();
		}

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
		/*//DBG std::cout << "++ counts2sky:" << std::endl
				  << "ac : " << ac << std::endl
				  << "dc : " << dc << std::endl
				  << "ha : " ;
				  print_hour(std::cout,ha) << " " << ha << std::endl
				  << "ls : " << ls << std::endl
				  << "ra : ";
				  print_hour(std::cout,ra) << " "  << ra << std::endl
				  << "dec: ";
				  print_deg(std::cout,dec) << " "   << dec << std::endl;
				  */
		return 0;
	}

	int Oavda810::setTo(double set_ra, double set_dec)
	{
		std::cout << "++ setTo" << std::endl;
		oavda::error_t err = 0;
		int32_t raPos = _motor_ra.get_position(err);
		if (err)
			return -1;
		int32_t decPos = _motor_dec.get_position(err);
		if (err)
			return -1;

		double t_telRa;
		double t_telDec;

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

		int init()
		{
			_tel.setTelLongLat(7.4781454, 45.7891243);
			_tel.setTelAltitude(1650);
			return 0;
			//TODO init motors
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

		void startRsync(double ra, double dec) {
			_tel.setOrigin(ra, dec);
		}

	private:
		rts2teld::Oavda810 &_tel;
	};

} // namespace rts2teld

//DGB #define OAVDA_TEST 1
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

	::oavda::AxisStepper motor_ra(::oavda::AxisStepper::axis::RA);
	auto i = motor_ra.init("127.0.0.1", 1500);
	std::cout << "init: " << i << std::endl;

	::oavda::error_t err;
	auto time = motor_ra.go_to(atoi(argv[1]), false, err);
	std::cout << "ETA: " << time << std::endl;


	/*
		int32_t zero = 2080171;

		int32_t ac = 0;
		int32_t dc = 0;
		double utc1, utc2;
		struct ln_equ_posn pos;
		struct ln_hrz_posn hrz_out;
		hrz_out.alt = 0;
		hrz_out.az = 0;

		Oavda810 device(argc, argv);
		OavdaTest test(device);
		test.init();


		//Altair
		pos.ra = 285.8463884861111;
		pos.dec = 8.868321194444444;

		// Sirrah
		pos.ra = 0.13985277777777777;
		pos.dec = 29.089499999999997;

		test.getEraUTC(utc1, utc2);
		test.sky2counts(utc1, utc2, &pos, &hrz_out, ac, dc, false, 360, false);

		std::cout << ac << " " << dc << std::endl;
		std::cout << hrz_out.alt << " " << hrz_out.az << std::endl;

		std::cout << "-------" << std::endl;
		ac = atoi(argv[1]);
		dc = atoi(argv[2]);

		double ra = 0;
		double dec = 0;

		test.counts2sky(utc1, utc2, ac, dc, ra, dec);
		std::cout << "-------" << std::endl;
		std::cout << "RA: " << ra << "  Dec: " << dec << std::endl;
		pos.ra = ra;
		pos.dec = dec;
		test.sky2counts(utc1, utc2, &pos, &hrz_out, ac, dc, false, 360, false);
		std::cout << "alt: " << hrz_out.alt << "  az: " << hrz_out.az << std::endl;
		std::cout << ac << " " << dc << std::endl;
		*//*
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