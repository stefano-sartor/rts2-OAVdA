#include "teld.h"
#include "configuration.h"
#include "connection/tcsng.h"

using namespace rts2teld;

/*moveState: 0=>Not moving no move called, 
                1=>move called but not moving, 2=>moving*/
#define TCSNG_NO_MOVE_CALLED   0
#define TCSNG_MOVE_CALLED      1
#define TCSNG_MOVING           2

/**
 * TCS telescope class to interact with TCSng control systems.
 *
 * author Scott Swindell
 */
class TCSNG:public Telescope
{
	public:
		TCSNG (int argc, char **argv);
		virtual ~TCSNG ();

	protected:
		virtual int processOption (int in_opt);

		virtual int initHardware ();

		virtual int info ();

		virtual int startResync ();

		virtual int startMoveFixed (double tar_az, double tar_alt);

		virtual int stopMove ();

		virtual int startPark ();

		virtual int endPark ()
		{
			return 0;
		}

		virtual int isMoving ();
			
		virtual int isMovingFixed ()
		{
			return isMoving ();
		}

		virtual int isParking ()
		{
			return isMoving ();
		}

		virtual int setValue (rts2core::Value *oldValue, rts2core::Value *newValue);

		virtual double estimateTargetTime ()
		{
			return getTargetDistance () * 2.0;
		}

	private:
		rts2core::ConnTCSNG *ngconn;

		HostString *host;
		const char *cfgFile;

		bool nillMove;
		rts2core::ValueBool *domeAuto;
		rts2core::ValueDouble *domeAz;
		rts2core::ValueSelection *tcsngmoveState;

		rts2core::ValueInteger *reqcount;
};

using namespace rts2teld;

const char *deg2hours (double h)
{
	static char rbuf[200];
	struct ln_hms hms;
	ln_deg_to_hms (h, &hms);

	snprintf (rbuf, 200, "%02d:%02d:%02.2f", hms.hours, hms.minutes, hms.seconds);
	return rbuf;
}

const char *deg2dec (double d)
{
	static char rbuf[200];
	struct ln_dms dms;
	ln_deg_to_dms (d, &dms);

	snprintf (rbuf, 200, "%c%02d:%02d:%02.2f", dms.neg ? '-':'+', dms.degrees, dms.minutes, dms.seconds);
	return rbuf;
}

TCSNG::TCSNG (int argc, char **argv):Telescope (argc,argv)
{
	createValue (domeAuto, "dome_auto", "dome follows the telescope", false, RTS2_VALUE_WRITABLE);
	domeAuto->setValueBool (false);

	createValue (domeAz, "dome_az", "dome azimuth", false);
	
	/*RTS2 moveState is an rts2 data type that is alwasy 
	set equal to the tcsng.moveState member 
	this way moveState variable can be seen in rts2-mon*/
	createValue (tcsngmoveState, "tcsng_move_state", "TCSNG move state", false);
	tcsngmoveState->addSelVal ("NO MOVE");
	tcsngmoveState->addSelVal ("MOVE CALLED");
	tcsngmoveState->addSelVal ("MOVING");
	tcsngmoveState->setValueInteger (0);

	createValue (reqcount, "tcsng_req_count", "TCSNG request counter", false);
	reqcount->setValueInteger (TCSNG_NO_MOVE_CALLED);

	ngconn = NULL;
	host = NULL;
	cfgFile = NULL;

	addOption (OPT_CONFIG, "config", 1, "configuration file");
	addOption ('t', NULL, 1, "TCS NG hostname[:port]");
}

TCSNG::~TCSNG ()
{
	delete ngconn;
}

int TCSNG::processOption (int in_opt)
{
	switch (in_opt)
	{
		case OPT_CONFIG:
			cfgFile = optarg;
			break;
		case 't':
			host = new HostString (optarg, "5750");
			break;
		default:
			return Telescope::processOption (in_opt);
	}
	return 0;
}

int TCSNG::initHardware ()
{
	if (host == NULL)
	{
		logStream (MESSAGE_ERROR) << "You must specify TCS NG server hostname (with -t option)." << sendLog;
		return -1;
	}

	ngconn = new rts2core::ConnTCSNG (this, host->getHostname (), host->getPort (), "TCSNG", "TCS");
	ngconn->setDebug (getDebug ());

	rts2core::Configuration *config;
	config = rts2core::Configuration::instance ();
	config->loadFile (cfgFile);
	telLatitude->setValueDouble (config->getObserver ()->lat);
	telLongitude->setValueDouble (config->getObserver ()->lng);
	telAltitude->setValueDouble (config->getObservatoryAltitude ());
			
	return Telescope::initHardware ();
}

int TCSNG::info ()
{
	setTelRaDec (ngconn->getSexadecimalHours ("RA"), ngconn->getSexadecimalAngle ("DEC"));
	double nglst = ngconn->getSexadecimalTime ("ST");

	const char * domest = ngconn->request ("DOME");
	double del,telaz,az;
	char *mod, *init, *home;
	mod = init = home = NULL;
	size_t slen = sscanf (domest, "%lf %ms %ms %lf %lf %ms", &del, &mod, &init, &telaz, &az, &home);
	if (slen == 6)
	{
		domeAuto->setValueBool (strcmp (mod, "AUTO"));
		domeAz->setValueDouble (az);
	}
	if (mod)
		free (mod);
	if (init)
		free (init);
	if (home)
		free (home);

	reqcount->setValueInteger (ngconn->getReqCount ());

	return Telescope::infoLST (nglst);
}

int TCSNG::startResync ()
{
	struct ln_equ_posn tar;
	getTarget (&tar);

	char cmd[200];
	snprintf (cmd, 200, "NEXTPOS %s %s 2000 0 0", deg2hours (tar.ra), deg2dec (tar.dec));
	ngconn->command (cmd);
	ngconn->command ("MOVNEXT");

	tcsngmoveState->setValueInteger (TCSNG_MOVE_CALLED);
  	return 0;
}

int TCSNG::startMoveFixed (double tar_az, double tar_alt)
{
	char cmd[200];
	snprintf (cmd, 200, "ELAZ %lf %lf", tar_alt, tar_az);

	ngconn->command (cmd);
	return 0;
}

int TCSNG::stopMove ()
{
	ngconn->command ("CANCEL");
	tcsngmoveState->setValueInteger (TCSNG_NO_MOVE_CALLED);
	return 0;
}

int TCSNG::startPark ()
{
	ngconn->command ("MOVSTOW");
	tcsngmoveState->setValueInteger (TCSNG_MOVE_CALLED);
	return 0;
}

int TCSNG::isMoving ()
{
	int mot = atoi (ngconn->request ("MOTION"));
	switch (tcsngmoveState->getValueInteger ())
	{
		case TCSNG_MOVE_CALLED:
			if ((mot & TCSNG_RA_AZ) || (mot & TCSNG_DEC_AL) || (mot & TCSNG_DOME))
				tcsngmoveState->setValueInteger (TCSNG_MOVING);
			return USEC_SEC / 100;
			break;
		case TCSNG_MOVING:
			if ((mot & TCSNG_RA_AZ) || (mot & TCSNG_DEC_AL) || (mot & TCSNG_DOME))
				return USEC_SEC / 100;
			tcsngmoveState->setValueInteger (TCSNG_NO_MOVE_CALLED);
			return -2;
			break;
	}
	return -1;
}

int TCSNG::setValue (rts2core::Value *oldValue, rts2core::Value *newValue)
{
	if (oldValue == domeAuto)
	{
		if (((rts2core::ValueBool *) newValue)->getValueBool ())
			ngconn->command ("DOME AUTO ON");
		else
			ngconn->command ("DOME AUTO OFF");
		return 0;
	}
	return Telescope::setValue (oldValue, newValue);
}

int main (int argc, char **argv)
{
	TCSNG device (argc, argv);
	return device.run ();
}