#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <errno.h>
#include <string.h>
#include <math.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <libnova/libnova.h>

#include "status.h"
#include "telescope.h"

#include <cstdio>
#include <cstdarg>
#include <opentpl/client.h>
#include <list>

using namespace OpenTPL;

#define LONGITUDE -3.3847222
#define LATITUDE 37.064167
#define ALTITUDE 2896

class ErrorTime
{
  time_t etime;
  int error;
public:
    ErrorTime (int in_error);
  int clean (time_t now);
  int isError (int in_error);
};

ErrorTime::ErrorTime (int in_error)
{
  error = in_error;
  time (&etime);
}

int
ErrorTime::clean (time_t now)
{
  if (now - etime > 600)
    return 1;
  return 0;
}

int
ErrorTime::isError (int in_error)
{
  if (error = in_error)
    {
      time (&etime);
      return 1;
    }
  return 0;
}

class Rts2DevTelescopeIr:public Rts2DevTelescope
{
private:
  std::string * ir_ip;
  int ir_port;
  Client *tplc;
  int timeout;
  double cover;
  enum
  { OPENED, OPENING, CLOSING, CLOSED } cover_state;

  struct ln_equ_posn target;

    template < typename T > int tpl_get (const char *name, T & val,
					 int *status);
    template < typename T > int tpl_set (const char *name, T val,
					 int *status);
    template < typename T > int tpl_setw (const char *name, T val,
					  int *status);

  double get_loc_sid_time ();

  virtual int coverClose ();
  virtual int coverOpen ();

  void addError (int in_error);

  void checkErrors ();
  void checkCover ();
  void checkPower ();

    std::list < ErrorTime * >errorcodes;

public:
    Rts2DevTelescopeIr (int argc, char **argv);
    virtual ~ Rts2DevTelescopeIr (void);
  virtual int processOption (int in_opt);
  virtual int init ();
  virtual int idle ();
  virtual int ready ();
  virtual int baseInfo ();
  virtual int info ();
  virtual int startMove (double tar_ra, double tar_dec);
  virtual int isMoving ();
  virtual int endMove ();
  virtual int startPark ();
  virtual int isParking ();
  virtual int endPark ();
  virtual int stopMove ();
  virtual int changeMasterState (int new_state);
  virtual int resetMount (resetStates reset_mount);
};

template < typename T > int
Rts2DevTelescopeIr::tpl_get (const char *name, T & val, int *status)
{
  int cstatus;

  if (!*status)
    {
      Request & r = tplc->Get (name, false);
      cstatus = r.Wait (USEC_SEC);

      if (cstatus != TPLC_OK)
	{
	  syslog (LOG_ERR, "Rts2DevTelescopeIr::tpl_get error %i\n", cstatus);
	  *status = 1;
	}

      if (!*status)
	{
	  RequestAnswer & answr = r.GetAnswer ();

	  if (answr.begin ()->second.first == TPL_OK)
	    val = (T) answr.begin ()->second.second;
	  else
	    *status = 2;
	}
      r.Dispose ();
    }
  return *status;
}

template < typename T > int
Rts2DevTelescopeIr::tpl_set (const char *name, T val, int *status)
{
  int cstatus;

  if (!*status)
    {
      tplc->Set (name, Value (val), true);	// change to set...?
    }
  return *status;
}

template < typename T > int
Rts2DevTelescopeIr::tpl_setw (const char *name, T val, int *status)
{
  int cstatus;

  if (!*status)
    {
      Request & r = tplc->Set (name, Value (val), false);	// change to set...?
      cstatus = r.Wait ();

      if (cstatus != TPLC_OK)
	{
	  syslog (LOG_ERR, "Rts2DevTelescopeIr::tpl_setw error %i\n",
		  cstatus);
	  *status = 1;
	}
      r.Dispose ();
    }
  return *status;
}

int
Rts2DevTelescopeIr::coverClose ()
{
  int status = 0;
  if (cover_state == CLOSED)
    return 0;
  status = tpl_set ("COVER.TARGETPOS", 0, &status);
  status = tpl_set ("COVER.POWER", 1, &status);
  cover_state = CLOSING;
  syslog (LOG_DEBUG, "Rts2DevTelescopeIr::coverClose status: %i", status);
  return status;
}

int
Rts2DevTelescopeIr::coverOpen ()
{
  int status = 0;
  if (cover_state == OPENED)
    return 0;
  status = tpl_set ("COVER.TARGETPOS", 1, &status);
  status = tpl_set ("COVER.POWER", 1, &status);
  cover_state = OPENING;
  syslog (LOG_DEBUG, "Rts2DevTelescopeIr::coverOpen status: %i", status);
  return status;
}

Rts2DevTelescopeIr::Rts2DevTelescopeIr (int argc, char **argv):Rts2DevTelescope (argc,
		  argv)
{
  ir_ip = NULL;
  ir_port = 0;
  tplc = NULL;
  addOption ('I', "ir_ip", 1, "IR TCP/IP address");
  addOption ('P', "ir_port", 1, "IR TCP/IP port number");

  strcpy (telType, "BOOTES_IR");
  strcpy (telSerialNumber, "001");

  cover = 0;
  cover_state = CLOSED;
}

Rts2DevTelescopeIr::~Rts2DevTelescopeIr (void)
{

}

int
Rts2DevTelescopeIr::processOption (int in_opt)
{
  switch (in_opt)
    {
    case 'I':
      ir_ip = new std::string (optarg);
      break;
    case 'P':
      ir_port = atoi (optarg);
      break;
    default:
      return Rts2DevTelescope::processOption (in_opt);
    }
  return 0;
}

int
Rts2DevTelescopeIr::init ()
{
  int ret;
  int status = 0;
  ret = Rts2DevTelescope::init ();
  if (ret)
    return ret;

  if (!ir_ip || !ir_port)
    {
      fprintf (stderr, "Invalid port or IP address of mount controller PC\n");
      return -1;
    }

  tplc = new Client (*ir_ip, ir_port);

  syslog (LOG_DEBUG,
	  "Status: ConnID = %i, connected: %s, authenticated %s, Welcome Message = %s",
	  tplc->ConnID (), (tplc->IsConnected ()? "yes" : "no"),
	  (tplc->IsAuth ()? "yes" : "no"), tplc->WelcomeMessage ().c_str ());

  // are we connected ?
  if (!tplc->IsAuth () || !tplc->IsConnected ())
    {
      syslog (LOG_ERR, "Connection to server failed");
      return -1;
    }
  tpl_get ("COVER.REALPOS", cover, &status);
  if (cover == 0)
    cover_state = CLOSED;
  else if (cover == 1)
    cover_state = OPENED;
  else
    cover_state = CLOSING;
  return 0;
}

void
Rts2DevTelescopeIr::addError (int in_error)
{
  char *txt;
  std::string desc;
  std::list < ErrorTime * >::iterator errIter;
  ErrorTime *errt;
  int status;
  for (errIter = errorcodes.begin (); errIter != errorcodes.end (); errIter++)
    {
      errt = *errIter;
      if (errt->isError (in_error))
	return;
    }
  // new error
  asprintf (&txt, "CABINET.STATUS.TEXT[%i]", in_error & 0x00ffffff);
  status = 0;
  status = tpl_get (txt, desc, &status);
  if (status)
    syslog (LOG_ERR,
	    "Rts2DevTelescopeIr::checkErrors Telescope getting error: %i sev:%x err:%x",
	    in_error, in_error & 0xff000000, in_error & 0x00ffffff);
  else
    syslog (LOG_DEBUG,
	    "Rts2DevTelescopeIr::checkErrors Telescope sev: %x err: %x desc: %s",
	    in_error & 0xff000000, in_error & 0x00ffffff, desc.c_str ());
  free (txt);
  errt = new ErrorTime (in_error);
  errorcodes.push_back (errt);
}

void
Rts2DevTelescopeIr::checkErrors ()
{
  int status = 0;
  int listCount;
  time_t now;
  std::list < ErrorTime * >::iterator errIter;
  ErrorTime *errt;

  status = tpl_get ("CABINET.STATUS.LIST_COUNT", listCount, &status);
  if (status == 0 && listCount > 0)
    {
      // print errors to log & ends..
      string::size_type pos = 1;
      string::size_type lastpos = 1;
      std::string list;
      status = tpl_get ("CABINET.STATUS.LIST", list, &status);
      if (status == 0)
	syslog (LOG_ERR,
		"Rts2DevTelescopeIr::checkErrors Telescope errors: %s",
		list.c_str ());
      // decode errors
      while (true)
	{
	  int errn;
	  if (lastpos > list.size ())
	    break;
	  pos = list.find (',', lastpos);
	  if (pos == string::npos)
	    pos = list.find ('"', lastpos);
	  if (pos == string::npos)
	    break;		// we reach string end..
	  errn = atoi (list.substr (lastpos, pos).c_str ());
	  addError (errn);
	  lastpos = pos + 1;
	}
      tpl_set ("CABINET.STATUS.CLEAR", 1, &status);
    }
  // clean from list old errors..
  time (&now);
  for (errIter = errorcodes.begin (); errIter != errorcodes.end ();)
    {
      errt = *errIter;
      if (errt->clean (now))
	{
	  errIter = errorcodes.erase (errIter);
	  delete errt;
	}
      else
	{
	  errIter++;
	}
    }
}

void
Rts2DevTelescopeIr::checkCover ()
{
  int status = 0;
  switch (cover_state)
    {
    case OPENING:
      tpl_get ("COVER.REALPOS", cover, &status);
      if (cover == 1.0)
	{
	  tpl_set ("COVER.POWER", 0, &status);
	  syslog (LOG_DEBUG, "Rts2DevTelescopeIr::checkCover opened %i",
		  status);
	  cover_state = OPENED;
	  break;
	}
      setTimeout (USEC_SEC);
      break;
    case CLOSING:
      tpl_get ("COVER.REALPOS", cover, &status);
      if (cover == 0.0)
	{
	  tpl_set ("COVER.POWER", 0, &status);
	  syslog (LOG_DEBUG, "Rts2DevTelescopeIr::checkCover closed %i",
		  status);
	  cover_state = CLOSED;
	  break;
	}
      setTimeout (USEC_SEC);
      break;
    }
}

void
Rts2DevTelescopeIr::checkPower ()
{
  int status;
  double power_state;
  double referenced;
  status = 0;
  status = tpl_get ("CABINET.POWER_STATE", power_state, &status);
  if (status)
    {
      syslog (LOG_ERR, "Rts2DevTelescopeIr::checkPower tpl_ret: %i", status);
      return;
    }
  if (power_state == 0)
    {
      status = tpl_setw ("CABINET.POWER", 1, &status);
      status = tpl_get ("CABINET.POWER_STATE", power_state, &status);
      if (status)
	{
	  syslog (LOG_ERR,
		  "Rts2DevTelescopeIr::checkPower set power ot 1 ret: %i",
		  status);
	  return;
	}
      while (power_state == 0.5)
	{
	  syslog (LOG_DEBUG,
		  "Rts2DevTelescopeIr::checkPower waiting for power up");
	  sleep (5);
	  status = tpl_get ("CABINET.POWER_STATE", power_state, &status);
	  if (status)
	    {
	      syslog (LOG_ERR,
		      "Rts2DevTelescopeIr::checkPower power_state ret: %i",
		      status);
	      return;
	    }
	}
    }
  while (true)
    {
      status = tpl_get ("CABINET.REFERENCED", referenced, &status);
      if (status)
	{
	  syslog (LOG_ERR,
		  "Rts2DevTelescopeIr::checkPower get referenced: %i",
		  status);
	  return;
	}
      if (referenced == 1)
	break;
      syslog (LOG_DEBUG, "Rts2DevTelescopeIr::checkPower referenced: %f",
	      referenced);
      if (referenced == 0)
	{
	  int reinit = (nextReset == RESET_COLD_START ? 1 : 0);
	  status = tpl_set ("CABINET.REINIT", reinit, &status);
	  if (status)
	    {
	      syslog (LOG_ERR, "Rts2DevTelescopeIr::checkPower reinit: %i",
		      status);
	      return;
	    }
	}
      sleep (1);
    }
}

int
Rts2DevTelescopeIr::idle ()
{
  // check for power..
  checkPower ();
  // check for errors..
  checkErrors ();
  checkCover ();
  return Rts2DevTelescope::idle ();
}

int
Rts2DevTelescopeIr::ready ()
{
  return !tplc->IsConnected ();
}

int
Rts2DevTelescopeIr::baseInfo ()
{
  std::string serial;
  int status = 0;

  telLongtitude = LONGITUDE;
  telLatitude = LATITUDE;
  telAltitude = ALTITUDE;
  tpl_get ("CABINET.SETUP.HW_ID", serial, &status);
  strncpy (telSerialNumber, serial.c_str (), 64);
  return 0;
}

double
Rts2DevTelescopeIr::get_loc_sid_time ()
{
  return ln_get_apparent_sidereal_time (ln_get_julian_from_sys ()) +
    telLongtitude / 15.0;
}

int
Rts2DevTelescopeIr::info ()
{
  double zd, az;
  int status = 0;
  int track = 0;

  if (!(tplc->IsAuth () && tplc->IsConnected ()))
    return -1;

  status = tpl_get ("POINTING.CURRENT.RA", telRa, &status);
  if (status)
    return -1;
  telRa *= 15.0;
  status = tpl_get ("POINTING.CURRENT.DEC", telDec, &status);
  if (status)
    return -1;
  status = tpl_get ("POINTING.TRACK", track, &status);
  if (status)
    return -1;

  telSiderealTime = get_loc_sid_time ();
  telLocalTime = 0;

  status = tpl_get ("ZD.REALPOS", zd, &status);
  status = tpl_get ("AZ.REALPOS", az, &status);

  if (!track)
    {
      // telRA, telDec are invalid: compute them from ZD/AZ
      struct ln_hrz_posn hrz;
      struct ln_lnlat_posn observer;
      struct ln_equ_posn curr;
      hrz.az = az;
      hrz.alt = 90 - zd;
      observer.lng = telLongtitude;
      observer.lat = telLatitude;
      ln_get_equ_from_hrz (&hrz, &observer, ln_get_julian_from_sys (), &curr);
      telRa = curr.ra;
      telDec = curr.dec;
      telFlip = 0;
    }
  else
    {
      telFlip = 0;
    }

  if (status)
    return -1;


  telAxis[0] = az;
  telAxis[1] = zd;

  return 0;
}

int
Rts2DevTelescopeIr::startMove (double ra, double dec)
{
  int status = 0;

  target.ra = ra;
  target.dec = dec;

  status = tpl_setw ("POINTING.TARGET.RA", ra / 15.0, &status);
  status = tpl_setw ("POINTING.TARGET.DEC", dec, &status);
  status = tpl_set ("POINTING.TRACK", 4, &status);
  syslog (LOG_DEBUG, "Rts2DevTelescopeIr::startMove TRACK status: %i",
	  status);

  if (status)
    return -1;

  timeout = 0;
  return 0;
}

int
Rts2DevTelescopeIr::isMoving ()
{
  double diffRa, diffDec;
  ln_equ_posn curr;

  timeout++;

  // finish due to error
  if (timeout > 20000)
    {
      return -1;
    }

  info ();

  curr.ra = telRa;
  curr.dec = telDec;

  if (ln_get_angular_separation (&target, &curr) > 2)
    {
      return 1000;
    }
  // bridge 20 sec timeout
  return -2;
}

int
Rts2DevTelescopeIr::endMove ()
{
  return 0;
}

int
Rts2DevTelescopeIr::startPark ()
{
  int status = 0;
  // Park to south+zenith
  status = tpl_set ("POINTING.TRACK", 0, &status);
  syslog (LOG_DEBUG, "Rts2DevTelescopeIr::startPark tracking status: %i",
	  status);
  usleep (100000);		//0.1s
  status = tpl_setw ("AZ.TARGETPOS", 0, &status);
  syslog (LOG_DEBUG, "Rts2DevTelescopeIr::startPark AZ.TARGETPOS status: %i",
	  status);
  status = tpl_setw ("ZD.TARGETPOS", 0, &status);
  syslog (LOG_DEBUG, "Rts2DevTelescopeIr::startPark ZD.TARGETPOS status: %i",
	  status);
  if (status)
    return -1;
  return 0;
}

int
Rts2DevTelescopeIr::isParking ()
{
  return -2;
}

int
Rts2DevTelescopeIr::endPark ()
{
  syslog (LOG_DEBUG, "Rts2DevTelescopeIr::endPark");
  return 0;
}

int
Rts2DevTelescopeIr::stopMove ()
{
  int status;
  double zd;
  info ();
  // ZD check..
  status = tpl_get ("ZD.CURRPOS", zd, &status);
  if (status)
    {
      syslog (LOG_DEBUG, "Rts2DevTelescopeIr::stopMove cannot get ZD! (%i)",
	      status);
      return -1;
    }
  if (fabs (zd) < 1)
    {
      syslog (LOG_DEBUG, "Rts2DevTelescopeIr::stopMove suspicious ZD.. %f",
	      zd);
      status = tpl_set ("POINTING.TRACK", 0, &status);
      if (status)
	{
	  syslog (LOG_DEBUG,
		  "Rts2DevTelescopeIr::stopMove cannot set track: %i",
		  status);
	  return -1;
	}
      return 0;
    }
  startMove (telRa, telDec);
  return 0;
}

int
Rts2DevTelescopeIr::changeMasterState (int new_state)
{
  switch (new_state)
    {
    case SERVERD_DUSK:
    case SERVERD_NIGHT:
    case SERVERD_DAWN:
      return coverOpen ();
    default:
      return coverClose ();
    }
  return 0;
}

int
Rts2DevTelescopeIr::resetMount (resetStates reset_mount)
{
  int status = 0;
  int power = 0;
  double power_state;
  status = tpl_setw ("CABINET.POWER", power, &status);
  if (status)
    {
      syslog (LOG_ERR, "Rts2DevTelescopeIr::resetMount powering off: %i",
	      status);
      return -1;
    }
  while (true)
    {
      syslog (LOG_DEBUG,
	      "Rts2DevTelescopeIr::resetMount waiting for power down");
      sleep (5);
      status = tpl_get ("CABINET.POWER_STATE", power_state, &status);
      if (status)
	{
	  syslog (LOG_ERR,
		  "Rts2DevTelescopeIr::resetMount power_state ret: %i",
		  status);
	  return -1;
	}
      if (power_state == 0 || power_state == -1)
	{
	  syslog (LOG_DEBUG,
		  "Rts2DevTelescopeIr::resetMount final power_state : %f",
		  power_state);
	  break;
	}
    }
  return Rts2DevTelescope::resetMount (reset_mount);
}

int
main (int argc, char **argv)
{
  Rts2DevTelescopeIr *device = new Rts2DevTelescopeIr (argc, argv);

  int ret;
  ret = device->init ();
  if (ret)
    {
      fprintf (stderr, "Cannot initialize telescope bridge - exiting!\n");
      exit (0);
    }
  device->run ();
  delete device;
}
