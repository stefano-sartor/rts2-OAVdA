/* 
 * Astro-Physics GTO mount daemon 
 * Copyright (C) 2009-2010, Markus Wildi, Petr Kubanek <petr@kubanek.net> and INDI
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details. 
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include "../utils/libnova_cpp.h"
#include "../utils/rts2config.h"

#include "tellx200.h"
#include "clicupola.h"
#include "pier-collision.h"

#define OPT_APGTO_ASSUME_PARKED  OPT_LOCAL + 55
#define OPT_APGTO_FORCE_START    OPT_LOCAL + 56
#define OPT_APGTO_CCD_DEVICE     OPT_LOCAL + 57
#define OPT_APGTO_KEEP_HORIZON   OPT_LOCAL + 58

#define SLEW_RATE_1200 '2'
#define SLEW_RATE_0900 '1'
#define SLEW_RATE_0600 '0'

#define MOVE_RATE_120000 '0'
#define MOVE_RATE_060000 '1'
#define MOVE_RATE_006400 '2'
#define MOVE_RATE_001200 '3'
#define MOVE_RATE_000100 '4'
#define MOVE_RATE_000050 '5'
#define MOVE_RATE_000025 '6'

#define DIR_NORTH 'n'
#define DIR_EAST  'e'
#define DIR_SOUTH 's'
#define DIR_WEST  'w'

#define setAPPark()         serConn->writePort ("#:KA#", 5)
#define setAPUnPark()       serConn->writePort ("#:PO#", 5)// ok, no response
#define setAPLongFormat()   serConn->writePort ("#:U#", 4) // ok, no response
#define setAPClearBuffer()  serConn->writePort ("#", 1)    // clear buffer

#define PARK_POSITION_RA -270.
#define PARK_POSITION_DEC 5.
#define PARK_POSITION_ANGLE "West"
#define PARK_POSITION_DIFFERENCE_MAX 2.

#define TRACK_MODE_LUNAR         0 
#define TRACK_MODE_SOLAR         1 
#define TRACK_MODE_SIDEREAL      2 
#define TRACK_MODE_ZERO          3 
#define TRACK_MODE_ZERO_NO_RESET 4 
#define DIFFERENCE_MAX_WHILE_NOT_TRACKING 1.  // [deg]

#define TIMEOUT_CCD_NOTTAKING_IMAGE 300. // [second]
#define TIMEOUT_SLEW_START 300. // [second]

namespace rts2teld
{
/**
 * Driver for Astro-Physics GTO protocol based mount. Some functions are
 * borrowed from INDI to make the transition smoother.
 * 
 * @author Markus Wildi
 * @author Petr Kubanek
 * @author Francisco Foster Buron
 */
class APGTO:public TelLX200 {
	public:
		APGTO (int argc, char **argv);
		virtual ~APGTO (void);
		virtual int processOption (int in_opt);
		virtual int init ();
		virtual int initValues ();
		virtual int info ();
		virtual int idle ();
		virtual int willConnect (Rts2Address * in_addr);

		virtual int setTo (double set_ra, double set_dec);
		virtual int correct (double cor_ra, double cor_dec, double real_ra, double real_dec);

		virtual int startResync ();
		virtual int isMoving ();
		virtual int stopMove ();
  
		virtual int startPark ();
		virtual int isParking ();
		virtual int endPark ();
  
		virtual int startDir (char *dir);
		virtual int stopDir (char *dir);
		virtual int abortAnyMotion () ;
		virtual bool shutterClosed() ;
		virtual int commandAuthorized (Rts2Conn * conn);
		virtual void valueChanged (Rts2Value * changed_value);
	protected:
		virtual int applyCorrectionsFixed (double ra, double dec);
		virtual void applyOffsets (struct ln_equ_posn *pos);

		virtual void startCupolaSync ();
		virtual void notMoveCupola ();

	private:
		char *ccdDevice;
		int force_start ;
		double on_set_HA ;
		double on_zero_HA ;
		double lastMoveRa, lastMoveDec;  
		enum { NOTMOVE, MOVE_REAL } move_state;
		time_t move_timeout;
		time_t slew_start_time;

		int f_scansexa (const char *str0, double *dp);
		void getSexComponents(double value, int *d, int *m, int *s) ;

		// Astro-Physics LX200 protocol specific functions
		int getAPVersionNumber ();
		int getAPUTCOffset ();
		int setAPUTCOffset (int hours);
		int APSyncCMR (char *matchedObject);
		int selectAPMoveToRate (int moveToRate);
		int selectAPTrackingMode (int trackMode);
		int tel_read_declination_axis ();
		int tel_check_declination_axis ();

		int tel_set_move_rate (char moveToRate);
		int setAPSiteLongitude( double Long) ;
		int setAPSiteLatitude( double Lat) ;
		int setAPLocalTime(int x, int y, int z) ;
		int setAPBackLashCompensation( int x, int y, int z) ;
		int setCalenderDate( int dd, int mm, int yy) ;
		int setAPMotionStop() ;

		// helper
		double siderealTime ();
		int checkSiderealTime (double limit) ;
		int setBasicData ();
		void ParkDisconnect ();

		int tel_slew_to (double ra, double dec);
		int tel_slew_to_altaz (double alt, double az);
		void set_move_timeout (time_t plus_time);
		/**
		 * Perform movement, do not move tube bellow horizon.
		 * @author Francisco Foster Buron
		 */
		int moveAvoidingHorizon (double ra, double dec);
		double HAp(double DECp, double DECpref, double HApref);
    		bool moveandconfirm(double interHAp, double interDECp); // FF: move and confirm that the position is reached
		int isInPosition(double coord1, double coord2, double err1, double err2, char coord); // Alonso: coord e {'a', 'c'} a = coordenadas en altaz y c lo otro

		int tel_check_coords (double ra, double dec);

		// fixed offsets
		Rts2ValueRaDec   *fixedOffsets;
		// further discussion with Petr required
		//int changeMasterState (int new_state);
		// Astro-Physics properties
		Rts2ValueAltAz   *APAltAz;
		Rts2ValueInteger *APslew_rate;
		Rts2ValueInteger *APmove_rate;
		Rts2ValueDouble  *APlocal_sidereal_time;
		Rts2ValueDouble  *APlocal_time;
		Rts2ValueDouble  *APutc_offset;
		Rts2ValueDouble  *APlongitude;
		Rts2ValueDouble  *APlatitude;
		Rts2ValueString  *APfirmware ;
		Rts2ValueString  *DECaxis_HAcoordinate ; // see pier_collision.c 
		Rts2ValueBool    *mount_tracking ;
		Rts2ValueBool    *transition_while_tracking ; 
		Rts2ValueBool    *block_sync_move;
		Rts2ValueBool    *assume_parked;
		Rts2ValueBool    *slew_state; // (move_state)
		Rts2ValueBool    *collision_detection; 
		Rts2ValueBool    *avoidBellowHorizon;
};

};

using namespace rts2teld;

void APGTO::getSexComponents(double value, int *d, int *m, int *s)
{
	*d = (int) fabs(value);
	*m = (int) ((fabs(value) - *d) * 60.0);
	*s = (int) rint(((fabs(value) - *d) * 60.0 - *m) *60.0);

	if (value < 0)
		*d *= -1;
}

int APGTO::f_scansexa ( const char *str0, double *dp) /* input string, cracked value, if return 0 */
{
	double a = 0, b = 0, c = 0;
	char str[128];
	char *neg;
	int r;

	/* copy str0 so we can play with it */
	strncpy (str, str0, sizeof(str)-1);
	str[sizeof(str)-1] = '\0';

	neg = strchr(str, '-');
	if (neg)
		*neg = ' ';
	r = sscanf (str, "%lf%*[^0-9]%lf%*[^0-9]%lf", &a, &b, &c);
	if (r < 1)
		return (-1);
	*dp = a + b/60 + c/3600;
	if (neg)
		*dp *= -1;
	return (0);
}

/**
 * Reads from APGTO the version character of the firmware
 *
 * @param none
 *
 * @return -1 and set errno on error, otherwise 0
 */
int APGTO::getAPVersionNumber()
{
	int ret= -1 ;

	char version[32];
	APfirmware->setValueString ("none");
	if (( ret = serConn->writeRead ( "#:V#", 4, version, 32, '#')) < 1 )
		return -1 ;
	version[ret] = '\0';
	APfirmware->setValueString ("test"); //version);
	return 0 ;
}

/**
 * Reads from APGTO the UTC offset and corrects the output according to Astro-Physics documentation firmware revision D
 * @param none
 *
 * @return -1 and set errno on error, otherwise 0
 */
int APGTO::getAPUTCOffset()
{
	int nbytes_read = 0;
	double offset ;

	char temp_string[16];

	if ((nbytes_read = serConn->writeRead ("#:GG#", 5, temp_string, 11, '#')) < 1 )
		return -1 ;
	// Negative offsets, see AP keypad manual p. 77
	if ((temp_string[0]== 'A') || ((temp_string[0]== '0') && (temp_string[1]== '0')) || (temp_string[0]== '@'))
	{
		int i;
		for (i = nbytes_read; i > 0; i--)
		{
			temp_string[i]= temp_string[i-1];
		}
		temp_string[0] = '-';
		temp_string[nbytes_read + 1] = '\0';

		if (temp_string[1] == 'A')
		{
			temp_string[1]= '0';
			switch (temp_string[2])
			{
				case '5':
					temp_string[2]= '1';
					break;
				case '4':
					temp_string[2]= '2';
					break;
				case '3':
					temp_string[2]= '3';
					break;
				case '2':
					temp_string[2]= '4';
					break;
				case '1':
					temp_string[2]= '5';
					break;
				default:
					logStream (MESSAGE_ERROR) << "APGTO::getAPUTCOffset string not handled >" << temp_string << "<END" << sendLog;
					return -1;
			}
		}
		else if (temp_string[1]== '0')
		{
			temp_string[1]= '0';
			temp_string[2]= '6';
		}
		else if( temp_string[1]== '@')
		{
			temp_string[1]= '0' ;
			switch (temp_string[2])
			{
				case '9':
					temp_string[2]= '7';
					break;
				case '8':
					temp_string[2]= '8';
					break;
				case '7':
					temp_string[2]= '9';
					break;
				case '6':
					temp_string[2]= '0';
					break;
				case '5':
					temp_string[1]= '1';
					temp_string[2]= '1';
					break;
				case '4':
					temp_string[1]= '1';
					temp_string[2]= '2';
					break;
				default:
					logStream (MESSAGE_DEBUG) <<"APGTO::getAPUTCOffset string not handled >" << temp_string << "<END" << sendLog;
					return -1;
			}    
		}
		else
		{
			logStream (MESSAGE_ERROR) <<"APGTO::getAPUTCOffset string not handled >" << temp_string << "<END" << sendLog;
		}
	}
	else
	{
		temp_string[nbytes_read - 1] = '\0';
	}
	if (f_scansexa(temp_string, &offset))
	{
		logStream (MESSAGE_ERROR) <<"APGTO::getAPUTCOffset string not handled  >" << temp_string << "<END" <<sendLog;
		return -1;
	}
	APutc_offset->setValueDouble( offset * 15. ) ;
	//logStream (MESSAGE_DEBUG) <<"APGTO::getAPUTCOffset received string >" << temp_string << "<END offset is " <<  APutc_offset->getValueDouble() <<sendLog;
	return 0;
}

/*!
 * Writes to APGTO UTC offset
 *
 *
 * @param hours
 *
 * @return -1 on failure, 0 otherwise
 */
int APGTO::setAPUTCOffset (int hours)
{
	int ret= -1;
	int h, m, s ;
	char temp_string[16];
	char retstr[1] ;
	// To avoid the peculiar output format of AP controller, see p. 77 key pad manual
	if (hours < 0.)
	{
		hours += 24.;
	}
	getSexComponents(hours, &h, &m, &s);
    
	snprintf (temp_string, sizeof(temp_string), "#:SG %+03d#", h);

	if ((ret = serConn->writeRead (temp_string, sizeof (temp_string), retstr, 1)) < 0)
		return -1;
	return 0;
}
/*!
 * Writes to APGTO #:Q#
 *
 *
 * @param 
 *
 * @return -1 on failure, 0 otherwise
 */
int APGTO::setAPMotionStop()
{
	int error_type;
    
	if ( (error_type = serConn->writePort ("#:Q#", 4)) < 0)
		return error_type;

	mount_tracking->setValueBool (false);
	notMoveCupola ();
	return 0 ;
}

/*!
 * Writes to APGTO sync #:CMR
 *
 *
 * @param *matchedObject 
 *
 * @return -1 on failure, 0 otherwise
 */
// wildi: not yet in in use
int 
APGTO::APSyncCMR(char *matchedObject)
{
	int error_type;
    
	if ( (error_type = serConn->writeRead ("#:CMR#", 6, matchedObject, 33, '#')) < 0)
		return error_type;
	return 0;
}
/*!
 * Writes to APGTO the tracking mode
 *
 *
 * @param trackMode
 *
 * @return -1 on failure, 0 otherwise
 */
int APGTO::selectAPTrackingMode(int trackMode)
{
  int error_type;
  double RA ;
  double JD ;
  double lng ;
  double local_sidereal_time ;
  switch (trackMode) {
    /* Lunar */
  case TRACK_MODE_LUNAR:
    //logStream (MESSAGE_DEBUG) <<"APGTO::selectAPTrackingMode setting tracking mode to lunar." << sendLog;
    if ( (error_type = serConn->writePort ("#:RT0#", 6)) < 0)
      return error_type;
    mount_tracking->setValueBool(true) ;
    break;
    
    /* Solar */
  case TRACK_MODE_SOLAR:
    //logStream (MESSAGE_DEBUG) <<"APGTO::selectAPTrackingMode setting tracking mode to solar." << sendLog;
    if ( (error_type = serConn->writePort ("#:RT1#", 6)) < 0)
      return error_type;
    mount_tracking->setValueBool(true) ;
    break;

    /* Sidereal */
  case TRACK_MODE_SIDEREAL:
    //logStream (MESSAGE_DEBUG) <<"APGTO::selectAPTrackingMode setting tracking mode to sidereal." << sendLog;
    if ( (error_type = serConn->writePort ("#:RT2#", 6)) < 0)
      return error_type;
    mount_tracking->setValueBool(true) ;
    break;

    /* Zero, used normally */
  case TRACK_MODE_ZERO:
    //logStream (MESSAGE_DEBUG) <<"APGTO::selectAPTrackingMode setting tracking mode to zero." << sendLog;
    if ( (error_type = serConn->writePort ( "#:RT9#", 6)) < 0)
      return error_type;
    mount_tracking->setValueBool(false) ;
    notMoveCupola() ;

    if( tel_read_ra ()) {
      logStream (MESSAGE_ERROR) <<"APGTO::selectAPTrackingMode can not read RA." << sendLog;
      return -1 ;
    }
    RA  = getTelRa() ;
    JD  = ln_get_julian_from_sys ();
    lng = telLongitude->getValueDouble ();
    local_sidereal_time= fmod((ln_get_mean_sidereal_time( JD) * 15. + lng + 360.), 360.);  // longitude positive to the East
    on_zero_HA= fmod( local_sidereal_time- RA+ 360., 360.) ;
    break;
    /* Zero, used if necessary during ::info, ::idle */
  case TRACK_MODE_ZERO_NO_RESET:
    //logStream (MESSAGE_DEBUG) <<"APGTO::selectAPTrackingMode setting tracking mode to zero, no reset of HA." << sendLog;
    if ( (error_type = serConn->writePort ( "#:RT9#", 6)) < 0)
      return error_type;
    mount_tracking->setValueBool(false) ;
    notMoveCupola() ;

    break;
  default:
    return -1;
    break;
  }
  return 0;
}

/*!
 * Writes to APGTO the observatory longitude
 *
 *
 * @param Long
 *
 * @return -1 on failure, 0 otherwise
 */
int APGTO::setAPSiteLongitude (double Long)
{
	int ret = -1;
	int d, m, s;
	char temp_string[32];
	char retstr[1];

	// fix east longitudes..
	if (Long < 0)
		Long += 360;

	getSexComponents (Long, &d, &m, &s);
	snprintf (temp_string, sizeof( temp_string ), "#:Sg %03d*%02d:%02d#", d, m, s);
  	if (( ret= serConn->writeRead ( temp_string, sizeof( temp_string ), retstr, 1)) < 0)
		return -1;
	return 0;
}

/*!
 * Writes to APGTO the observatory latitude
 *
 *
 * @param Lat
 *
 * @return -1 on failure, 0 otherwise
 */
int APGTO::setAPSiteLatitude(double Lat)
{
	int ret = -1;
	int d, m, s;
	char temp_string[32];
	char retstr[1];

	getSexComponents(Lat, &d, &m, &s);
	snprintf (temp_string, sizeof( temp_string ), "#:St %+02d*%02d:%02d#", d, m, s);
	if (( ret = serConn->writeRead ( temp_string, sizeof(temp_string), retstr, 1)) < 0)
		return -1 ;
	return 0;
}

/*!
 * Writes to APGTO the gear back lash compensation
 *
 *
 * @param x see Astro-Physics documentation
 * @param y see Astro-Physics documentation
 * @param z see Astro-Physics documentation
 *
 * @return -1 on failure, 0 otherwise
 */
int APGTO::setAPBackLashCompensation( int x, int y, int z)
{
	int ret = -1;
	char temp_string[16];
	char retstr[1];
  
	snprintf(temp_string, sizeof( temp_string ), "%s %02d:%02d:%02d#", "#:Br", x, y, z);
  
	if ((ret = serConn->writeRead ( temp_string, sizeof(temp_string), retstr, 1)) < 0)
		return -1 ;
	return 0;
}
/*!
 * Writes to APGTO the local timezone time
 *
 *
 * @param x hour
 * @param y minute
 * @param z seconds
 *
 * @return -1 on failure, 0 otherwise
 */
int APGTO::setAPLocalTime(int x, int y, int z)
{
	int ret = -1;
	char temp_string[16];
	char retstr[1];
  
	snprintf (temp_string, sizeof( temp_string ), "%s %02d:%02d:%02d#", "#:SL" , x, y, z);
	if ((ret = serConn->writeRead (temp_string, sizeof(temp_string), retstr, 1)) < 0)
		return -1;
	return 0;
}
/*!
 * Writes to APGTO the local timezone date
 *
 *
 * @param dd day
 * @param mm month
 * @param yy year
 *
 * @return -1 on failure, 0 otherwise
 */
int APGTO::setCalenderDate(int dd, int mm, int yy)
{
	char cmd_string[32];
	char temp_string[256];

	//Command: :SC MM/DD/YY#
	//Response: 32 spaces followed by “#”, followed by 32 spaces, followed by “#”
	//Sets the current date. Note that year fields equal to or larger than 97 are assumed to be 20 century, 
	//Year fields less than 97 are assumed to be 21st century.

	yy = yy % 100;
	snprintf(cmd_string, sizeof(cmd_string), "#:SC %02d/%02d/%02d#", mm, dd, yy);

	if (serConn->writeRead (cmd_string, 14, temp_string, 33, '#') < 1)
		return -1;
	if (serConn->readPort (temp_string, 33, '#') < 1)
		return -1 ;
	// wildi ToDo: completely unclear:
	// if this line is commented out, subsequent RS 232 reads fail!
	logStream (MESSAGE_INFO) << sendLog;
	return 0;
}

/*!
 * Reads APGTO relative position of the declination axis to the observed hour angle.
 *
 * @return -1 on error, otherwise 0
 *
 */
int APGTO::tel_read_declination_axis ()
{
	int ret ;
	char new_declination_axis[32] ;

	ret = serConn->writeRead ("#:pS#", 5, new_declination_axis, 5, '#');
	if (ret < 0)
		return -1;
	new_declination_axis[ret - 1] = '\0';
	
	DECaxis_HAcoordinate->setValueString(new_declination_axis) ;
	return 0;
}

/*!
 * Reads and checks APGTO relative position of the declination axis to the observed hour angle.
 *
 * @return -1 on error, otherwise 0, or exit(1) in case of a serious failure
 *
 */
int
APGTO::tel_check_declination_axis ()
{
  int stop= 0 ;
// make sure that the (declination - optical axis) have the correct sign
// if not you'll buy a new tube.
  if (tel_read_ra () || tel_read_dec ()) {
    stop= 1 ;
  }  
  double HA, HA_h ;
  double RA, RA_h ;
  char RA_str[32] ;
  char HA_str[32] ;
  int h, m, s ;

  double JD  = ln_get_julian_from_sys ();
  double lng = telLongitude->getValueDouble ();
  double local_sidereal_time= fmod((ln_get_mean_sidereal_time( JD) * 15. + lng + 360.), 360.);  // longitude positive to the East

  RA  = getTelRa() ;
  RA_h= RA/15. ;

  getSexComponents( RA_h, &h, &m, &s) ;
  snprintf(RA_str, 9, "%02d:%02d:%02d", h, m, s);

  HA  = fmod( local_sidereal_time- RA+ 360., 360.) ;
  HA_h= HA / 15. ;

  getSexComponents( HA_h, &h, &m, &s) ;
  snprintf(HA_str, 9, "%02d:%02d:%02d", h, m, s);
	
  if( tel_read_declination_axis()) {
    block_sync_move->setValueBool(true) ;
    stop = 1 ;
    logStream (MESSAGE_ERROR) << "APGTO::tel_check_declination_axis could not retrieve sign of declination axis, severe error, exiting." << sendLog;
  }
  if (!strcmp("West", DECaxis_HAcoordinate->getValue())) {
    if(( HA > 180.) && ( HA <= 360.)) {
    } else {
      if( transition_while_tracking->getValueBool()) {
	// the mount is allowed to transit the meridian while tracking
      } else {
	// block_sync_move->setValueBool(true) ;
	// stop = 1 ;
	logStream (MESSAGE_ERROR) << "APGTO::tel_check_declination_axis sign of declination and optical axis is wrong (HA=" << HA_str<<", West), severe error, blocking any sync and moves" << sendLog;
	// can not exit here because if HA>0, West the telecope must be manually "rot e" to the East
      }
    }
  } else if (!strcmp("East", DECaxis_HAcoordinate->getValue())) {
    if(( HA >= 0.0) && ( HA <= 180.)) {
    } else {
      if( force_start != true) {
	logStream (MESSAGE_ERROR) << "APGTO::tel_check_declination_axis sign of declination and optical axis is wrong (HA=" << HA_str<<", East), severe error." << sendLog;
	stop = 1 ;
      } else {
	block_sync_move->setValueBool(true) ;
	stop = 1 ;
	logStream (MESSAGE_ERROR) << "APGTO::tel_check_declination_axis sign of declination and optical axis is wrong (HA=" << HA_str<<", East), severe error, sync manually, blocking any sync and moves" << sendLog;
      }
    }
  }
  if( stop != 0) {
    if( (abortAnyMotion () !=0)) {
      logStream (MESSAGE_ERROR) << "APGTO::tel_check_declination_axis failed to stop any tracking and motion" << sendLog;
    }

    return -1 ;
  } else {
    return 0 ;
  }
}

/*!
 * Set move rate.
 *
 * @param new_rate	new move speed to set.
 *
 * @return -1 on failure & set errno, 5 (>=0) otherwise
 */
int
APGTO::tel_set_move_rate (char moveToRate)
{
  char command[6];
  switch (moveToRate) {
    /* 0.25x*/
  case MOVE_RATE_000025:
    //logStream (MESSAGE_DEBUG) <<"APGTO::tel_set_move_rate setting move to rate to 0.25x." << sendLog;
    strcpy( command, "#:RG0#"); 
    /* 0.5x*/
    break;
  case MOVE_RATE_000050:
    //logStream (MESSAGE_DEBUG) <<"APGTO::tel_set_move_rate setting move to rate to 0.5x." << sendLog;
    strcpy( command, "#:RG1#"); 
    break;
    /* 1x*/
  case MOVE_RATE_000100:
    //logStream (MESSAGE_DEBUG) <<"APGTO::tel_set_move_rate setting move to rate to 1x." << sendLog;
    strcpy( command, "#:RG2#"); 
    break;
    /* 12x*/
  case MOVE_RATE_001200:
    //logStream (MESSAGE_DEBUG) <<"APGTO::tel_set_move_rate setting move to rate to 12x." << sendLog;
    strcpy( command, "#:RC0#"); 
    break;
    /* 64x */
  case MOVE_RATE_006400:
    //logStream (MESSAGE_DEBUG) <<"APGTO::tel_set_move_rate setting move to rate to 64x." << sendLog;
    strcpy( command, "#:RC1#"); 
    break;
    /* 600x */
  case MOVE_RATE_060000: 
    //logStream (MESSAGE_DEBUG) <<"APGTO::tel_set_move_rate setting move to rate to 600x." << sendLog;
    strcpy( command, "#:RC2#"); 
    break;
    /* 1200x */
  case MOVE_RATE_120000:
    //logStream (MESSAGE_DEBUG) <<"APGTO::tel_set_move_rate setting move to rate to 1200x." << sendLog;
    strcpy( command, "#:RC3#"); 
    break;
  default:
    return -1;
    break;
  }
  return serConn->writePort (command, 6);
}

/*!
 * Slew (=set) APGTO to new coordinates.
 *
 * @param ra 		new right ascenation
 * @param dec 		new declination
 *
 * @return -1 on error, otherwise 0
 */
int
APGTO::tel_slew_to (double ra, double dec)
{
  int ret ;
  char retstr;
  char target_DECaxis_HAcoordinate[32] ;
  struct ln_lnlat_posn observer;
  struct ln_equ_posn target_equ;
  struct ln_hrz_posn hrz;
  double JD;

  if( block_sync_move->getValueBool()) {
    logStream (MESSAGE_INFO) << "APGTO::tel_slew_to slew is blocked, see BLOCK_SYNC_MOVE, ignore slew command to RA " << ra << "Dec "<< dec << sendLog;
    return -1 ;
  }
  if( slew_state->getValueBool()) {
    logStream (MESSAGE_INFO) << "APGTO::tel_slew_to mount is slewing, ignore slew command to RA " << ra << "Dec "<< dec << sendLog;
    return -1 ;
  }
  if( !( collision_detection->getValueBool())) {

    logStream (MESSAGE_INFO) << "APGTO::tel_slew_to collision detection is disabled" << sendLog;
    return -1 ;
  }

  target_equ.ra = fmod( ra + 360., 360.) ;
  target_equ.dec= fmod( dec, 90.);
  observer.lng = telLongitude->getValueDouble ();
  observer.lat = telLatitude->getValueDouble ();

  JD = ln_get_julian_from_sys ();
  ln_get_hrz_from_equ (&target_equ, &observer, JD, &hrz);

  if( hrz.alt < 0.) {
    logStream (MESSAGE_ERROR) << "APGTO::tel_slew_to target_equ ra " << target_equ.ra << " dec " <<  target_equ.dec << " is below horizon" << sendLog;
    return -1 ;
  }
  if( tel_check_declination_axis()) {
    logStream (MESSAGE_ERROR) << "APGTO::tel_slew_to check of the declination axis failed." << sendLog;
    return -1;
  }
  double local_sidereal_time= fmod((ln_get_mean_sidereal_time( JD) * 15. + observer.lng + 360.), 360.);  // longitude positive to the East
  double target_HA= fmod( local_sidereal_time- target_equ.ra+ 360., 360.) ;

  if(( target_HA > 180.) && ( target_HA <= 360.)) {
    strcpy(target_DECaxis_HAcoordinate, "West");
    //logStream (MESSAGE_DEBUG) << "APGTO::tel_slew_to assumed target is 180. < HA <= 360." << sendLog;
  } else {
    strcpy(target_DECaxis_HAcoordinate, "East");
    //logStream (MESSAGE_DEBUG) << "APGTO::tel_slew_to assumed target is 0. < HA <= 180." << sendLog;
  }

  if( !( strcmp( "West", target_DECaxis_HAcoordinate))) {
    ret= pier_collision( &target_equ, &observer) ;
  } else if( !( strcmp( "East", target_DECaxis_HAcoordinate))) {
    //really target_equ.dec += 180. ;
    struct ln_equ_posn t_equ;
    t_equ.ra = target_equ.ra ;
    t_equ.dec= target_equ.dec + 180. ;
    ret= pier_collision( &t_equ, &observer) ;
  }
  if(ret != NO_COLLISION) {
    if( ret== COLLIDING) {
      logStream (MESSAGE_ERROR) << "APGTO::tel_slew_to NOT slewing ra " << target_equ.ra << " dec " << target_equ.dec << " COLLIDING, NOT syncing cupola"  << sendLog;
    } else if( ret== WITHIN_DANGER_ZONE){
      logStream (MESSAGE_ERROR) << "APGTO::tel_slew_to NOT slewing ra " << target_equ.ra << " dec " << target_equ.dec << " within DANGER zone, NOT syncing cupola"  << sendLog;
    } else {
      logStream (MESSAGE_ERROR) << "APGTO::tel_slew_to NOT slewing ra " << target_equ.ra << " target_equ.dec " << target_equ.dec << " invalid condition, exiting"  << sendLog;
      exit(1) ;
    }
    return -1;
  }
  logStream (MESSAGE_INFO) << "APGTO::tel_slew_to cleared geometry  slewing ra " << target_equ.ra << " target_equ.dec " << dec  << sendLog;

  if (( ret=tel_write_ra (target_equ.ra)) < 0) {
    logStream (MESSAGE_DEBUG) << "APGTO::tel_slew_to, not slewing, tel_write_ra return value was " << ret << sendLog ;
    return -1;
  }
  if (( ret=tel_write_dec (target_equ.dec)) < 0) {
    logStream (MESSAGE_DEBUG) << "APGTO::tel_slew_to, not slewing, tel_write_dec return value was " << ret << sendLog ;
    return -1;
  }
  // (re-)enable tracking
  // in case the mount detected a collision, tracking and any motion is stopped
  // but:
  // if target position is ok, mount can track again
  // renable tracking before slew
  if( ! mount_tracking->getValueBool()) {
    if ( selectAPTrackingMode(TRACK_MODE_SIDEREAL) < 0 ) { 
      logStream (MESSAGE_ERROR) << "APGTO::tel_slew_to set track mode sidereal failed." << sendLog;
      notMoveCupola() ;
      return -1;
    } else {
      logStream (MESSAGE_DEBUG) << "APGTO::tel_slew_to set track mode sidereal (re-)enabled." << sendLog;
    }
  }
	// slew now
	if ((ret = serConn->writeRead ("#:MS#", 5, &retstr, 1)) < 0)
		return -1;

  //logStream (MESSAGE_DEBUG) << "APGTO::tel_slew_to #:MS# on ra " << ra << ", dec " << dec << sendLog ;
  if (retstr == '0') {
    if(( ret= tel_read_declination_axis()) != 0)
      return -1 ;
    // check if the assumption was correct
    if( !( strcmp( target_DECaxis_HAcoordinate, target_DECaxis_HAcoordinate))) {
      //logStream (MESSAGE_ERROR) << "APGTO::tel_slew_to assumed target is correct" << sendLog;
    } else {
      if( (abortAnyMotion () !=0)) {
	logStream (MESSAGE_ERROR) << "APGTO::tel_slew_to stoped any tracking and motion" << sendLog;
      }
    }
    on_set_HA= target_HA ; // used for defining state transition while tracking
    time(&slew_start_time);
    return 0;
  }
  logStream (MESSAGE_ERROR) << "APGTO::tel_slew_to NOT slewing ra " << target_equ.ra << " dec " << target_equ.dec << " got '0'!= >" << retstr<<"<END, NOT syncing cupola"  << sendLog;
  return -1;
}
/*!
 * Check, if mount match given coordinates.
 *
 * @param ra		target right ascenation
 * @param dec		target declination
 *
 * @return -1 on error, 0 if not matched, 1 if matched, 2 if timeouted
 */
int
APGTO::tel_check_coords (double ra, double dec)
{
  // ADDED BY JF
  double JD;

  double sep;
  time_t now;

  struct ln_equ_posn object, target;
  struct ln_lnlat_posn observer;
  struct ln_hrz_posn hrz;

  time (&now);
  if (now > move_timeout)
    return 2;

  if ((tel_read_ra () < 0) || (tel_read_dec () < 0))
    return -1;

  // ADDED BY JF
  // CALCULATE & PRINT ALT/AZ & HOUR ANGLE TO LOG
  object.ra = fmod(getTelRa () + 360., 360.);
  object.dec= fmod( getTelDec (), 90.);

  observer.lng = telLongitude->getValueDouble ();
  observer.lat = telLatitude->getValueDouble ();

  JD = ln_get_julian_from_sys ();

  ln_get_hrz_from_equ (&object, &observer, JD, &hrz);
  
  //logStream (MESSAGE_DEBUG) << "APGTO::tel_check_coords MOUNT ALT " << hrz.alt << " AZ " << hrz.az << sendLog;

  target.ra = fmod( ra+ 360., 360.);
  target.dec= fmod( dec , 90.) ;
  
  sep = ln_get_angular_separation (&object, &target);
  
  if (sep > 0.1)
    return 0;
  
  return 1;
}

/*
 * Solo mueve el telescopio  a las coordenadas indicadas, no guarda ningu  valor en la instancia
 */
int
APGTO::tel_slew_to_altaz(double alt, double az)
{
	char retstr;
	
	if(tel_write_altitude(alt) < 0)
	{
		logStream (MESSAGE_ERROR) <<"APGTO::tel_slew_to_altaz tel_write_altitude("<< alt<< ") failed"<< sendLog;
                return -1;
	}
	if(tel_write_azimuth(az) < 0)
	{
		logStream (MESSAGE_ERROR) <<"APGTO::tel_slew_to_altaz tel_write_azimuth("<< az<< ") failed"<< sendLog;
                return -1;
	}
	if (serConn->writeRead ("#:MS#", 5, &retstr, 1) < 0)
        {
                logStream (MESSAGE_ERROR) <<"APGTO::tel_slew_to_altaz tel_write_read #:MS# failed"<< sendLog;
                return -1;
        }

	logStream (MESSAGE_DEBUG) << "APGTO::tel_slew_to_altaz #:MS# on alt " << alt << ", az " << az << sendLog ;
	if(retstr == '0')
	{
		return 0;
	}
	else
	{
		logStream (MESSAGE_ERROR) << "APGTO::tel_slew_to_altaz error:" << "retstring:"<< retstr << "should be '0'" << sendLog;
		return -1;
	}
}

void APGTO::set_move_timeout (time_t plus_time)
{
	time_t now;
	time (&now);

	move_timeout = now + plus_time;
}

int APGTO::moveAvoidingHorizon (double ra, double dec)
{
	double _lst;
	double currentRa, newRA, currentHA, newHA, currentDEC, newDEC;
	double DECp1, HAp1;  // initial point
	double DECp2, HAp2;  // final point
	double DECpS = 53.39095314569674; // critical declination and hour angle in telescope internal coordinates
	double HApS = 115.58068825790751; // S: superior, I: inferior
	double DECpI = -53.39095314569674;
	double HApI = 180. - 115.58068825790751;

	if(tel_read_sidereal_time() < 0)
		return -1;
	_lst = APlocal_sidereal_time->getValueDouble();
	
	if(tel_read_ra())
		return -1;
	
	currentRa = getTelRa();
	newRA = ra;
	
	currentHA = fmod(360.+_lst-currentRa, 360.);
	newHA = fmod(360.+_lst-ra, 360.);
	
	currentDEC = getTelDec();
	newDEC = dec;

	// find telescope internal coordinates
	if (currentHA <= 180)
	{
		DECp1 = fabs(currentDEC + 90.);
		HAp1 = currentHA;
	}
	else
	{
		DECp1 = -fabs(currentDEC + 90.);
		HAp1 = currentHA - 180.;
	}

	if (newHA <= 180)
	{
		DECp2 = fabs(newDEC + 90.);
		HAp2 = currentHA;
	}
	else
	{
		DECp2 = -fabs(newDEC + 90.);
		HAp2 = newHA - 180.;
	}
	  
	// first point is below superior critical line and above inferior critical line
	if (HAp(DECp1,DECpS,HApS) >= HAp1 && HAp(DECp1,DECpI,HApI) <= HAp1)
	{
		logStream (MESSAGE_DEBUG) << "APGTO::moveTo Initial Target in safe zone" << sendLog;

		// move to final target
		if(tel_slew_to(newRA, newDEC) < 0)
			return -1;
		set_move_timeout(100);
		return 0;
	}
	else
	{  // first point is either above superior critical line or below inferior critical line
		if (HAp(DECp1,DECpS,HApS) < HAp1) {  // initial point is above superior critical line:
			// final point is inside point's safe cone or above point's critical line and in the same excluded region
			if ((HAp2 <= HAp1 && DECp2 <= DECp1) || (HAp(DECp2,DECp1,HAp1) <= HAp2 && (DECp2 - DECpS) * (DECp1 - DECpS) >= 0))
			{
				// move to final target
				if(tel_slew_to(newRA, newDEC) < 0)
					return -1;
				set_move_timeout(100);
				return 0;
			}
			else
			{  // initial point is above superior critical line, but final point is not in safe cone
				// final point is below superior critical line or above superior critical line and in different excluded region
				if (HAp(DECp2,DECpS,HApS) > HAp2 || (HAp(DECp2,DECpS,HApS) <= HAp2 && (DECp2 - DECpS) * (DECp1 - DECpS) <= 0))
				{
					// go to superior critical line:
					if (DECp2 > DECp1) {  // with constant DECp...
						if (!moveandconfirm(HAp(DECp1,DECpS,HApS), DECp1))
							return -1; 
	  				} else { // or with constant HAp.
						if (!moveandconfirm(HAp1, DECp1 - (HAp1 - HAp(DECp1,DECpS,HApS))))
							return -1;
	  				}
	  
					// move to final target
					if(tel_slew_to(newRA, newDEC) < 0)
						return -1;
					set_move_timeout(100);
					return 0;
				}
				else
				{ // the final point is above the superior critical line, outside the safe cone and in the same region

					if (DECp2 >= DECp1)
					{ // move with constant DECp
						if (!moveandconfirm(HAp(DECp1,DECp2,HAp2), DECp1))
							return -1;
	 				}
					else
					{ // move with constant HAp
						if (!moveandconfirm(HAp1, DECp1 - (HAp1 - HAp(DECp1,DECp2,HAp2))))
							return -1;
					}
					// move to final target
					if(tel_slew_to(newRA, newDEC) < 0)
						return -1;
					set_move_timeout(100);
					return 0;
				}
			}
		}
		else
		{  // initial point is below inferior critical line
			// final point inside point's safe cone or below point's critical line and in the same excluded region
			if ((HAp2 >= HAp1 && DECp2 >= DECp1) || (HAp(DECp2,DECp1,HAp1) >= HAp2 && (DECp2 - DECpI) * (DECp1 - DECpI) >= 0))
			{
				// move to final target
				if(tel_slew_to(newRA, newDEC) < 0)
					return -1;
				set_move_timeout(100);
				return 0;
			}
			else
			{ 
				// final point above inferior critical line or below inferior critical line and in different excluded region
				if (HAp(DECp2,DECpI,HApI) <  HAp2 || (HAp(DECp2,DECpI,HApI) >= HAp2 && (DECp2 - DECpI) * (DECp1 - DECpI) <= 0))
				{
					// go to inferior critical line
					if (DECp2 > DECp1)
					{ // with constant DECp
						if (!moveandconfirm(HAp(DECp1,DECpI,HApI), DECp1))
							return -1;
					}
					else
					{ // with constant HAp
						if (!moveandconfirm(HAp1, DECp1 + (HAp(DECp1,DECpI,HApI) - HAp1)))
							return -1;
					}
					// move to final target
					if(tel_slew_to(newRA, newDEC) < 0)
						return -1;
					set_move_timeout(100);
					return 0;
				}
				else
				{ // final point below the inferior critical line, outside the safe cone and in the same region
					// go to inferior critical line
					if (DECp2 >= DECp1)
					{ // with constant DECp
						if (!moveandconfirm(HAp(DECp1,DECp2,HAp2), DECp1))
							return -1;
					}
					else
					{ // with constant HAp
						if (!moveandconfirm(HAp1, DECp1 + (HAp(DECp1,DECp2,HAp2) - HAp1)))
							return -1;
					}
					// move to final target
					if (tel_slew_to(newRA, newDEC) < 0)
						return -1;
					set_move_timeout(100);
					return 0;
				}
			}
		}
	}
	
	return 0;
}

// critical line given a reference point and an arbitrary DECp value
double APGTO::HAp(double DECp, double DECpref, double HApref)
{
  return HApref - (DECp - DECpref);
}

bool APGTO::moveandconfirm(double interHAp, double interDECp)
{
	double _lst;

	// given internal telescope coordinates, move and wait until telescope arrival

	logStream (MESSAGE_DEBUG) << "APGTO::moveTo moving to intermediate point"<< sendLog;

	// obtain DEC and RA from telescope internal coordinates DECp and HAp
	if (interDECp >= 0)
	{
		interDECp = interDECp - 90.;
		interHAp = interHAp;
	}
	else
	{
		interDECp = -(90. + interDECp);
		interHAp = interHAp + 180.;
	}    
	_lst = APlocal_sidereal_time->getValueDouble();
	interHAp = fmod(360. + (_lst - interHAp), 360.);

	// move to new auxiliar position
	if (tel_slew_to(interHAp, interDECp) < 0)
		return false;
	while (isInPosition(interHAp, interDECp, .1, .1, 'c') != 0)
	{
		logStream (MESSAGE_DEBUG) << "APGTO::moveTo waiting to arrive to intermediate point"<< sendLog;
		usleep(100000);
	}
	setAPMotionStop();
	return true;
}

// check whether the telescope is in a given position
// coord is 'a' = ALTAZ (altitude, azimuth), 'c' = RADEC (right ascension, declination) or 'h' = HADEC (hour angle, declination)
// Units are degrees

int APGTO::isInPosition(double coord1, double coord2, double err1, double err2, char coord)
{
	double _lst, currentHa;
  
	switch (coord)
	{
		case 'a': // ALTAZ
			if (tel_read_altitude() < 0)
			{
				logStream (MESSAGE_ERROR) << "APGTO::isInPosition tel_read_altitude error" << sendLog;
				return -1;
			}
			if (tel_read_azimuth() < 0)
			{
				logStream (MESSAGE_ERROR) << "APGTO::isInPosition tel_read_altitude error" << sendLog;
				return -1;
			}
			logStream (MESSAGE_DEBUG) << "APGTO::isInPosition distances : alt "<< fabs(APAltAz->getAlt()-coord1)<<
				"tolerance "<< err1<< " az "<< fabs(APAltAz->getAz() - coord2)<< " tolerance "<< err2<< sendLog;

			if(fabs(APAltAz->getAlt()-coord1) < err1 && fabs(APAltAz->getAz()-coord2) < err2)
				return 0;
			return  -1;
		case 'c': // RADEC
			if (tel_read_ra() < 0)
			{
				logStream (MESSAGE_ERROR) << "APGTO::isInPosition tel_read_ra error" << sendLog;
				return -1;
			}
			if (tel_read_dec() < 0)
			{
				logStream (MESSAGE_ERROR) << "APGTO::isInPosition tel_read_dec error" << sendLog;
				return -1;
			}
			logStream (MESSAGE_DEBUG) << "APGTO::isInPosition distances : current ra "<< getTelRa()<< " obj ra "<<coord1 <<
				" dist "<< fabs(getTelRa()-coord1)<< " tolerance "<< err1<<
				" current dec "<< getTelDec()<< " obj dec "<< coord2<< 
				" dist "<< fabs(getTelDec() - coord2)<< " tolerance "<< err2<< sendLog;

			if (fabs(getTelRa()-coord1) < err1 && fabs(getTelDec()-coord2) < err2)
				return 0;
			return -1;
		case 'h': // HADEC
			if (tel_read_sidereal_time() < 0)
				return -1;

			_lst = APlocal_sidereal_time->getValueDouble();
			if (tel_read_ra() < 0)
			{
				logStream (MESSAGE_ERROR) << "APGTO::isInPosition tel_read_ra error" << sendLog;
				return -1;
			}
			if (tel_read_dec() < 0)
			{
				logStream (MESSAGE_ERROR) << "APGTO::isInPosition tel_read_dec error" << sendLog;
				return -1;
			}

			currentHa = fmod(360.+_lst-getTelRa(), 360.);
			logStream (MESSAGE_DEBUG) << "APGTO::isInPosition distances : current ha "<< currentHa<< " next ha "<<coord1<<
				" dist "<< fabs(currentHa-coord1)<< " tolerance "<< err1<<
				" current dec "<< getTelDec()<< " obj dec "<< coord2<<
				" dist "<< fabs(getTelDec() - coord2)<< " tolerance "<< err2<< sendLog;

			if (fabs(currentHa-coord1) < err1 && fabs(getTelDec()-coord2) < err2)
				return 0;
      
		default:
			return  -1;

	}
}

int APGTO::startResync ()
{
	int ret;

	if (avoidBellowHorizon->getValueBool ())
	{
		lastMoveRa = getTelTargetRa ();
		lastMoveDec = getTelTargetDec ();
		if (moveAvoidingHorizon (lastMoveRa, lastMoveDec) < 0)
			return -1;
	}
	else
	{
		lastMoveRa = fmod (getTelTargetRa () + 360., 360.);
		lastMoveDec = fmod (getTelTargetDec (), 90.);

		ret = tel_slew_to (lastMoveRa, lastMoveDec);
		if (ret)
			return -1;
		move_state = MOVE_REAL;
		slew_state->setValueBool(true) ;
		set_move_timeout (100);
	}
	return 0;
}

int APGTO::applyCorrectionsFixed (double ra, double dec)
{
	if (fixedOffsets->getRa () == 0 && fixedOffsets->getDec () == 0)
	{
		fixedOffsets->setValueRaDec (ra, dec);
		logStream (MESSAGE_INFO) << "applying offsets as fixed offsets (" << LibnovaDegDist (ra) << " " << LibnovaDegDist (dec) << sendLog;
		return 0;
	}
	return TelLX200::applyCorrectionsFixed (ra, dec);
}

void APGTO::applyOffsets (struct ln_equ_posn *pos)
{
	TelLX200::applyOffsets (pos);
	pos->ra += fixedOffsets->getRa ();
	pos->dec += fixedOffsets->getDec ();
}

void APGTO::startCupolaSync ()
{
  struct ln_equ_posn target_equ;
  getTarget (&target_equ);

  target_equ.ra = fmod( target_equ.ra + 360., 360.) ;
  target_equ.dec= fmod( target_equ.dec, 90.) ; 
  if( !( strcmp( "West", DECaxis_HAcoordinate->getValue()))) {
    postEvent (new Rts2Event (EVENT_CUP_START_SYNC, (void*) &target_equ));
  } else if( !( strcmp( "East", DECaxis_HAcoordinate->getValue()))) {
    //tel_equ.dec += 180. ;
    struct ln_equ_posn t_equ;
    t_equ.ra = target_equ.ra ;
    t_equ.dec= target_equ.dec + 180. ;
    postEvent (new Rts2Event (EVENT_CUP_START_SYNC, (void*) &t_equ));
  }
  logStream (MESSAGE_INFO) << "APGTO::startCupolaSync sync cupola" << sendLog;
}

void APGTO::notMoveCupola ()
{
  postEvent (new Rts2Event (EVENT_CUP_NOT_MOVE));
}

int
APGTO::isMoving ()
{
  int ret;
  int loc_state= move_state ; // wildi ToDo:, ask Petr, to avoid compile time error
  switch (move_state) {
  case MOVE_REAL:
    ret = tel_check_coords (lastMoveRa, lastMoveDec);
    switch (ret) {
    case -1:
      return -1;
    case 0:
      return USEC_SEC / 10;
    case 1:
    case 2:
      move_state = NOTMOVE;
      slew_state->setValueBool(false) ;
      return -2;
    }
    break;
  case NOTMOVE:
    slew_state->setValueBool(false) ;
    return -2;
    break ;
  default:
    logStream (MESSAGE_ERROR) << "APGTO::isMoving NO case " << loc_state  << sendLog;
    break;
  }
  return -1;
}

int
APGTO::stopMove ()
{
  char dirs[] = { 'e', 'w', 'n', 's' };
  int i;
  for (i = 0; i < 4; i++) {
    if (tel_stop_move (dirs[i]) < 0)
      
      return -1;
  }
  return 0;
}
/*!
 * Set mount to match given coordinates (sync)
 *
 * AP GTO remembers the last position only occasionally it looses it.
 *
 * @param ra		setting right ascscension
 * @param dec		setting declination
 *
 * @return -1 and set errno on error, otherwise 0
 */
int APGTO::setTo (double ra, double dec)
{
	char readback[101];

	if (block_sync_move->getValueBool())
	{
		logStream (MESSAGE_INFO) << "APGTO::setTo sync is blocked, see BLOCK_SYNC_MOVE" << sendLog;
		return -1;
	}
	if (slew_state->getValueBool())
	{
		logStream (MESSAGE_INFO) << "APGTO::setTo mount is slewing, ignore sync command to RA " << ra << "Dec "<< dec << sendLog;
		return -1 ;
	}
	if ( !( collision_detection->getValueBool()))
	{
		logStream (MESSAGE_INFO) << "APGTO::setTo collision detection is disabled, enable it" << sendLog;
		return -1;
	}

	ra = fmod (ra + 360., 360.);
	dec = fmod (dec, 90.);
	if ((tel_write_ra (ra) < 0) || (tel_write_dec (dec) < 0))
		return -1;

	//AP manual:
	//Position
	//Command:  :CM#
	//Response: “Coordinates matched.     #”
	//          (there are 5 spaces between “Coordinates” and “matched”, and 8 trailing spaces before the “#”, 
	//          the total response length is 32 character plus the “#”.	  

	//logStream (MESSAGE_ERROR) <<"APGTO::setTo #:CM# doing SYNC" << sendLog;

	if (serConn->writeRead ("#:CM#", 5, readback, 100, '#') < 0)
		return -1;
	return 0 ;
}
/*!
 * Correct mount coordinates.
 * Used for closed loop coordinates correction based on astronometry
 * of obtained images.
 * @param ra		ra correction
 * @param dec		dec correction
 * @return -1 and set errno on error, 0 otherwise.
 */
int APGTO::correct (double cor_ra, double cor_dec, double real_ra, double real_dec)
{
	if (setTo (real_ra, real_dec)) //  means sync
		return -1;
	return 0;
}
/*!
 * Park mount to neutral location.
 *
 * @return -1 and errno on error, 0 otherwise
 */
int
APGTO::startPark ()
{
  notMoveCupola() ;
  struct ln_lnlat_posn observer;
  observer.lng = telLongitude->getValueDouble ();
  observer.lat = telLatitude->getValueDouble ();

  double JD= ln_get_julian_from_sys ();
  double local_sidereal_time= fmod((ln_get_mean_sidereal_time( JD) * 15. + observer.lng + 360.), 360.);  // longitude positive to the East
  double park_ra= fmod( (PARK_POSITION_RA + local_sidereal_time) + 360., 360.);

  setTarget ( park_ra, PARK_POSITION_DEC);
  startCupolaSync() ;
  logStream (MESSAGE_DEBUG) << "APGTO::startParking " << getTelTargetRa () << " dec " <<getTelTargetDec ()  << sendLog;
  return startResync ();
}
int
APGTO::isParking ()
{
  int ret= isMoving() ;

  if( ret > 0) {
    return USEC_SEC; // still moving
  }
  switch (ret) {
  case -2 :
    return -2 ; // target reached
    break ;
  case -1: // read error 
    logStream (MESSAGE_ERROR) << "APGTO::isParking coordinates read error" << sendLog;
    return -1 ;
    break ;
  default: 
    logStream (MESSAGE_ERROR) << "APGTO::isParking no valid case :" <<ret << sendLog;
    break ;
  }
  return -1 ;
}
int
APGTO::endPark ()
{
  logStream (MESSAGE_DEBUG) << "APGTO::endPark" << sendLog;
  ParkDisconnect() ;
  return 0;
}
void 
APGTO::ParkDisconnect()
{
  // The AP mount will not surely stop with #:KA alone
  if (setAPMotionStop() < 0) {
    logStream (MESSAGE_ERROR) << "APGTO::ParkDisconnect motion stop failed #:Q#" << sendLog;
    return;
  }
  if ( selectAPTrackingMode(TRACK_MODE_ZERO) < 0 ) { 
    logStream (MESSAGE_ERROR) << "APGTO::ParkDisconnect setting tracking mode ZERO failed." << sendLog;
    return;
  }
  // wildi ToDo: not yet decided
  // issuing #:KA# means a power cycle of the mount (not always)
  // From experience I know that the Astro-Physics controller saves the last position
  // without #:KA#, but not always!
  // so no #:KA# is sent and I leave the file handle open
  //  if (setAPPark() < 0) {
  //  logStream (MESSAGE_ERROR) << "APGTO::ParkDisconnect parking failed #:KA#" << sendLog;
  //  return;
  //}
  logStream (MESSAGE_INFO) << "APGTO::ParkDisconnect motion stopped, the mount is parked but still connected (no #:KA#)." << sendLog; 
  return;
}
int
APGTO::startDir (char *dir)
{
  switch (*dir) {
  case DIR_EAST:
  case DIR_WEST:
  case DIR_NORTH:
  case DIR_SOUTH:
    return tel_start_move (*dir);
  }
  return -2;
}
int
APGTO::stopDir (char *dir)
{
  switch (*dir) {
  case DIR_EAST:
  case DIR_WEST:
  case DIR_NORTH:
  case DIR_SOUTH:
    return tel_stop_move (*dir);
  }
  return -2;
}
int
APGTO::abortAnyMotion ()
{
  int ret = 0 ;
  int failed= 0 ;

    if((ret = setAPMotionStop()) < 0) {
      logStream (MESSAGE_ERROR) << "APGTO::abortAnyMotion stop motion #:Q# failed, SWITCH the mount OFF" << sendLog;
      failed = 1 ;
    }
    if (( ret= selectAPTrackingMode(TRACK_MODE_ZERO)) < 0 ) {
      logStream (MESSAGE_ERROR) << "APGTO::abortAnyMotion setting tracking mode ZERO failed." << sendLog;
      failed = 1 ;
    }
    if( failed == 0) {
      logStream (MESSAGE_ERROR) << "APGTO::abortAnyMotion successfully stoped motion #:Q# and tracking" << sendLog;
      return 0 ;
    } else {
      logStream (MESSAGE_ERROR) << "APGTO::abortAnyMotion failed: check motion and tracking now" << sendLog;
      return -1 ;
    }
}
bool
APGTO::shutterClosed() {

  Rts2Conn * conn_shutter = getOpenConnection (ccdDevice);
  if( conn_shutter) {
    Rts2Value * flishutter =  conn_shutter->getValue ("SHUTTER");
    if( flishutter) {
      if(flishutter->getValueType()== RTS2_VALUE_SELECTION) {
	if( flishutter->getValueInteger() !=0)  { //0=LIGHT, 1=DARK
	  return true ;
	}
      }
    } else {
      logStream (MESSAGE_DEBUG) << "APGTO::shutterClosed flishutter=NULL : "<< sendLog;
    }
  }
  return false ;
}
void 
APGTO::valueChanged (Rts2Value * changed_value)
{
  int ret= -1 ;
  int slew_rate= -1 ;
  int move_rate= -1 ;
  char command ;

  if (changed_value ==APslew_rate){
    if(( slew_rate= APslew_rate->getValueInteger())== 1200) {
      command= SLEW_RATE_1200 ;
    } else if( APslew_rate->getValueInteger()== 900) {
      command= SLEW_RATE_0900 ;
    } else if( APslew_rate->getValueInteger()== 600) {
      command= SLEW_RATE_0600 ;
    } else {
      APslew_rate->setValueInteger(-1);  
      logStream (MESSAGE_ERROR) << "APGTO::valueChanged wrong slew rate " << APslew_rate->getValue() << ", valid: 1200, 900, 600" << sendLog;
      return ;
    }

    if(( ret= tel_set_slew_rate (command)) !=5) {
      // wildi ToDo: thinking about what to do in this case
      APslew_rate->setValueInteger(-1); 
      logStream (MESSAGE_ERROR) << "APGTO::valueChanged tel_set_slew_rate failed" << sendLog;
      // return -1 ;
    }
    return ;
  } else if (changed_value ==APmove_rate) {

    if(( move_rate= APmove_rate->getValueInteger())== 600) {
      command= MOVE_RATE_060000 ;
    } else if( APmove_rate->getValueInteger()== 64) {
      	  command= MOVE_RATE_006400 ;
    } else if( APmove_rate->getValueInteger()== 12) {
      command= MOVE_RATE_001200 ;
    } else if( APmove_rate->getValueInteger()== 1) {
      command= MOVE_RATE_000100 ;
    } else {
      APmove_rate->setValueInteger(-1);  
      logStream (MESSAGE_ERROR) << "APGTO::valueChanged wrong move rate " << APmove_rate->getValue() << ", valid: 600, 64, 12, 1" << sendLog;
      return ;
    }

    if(( ret= tel_set_move_rate (command)) !=5) {
      // wildi ToDo: thinking about what to do in this case
      APmove_rate->setValueInteger(-1); 
      logStream (MESSAGE_ERROR) << "APGTO::valueChanged tel_set_move_rate failed" << sendLog;
      // return -1 ;
    }
    return ;
  }
  TelLX200::valueChanged (changed_value);
}

int 
APGTO::commandAuthorized (Rts2Conn *conn)
{
  int ret = -1 ;

  if (conn->isCommand ("abort")) {
    if( (abortAnyMotion () !=0)) {
      logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized failed to stop any tracking and motion" << sendLog;
      return -1 ;
    }
    return 0;
  } else if (conn->isCommand ("track")) {
    if ( selectAPTrackingMode(TRACK_MODE_SIDEREAL) < 0 ) { 
      logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized set track mode sidereal failed." << sendLog;
      return -1;
    }
    return 0 ;
  } else if (conn->isCommand ("rot")) { // move is used for a slew to a specific position
    char *direction ;
    if (conn->paramNextStringNull (&direction) || !conn->paramEnd ()) { 
      logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized rot failed" << sendLog;
      return -2;
    }
    if( startDir( direction)){
      logStream (MESSAGE_ERROR) <<"APGTO::commandAuthorized startDir( direction) failed" << sendLog;
      return -1;
    }
    return 0 ;
  } else if ((conn->isCommand ("move_sg"))||(conn->isCommand ("move_ha"))||(conn->isCommand ("move_ha_sg"))) {
    double move_ra, move_dec ;
    double move_ha ;

    if (conn->isCommand ("move_sg")) {
      char *move_ra_str;
      char *move_dec_str;
      if (conn->paramNextStringNull (&move_ra_str) || conn->paramNextStringNull (&move_dec_str) || !conn->paramEnd ()) { 
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized move_sg paramNextString ra or dec failed" << sendLog;
	return -2;
      }
      if(( ret= f_scansexa ( move_ra_str, &move_ra))== -1) {
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized move_sg parsing ra failed" << sendLog;
	return -1;
      }
      move_ra *= 15. ;
      if(( ret= f_scansexa ( move_dec_str, &move_dec))== -1) {
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized sync_sg parsing dec failed" << sendLog;
	return -1;
      }
    } else if(conn->isCommand ("move_ha")){
      if (conn->paramNextDouble (&move_ha) || conn->paramNextDouble (&move_dec) || !conn->paramEnd ()) { 
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized sync_ha paramNextDouble ra or dec failed" << sendLog;
	return -2;
      }
      double JD= ln_get_julian_from_sys ();
      move_ra= ln_get_mean_sidereal_time( JD) * 15. + telLongitude->getValueDouble () - move_ha; // RA is a right, HA left system

    } else if (conn->isCommand ("move_ha_sg")) {
      char *move_ha_str;
      char *move_dec_str;
      if (conn->paramNextStringNull (&move_ha_str) || conn->paramNextStringNull (&move_dec_str) || !conn->paramEnd ()) { 
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized move_ha_sg paramNextString ra or dec failed" << sendLog;
	return -2;
      }
      if(( ret= f_scansexa ( move_ha_str, &move_ha))== -1) {
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized move_ha_sg parsing ra failed" << sendLog;
	return -1;
      }
      move_ha *= 15. ;
      if(( ret= f_scansexa ( move_dec_str, &move_dec))== -1) {
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized move_ha_sg parsing dec failed" << sendLog;
	return -1;
      }
      double JD= ln_get_julian_from_sys ();
      move_ra= ln_get_mean_sidereal_time( JD) * 15. + telLongitude->getValueDouble () - move_ha; // RA is a right, HA left system
    }
    setTarget ( move_ra, move_dec);
    startCupolaSync() ;
    return startResync ();

  } else if ((conn->isCommand("sync"))||(conn->isCommand("sync_sg"))||(conn->isCommand("sync_ha"))||(conn->isCommand("sync_ha_sg")||(conn->isCommand("sync_delta")))) {
    double sync_ra, sync_dec ;
    double sync_delta_ra, sync_delta_dec ; // defined for calrity
    double sync_ha ;
    if(conn->isCommand ("sync")){
      if (conn->paramNextDouble (&sync_ra) || conn->paramNextDouble (&sync_dec) || !conn->paramEnd ()) { 
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized sync paramNextDouble ra or dec failed" << sendLog;
	return -2;
      }
    } else if(conn->isCommand ("sync_ha")){
      if (conn->paramNextDouble (&sync_ha) || conn->paramNextDouble (&sync_dec) || !conn->paramEnd ()) { 
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized sync_ha paramNextDouble ra or dec failed" << sendLog;
	return -2;
      }
      double JD= ln_get_julian_from_sys ();
      sync_ra= ln_get_mean_sidereal_time( JD) * 15. + telLongitude->getValueDouble () - sync_ha; // RA is a right, HA left system

    } else if (conn->isCommand ("sync_sg")) {
      char *sync_ra_str;
      char *sync_dec_str;
      if (conn->paramNextStringNull (&sync_ra_str) || conn->paramNextStringNull (&sync_dec_str) || !conn->paramEnd ()) { 
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized sync_sg paramNextString ra or dec failed" << sendLog;
	return -2;
      }
      if(( ret= f_scansexa ( sync_ra_str, &sync_ra))== -1) {
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized sync_sg parsing ra failed" << sendLog;
	return -1;
      }
      sync_ra *= 15. ;
      if(( ret= f_scansexa ( sync_dec_str, &sync_dec))== -1) {
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized sync_sg parsing dec failed" << sendLog;
	return -1;
      }
    } else if (conn->isCommand ("sync_ha_sg")) {
      char *sync_ha_str;
      char *sync_dec_str;
      if (conn->paramNextStringNull (&sync_ha_str) || conn->paramNextStringNull (&sync_dec_str) || !conn->paramEnd ()) { 
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized sync_ha_sg paramNextString ra or dec failed" << sendLog;
	return -2;
      }
      if(( ret= f_scansexa ( sync_ha_str, &sync_ha))== -1) {
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized sync_ha_sg parsing ra failed" << sendLog;
	return -1;
      }
      sync_ha *= 15. ;
      if(( ret= f_scansexa ( sync_dec_str, &sync_dec))== -1) {
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized sync_ha_sg parsing dec failed" << sendLog;
	return -1;
      }
      double JD= ln_get_julian_from_sys ();
      sync_ra= ln_get_mean_sidereal_time( JD) * 15. + telLongitude->getValueDouble () - sync_ha; // RA is a right, HA left system
    } else if (conn->isCommand ("sync_delta")) {
      // This command is intended for use only in conjunction with an astrometric calibration script.
      // In order that the mount syncs to the correct location independent of the actual location
      // the offset is added to the mount's current location
      // This can happen even the mount is tracking or exposing
      // Note:
      // If this feature is used the script doing the astrometric calibration must return
      // $crval1-$oira= $crval2-$oirdec= 0. :
      //
      //   $img_id, $crval1, $crval2, $crval1-$oira, $crval2-$oirdec
      //   as e.g.
      //   1 131.254623 18.904446 (0.,0.)
      //
      // in order to bypass rts2's offset (OFFS) mechanism.
      // This feature is doomed to disappear again in future.

      if (conn->paramNextDouble (&sync_delta_ra) || conn->paramNextDouble (&sync_delta_dec) || !conn->paramEnd ()) { 
	logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized sync_delta paramNextDouble ra or dec failed" << sendLog;
	return -2;
      }
      if (tel_read_ra () || tel_read_dec ()) {
	logStream (MESSAGE_ERROR) << "APGTO::valueChanged could not retrieve ra, dec  " << sendLog;
	return -1;
      }
      double tdec= 91. ;
      if((tdec= getTelDec () + sync_delta_dec) >= 90.) {
	logStream (MESSAGE_ERROR) << "APGTO::valueChanged do not sync dec .ge. 90 deg: " << tdec << sendLog;
	return -1;
      }
      struct ln_equ_posn tel, sync;
      getTelRaDec (&tel);

      sync_ra = fmod( getTelRa ()  + sync_delta_ra + 360., 360.);
      sync_dec= fmod( getTelDec () + sync_delta_dec, 90.);

      sync.ra= sync_ra ;
      sync.dec= sync_dec ;

      double sdist= ln_get_angular_separation (&tel, &sync) ;
      if( sdist > 5.) { // ToDo ad hoc limit
	logStream (MESSAGE_INFO) << "APGTO::valueChanged astrometric calibrated sync " << sdist << " is larger than ad hoc limit of 5. degrees " << sendLog;
	return -1;
      }
      logStream (MESSAGE_INFO) << "APGTO::valueChanged syncing astrometrically calibrated to ra " << sync_ra << ", dec " << sync_dec << sendLog;
    }
    // do not sync while slewing
    // this can occur if a script tries to sync after e.g. an astrometric calibration
    //
    while( slew_state->getValueBool()) {
      logStream (MESSAGE_INFO) << "APGTO::valueChanged astrometryOffsetRaDec, sleeping while slewing" << sendLog;
      sleep( 1) ;
    }
    if(( ret= setTo(sync_ra, sync_dec)) !=0) {
      logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized setTo failed" << sendLog;
      return -1 ;
    }
    if( tel_check_declination_axis()) {
      logStream (MESSAGE_ERROR) << "APGTO::commandAuthorized check of the declination axis failed." << sendLog;
      return -1;
    }
    //logStream (MESSAGE_DEBUG) << "APGTO::commandAuthorized sync on ra " << sync_ra << " dec " << sync_dec << sendLog;
    return 0 ;
  }
  return TelLX200::commandAuthorized (conn);
}
#define ERROR_IN_INFO 1
int APGTO::info ()
{
  int ret ;
  int flip= -1 ;
  int error= -1 ;

  
  // if there are more than one CCD running use an iterator
  // and find the appropriate camera
  // if no images have been taken with TIMEOUT_CCD_NOTTACKING_IMAGE secons
  // stop tracking.
  // tracking is reenabled in case a new target has been acquired
  // this section is relying on that for each image a slew is performed
  time_t now;
  if (ccdDevice) {
    if((( slew_start_time - time(&now) + TIMEOUT_SLEW_START) < 0.) || (transition_while_tracking->getValueBool())) {
      if(mount_tracking->getValueBool()) {
	if( shutterClosed()) {
	  if( (abortAnyMotion () !=0)) {
	    logStream (MESSAGE_ERROR) << "APGTO::info abortAnyMotion failed" << sendLog;
	    return -1;
	  } else {
	    logStream (MESSAGE_ERROR) << "APGTO::info stopped tracking due to shutter closed" << sendLog;
	  }
	}
	Rts2Conn * conn_time = getOpenConnection (ccdDevice);
	if( conn_time) {
	  Rts2Value * flitime = conn_time->getValue ("exposure_end"); // it is time when shutter closes!
	  if( flitime) {
	    if( flitime->getValueType() == RTS2_VALUE_TIME) { // No it is not a Double
	      if(( flitime->getValueDouble() - time(&now) + TIMEOUT_CCD_NOTTAKING_IMAGE) < 0.) {
		logStream (MESSAGE_INFO) << "APGTO::info ccd data tacking timed out, flitime: " <<flitime->getValueDouble() << sendLog;
		if( (abortAnyMotion () !=0)) {
		  logStream (MESSAGE_ERROR) << "APGTO::info abortAnyMotion failed" << sendLog;
		  return -1;
		}  else {
		  logStream (MESSAGE_ERROR) << "APGTO::info stopped tracking due to ccd data tacking timed out" << sendLog;
		}
	      } else {
		//logStream (MESSAGE_DEBUG) << "APGTO::info fli time " << ( flitime->getValueDouble() - time(&now) + TIMEOUT_CCD_NOTTAKING_IMAGE) << " > 0." << sendLog;
	      }
	    } else {
	      logStream (MESSAGE_DEBUG) << "APGTO::info time not RTS2_VALUE_TIME "<< sendLog;
	    }
	  }	
	}
      }
    } else {
      //      logStream (MESSAGE_DEBUG) << "APGTO::info slew time " << ( slew_start_time - time(&now) + TIMEOUT_SLEW_START)  << " < 0. slew_start " << slew_start_time << sendLog;
    }
  }
  //  if ((getState () & TEL_MASK_MOVING) == TEL_MOVING){
  //    logStream (MESSAGE_INFO) << "APGTO::info state TEL_MOVING" << sendLog;
  //  }

  if (tel_read_ra () || tel_read_dec ()) {
    error = ERROR_IN_INFO ;
    logStream (MESSAGE_ERROR) << "APGTO::info could not retrieve ra, dec  " << sendLog;
  }

  if(( ret= tel_read_local_time()) != 0) {
    error = ERROR_IN_INFO ;
    logStream (MESSAGE_ERROR) << "APGTO::info could not retrieve localtime  " << sendLog;
  }
  if(( ret= tel_read_sidereal_time()) != 0) {
    error = ERROR_IN_INFO ;
    logStream (MESSAGE_ERROR) << "APGTO::info could not retrieve sidereal time  " << sendLog;
  }
  if(( ret= tel_read_declination_axis()) != 0) {
    error = ERROR_IN_INFO ;
    logStream (MESSAGE_ERROR) << "APGTO::info could not retrieve position of declination axis  " << sendLog;
  }

  if( !( strcmp( "West", DECaxis_HAcoordinate->getValue()))) {
    flip = 1 ;
  } else if( !( strcmp( "East", DECaxis_HAcoordinate->getValue()))) {
    flip= 0 ;
  } else {
    error = ERROR_IN_INFO ;
    logStream (MESSAGE_ERROR) << "APGTO::info could not retrieve angle (declination axis, hour axis)" << sendLog;
  }
  
  telFlip->setValueInteger (flip);
  if (tel_read_azimuth () || tel_read_altitude ()) {
    error = ERROR_IN_INFO ;
    logStream (MESSAGE_ERROR) << "APGTO::info could not retrieve  azimuth altitude " << sendLog;
  }
  if( error== ERROR_IN_INFO) {
    return -1 ;
  }
  // while the telecope is tracking Astro-Physics controller does not
  // carry out any checks, meaning that it turns for ever
  // 
  // 0 <= HA <=180.: check if the mount approaches horizon
  // 180 < HA < 360: check if the mount crosses the meridian
  // Astro-Physics controller has the ability to delay the meridian flip.
  // Here I assume that the flip takes place on HA=0 on slew.
  // If the telescope does not collide, it may track as long as:
  // West:   HA < 15.
  // East:  Alt > 10.

  struct ln_equ_posn object;
  struct ln_lnlat_posn observer;
  object.ra = fmod( getTelRa () + 360., 360.);
  object.dec= fmod( getTelDec (), 90.);
  observer.lng = telLongitude->getValueDouble ();
  observer.lat = telLatitude->getValueDouble ();

  double JD= ln_get_julian_from_sys ();
  double local_sidereal_time= fmod((ln_get_mean_sidereal_time( JD) * 15. + observer.lng + 360.), 360.);  // longitude positive to the East
  double HA= fmod( local_sidereal_time- object.ra+ 360., 360.) ;
  // HA= [0.,360.], HA monoton increasing, but not strictly
  // man fmod: On success, these functions return the value x - n*y
  // e.g. if set on_set_HA=360.: 360. - 1 * 360.= 0.
  // after a slew HA > 0, therefore no transition occurs
  // wildi ToDo: verify the cases
  // move_ha_sg 00:00:00  0:
  // move_ha_sg 24:00:00  0:
  // with the Astro-Physics mount
	if ((HA < on_set_HA) && (mount_tracking->getValueBool()) && (move_state== NOTMOVE))
	{
		transition_while_tracking->setValueBool(true);
		logStream (MESSAGE_INFO) << "APGTO::info transition while tracking occured" << sendLog;
	}
	else
	{
		transition_while_tracking->setValueBool(false) ;
	}
  // wildi ToDo: This is the real stuff
//   if ((getState () & TEL_MASK_MOVING) == TEL_MOVING) {
//     logStream (MESSAGE_INFO) << "APGTO::info  moving" << sendLog ;
//   } else if ((getState () & TEL_MASK_MOVING) == TEL_PARKING) {
//     logStream (MESSAGE_INFO) << "APGTO::info  parking" << sendLog ;
//   } else {
//     logStream (MESSAGE_INFO) << "APGTO::info  not ( moving || parking)" << sendLog ;
//   }
	if (move_state == NOTMOVE)
	{
		int stop= 0 ;
		if (!(strcmp("West", DECaxis_HAcoordinate->getValue())))
		{
			ret= pier_collision( &object, &observer) ;
			if (ret != NO_COLLISION)
			{
				// stop= 1;
				logStream (MESSAGE_ERROR) << "APGTO::info collision detected (West)" << sendLog;
				ret = NO_COLLISION;
			}
			else
			{
				if ((HA >= 15.) && (HA < 180.))
				{ // 15. degrees are a matter of taste
					if (mount_tracking->getValueBool())
					{
						stop = 1;
						logStream (MESSAGE_ERROR) << "APGTO::info t_equ ra " << getTelRa() << " dec " <<  getTelDec() << " is crossing the (meridian + 15 deg), stopping any motion" << sendLog;
					}
				}
			}
		}
		else if (!(strcmp("East", DECaxis_HAcoordinate->getValue())))
		{
			//really target_equ.dec += 180. ;
			struct ln_equ_posn t_equ;
			t_equ.ra = object.ra;
			t_equ.dec= object.dec + 180.;
			ret = pier_collision (&t_equ, &observer);
			if (ret != NO_COLLISION)
			{
				stop = 1;
				logStream (MESSAGE_ERROR) << "APGTO::info collision detected (East)" << sendLog;
			}
			else
			{
				if (APAltAz->getAlt() < 10.)
				{
					if (mount_tracking->getValueBool())
					{
						// stop = 1;
						// notMoveCupola ();
	    					logStream (MESSAGE_ERROR) << "APGTO::info t_equ ra " << getTelRa() << " dec " <<  getTelDec() << " is below 10 deg, stopping any motion" << sendLog;
					}
				}
			}
		} 
		if (stop !=0 )
		{
			// if a collision is detected it is necessary that the mount can be moved with rot e, w, n, s commands
			// set collision_detection to true if that occurs
			if (collision_detection->getValueBool())
			{
				if ((abortAnyMotion () !=0))
				{
					logStream (MESSAGE_ERROR) << "APGTO::info failed to stop any tracking and motion" << sendLog;
					return -1;
				}
			}
			else
			{
				if (selectAPTrackingMode(TRACK_MODE_ZERO) < 0 )
				{
					logStream (MESSAGE_ERROR) << "APGTO::info setting tracking mode ZERO failed." << sendLog;
					return -1;
				}
				logStream (MESSAGE_ERROR) << "APGTO::info stop tracking but not motion" << sendLog;
			}
		}
	}
	else
	{
		ret = isMoving ();
		if (ret > 0)
		{
			// still moving, ignore that here
		}
		else
		{
			switch (ret)
			{
				case -2:
					move_state = NOTMOVE;
					slew_state->setValueBool (false);
					break;
				case -1: // read error 
					logStream (MESSAGE_ERROR) << "APGTO::info coordinates read error" << sendLog;
					break ;
				default: 
					logStream (MESSAGE_ERROR) << "APGTO::info no valid case :" << ret << sendLog;
					break ;
			}
		}
	}
	// There is a bug in the revision D Astro-Physics controller
	// find out, when the local sidereal time gets wrong, difference is 237 sec
	// Check if the sidereal time read from the mount is correct 
	JD = ln_get_julian_from_sys ();
	double lng = telLongitude->getValueDouble ();
	local_sidereal_time = fmod ((ln_get_mean_sidereal_time (JD) * 15. + lng + 360.), 360.);  // longitude positive to the East
	double diff_loc_time = local_sidereal_time - APlocal_sidereal_time->getValueDouble ();
	if (diff_loc_time >= 180.)
	{
		diff_loc_time -=360. ;
	}
	else if ((diff_loc_time) <= -180.)
	{
		diff_loc_time += 360. ;
	}
	if (fabs( diff_loc_time) > (1./8.) )
	{ // 30 time seconds
		logStream (MESSAGE_DEBUG) << "APGTO::info  local sidereal time, calculated time " 
			<< local_sidereal_time << " mount: "
			<< APlocal_sidereal_time->getValueDouble() 
			<< " difference " << diff_loc_time <<sendLog;

		logStream (MESSAGE_DEBUG) << "APGTO::info ra " << getTelRa() << " dec " << getTelDec() << " alt " <<   APAltAz->getAlt() << " az " << APAltAz->getAz()  <<sendLog;

		char date_time[256];
    struct ln_date utm;
    struct ln_zonedate ltm;
    ln_get_date_from_sys( &utm) ;
    ln_date_to_zonedate(&utm, &ltm, -1 * timezone + 3600 * daylight); // Adds "only" offset to JD and converts back (see DST below)

    if(( ret= setAPLocalTime(ltm.hours, ltm.minutes, (int) ltm.seconds) < 0)) {
      logStream (MESSAGE_ERROR) << "APGTO::info setting local time failed" << sendLog;
      return -1;
    }
    if (( ret= setCalenderDate(ltm.days, ltm.months, ltm.years) < 0) ) {
      logStream (MESSAGE_ERROR) << "APGTO::info setting local date failed" << sendLog;
      return -1;
    }
    sprintf( date_time, "%4d-%02d-%02dT%02d:%02d:%02d", ltm.years, ltm.months, ltm.days, ltm.hours, ltm.minutes, (int) ltm.seconds) ;
    logStream (MESSAGE_DEBUG) << "APGTO::info local date and time set :" << date_time << sendLog ;

    // read the coordinates times again
    if (tel_read_ra () || tel_read_dec ())
      return -1;
    if (tel_read_azimuth () || tel_read_altitude ())
      return -1;
    if(( ret= tel_read_local_time()) != 0)
      return -1 ;
    if(( ret= tel_read_sidereal_time()) != 0)
      return -1 ;
    diff_loc_time = local_sidereal_time- APlocal_sidereal_time->getValueDouble() ;

    logStream (MESSAGE_DEBUG) << "APGTO::info ra " << getTelRa() << " dec " << getTelDec() << " alt " <<   APAltAz->getAlt() << " az " << APAltAz->getAz()  <<sendLog;

    logStream (MESSAGE_DEBUG) << "APGTO::info  local sidereal time, calculated time " 
			      << local_sidereal_time << " mount: "
			      << APlocal_sidereal_time->getValueDouble() 
			      << " difference " << diff_loc_time <<sendLog;
  } 
  // The mount unexpectedly starts to track, stop that
  if( ! mount_tracking->getValueBool()) {
    double RA= getTelRa() ;
    JD= ln_get_julian_from_sys ();
    local_sidereal_time= fmod((ln_get_mean_sidereal_time( JD) * 15. + lng + 360.), 360.);  // longitude positive to the East
    HA= fmod( local_sidereal_time- RA+ 360., 360.) ;

    double diff= HA -on_zero_HA ;
    // Shortest path
    if( diff >= 180.) {
      diff -=360. ;
    } else if( diff <= -180.) {
      diff += 360. ;
    }
    if( fabs( diff) > DIFFERENCE_MAX_WHILE_NOT_TRACKING) {
      logStream (MESSAGE_INFO) << "APGTO::info HA changed while mount is not tracking." << sendLog;
      notMoveCupola() ;
      if ( selectAPTrackingMode(TRACK_MODE_ZERO) < 0 ) {
	logStream (MESSAGE_ERROR) << "APGTO::info setting tracking mode ZERO failed." << sendLog;
	return -1;
      }
    }
  }
  return TelLX200::info ();
}
int
APGTO::idle ()
{
  // make sure that the checks in ::info are done
  info() ;

  return TelLX200::idle() ;
}

int APGTO::willConnect (Rts2Address * in_addr)
{
	if (ccdDevice && in_addr->getType () == DEVICE_TYPE_CCD)
	{
		logStream (MESSAGE_INFO) << "APGTO::willConnect to DEVICE_TYPE_CCD: "<< ccdDevice << sendLog;
		return 1;
	}
	return TelLX200::willConnect (in_addr);
}

double APGTO::siderealTime()
{
	double JD  = ln_get_julian_from_sys ();
	double lng = telLongitude->getValueDouble ();
	return fmod((ln_get_mean_sidereal_time( JD) * 15. + lng + 360.), 360.);  // longitude positive to the East
}

int APGTO::checkSiderealTime( double limit) 
{
  int ret ;
  int error= -1 ;
  // Check if the sidereal time read from the mount is correct 
  double local_sidereal_time= siderealTime() ;

  if(( ret= tel_read_sidereal_time()) != 0) {
    error = ERROR_IN_INFO ;
    logStream (MESSAGE_ERROR) << "APGTO::info could not retrieve sidereal time  " << sendLog;
  }
  logStream (MESSAGE_DEBUG) << "APGTO::checkSiderealTime  local sidereal time, calculated time " 
			    << local_sidereal_time << " mount: "
			    << APlocal_sidereal_time->getValueDouble() 
			    << " difference " << local_sidereal_time- APlocal_sidereal_time->getValueDouble()<<sendLog;
	
  if( fabs(local_sidereal_time- APlocal_sidereal_time->getValueDouble()) > limit ) { // usually 30 time seconds
    logStream (MESSAGE_INFO) << "APGTO::checkSiderealTime AP sidereal time off by " << local_sidereal_time- APlocal_sidereal_time->getValueDouble() << sendLog;
    return -1 ;
  } 
  return 0 ;
}

int APGTO::setBasicData()
{
	int ret ;
	struct ln_date utm;
	struct ln_zonedate ltm;
	if ((ret = tel_set_slew_rate(SLEW_RATE_0600)) < 0)
	{
		/* slew rate 2 = 600x, this the slowest */
		logStream (MESSAGE_ERROR) << "APGTO::setBasicData setting slew rate failed." << sendLog;
		return -1;
	}
	APslew_rate->setValueInteger(600);

	if ((ret = tel_set_move_rate(MOVE_RATE_000100)) < 0)
	{ /* move rate MOVE_RATE_000100 = 1x */
		logStream (MESSAGE_ERROR) << "APGTO::setBasicData setting tracking mode sidereal failed." << sendLog;
		return -1;
	}
	APmove_rate->setValueInteger(1);
	//if the sidereal time read from the mount is correct then consider it as a warm start 
	if (checkSiderealTime( 1./60.) == 0)
	{
		logStream (MESSAGE_DEBUG) << "APGTO::setBasicData performing warm start due to correct sidereal time" << sendLog;
		return 0 ;
	}

	logStream (MESSAGE_DEBUG) << "APGTO::setBasicData performing cold start due to incorrect sidereal time" << sendLog;

	if (setAPClearBuffer() < 0)
	{
		logStream (MESSAGE_ERROR) << "APGTO::setBasicData clearing the buffer failed" << sendLog;
		return -1;
	}
	if (setAPLongFormat() < 0)
	{
		logStream (MESSAGE_ERROR) << "APGTO::setBasicData setting long format failed" << sendLog;
		return -1;
	}
	if (setAPBackLashCompensation(0,0,0) < 0)
	{
		logStream (MESSAGE_ERROR) << "APGTO::setBasicData setting back lash compensation failed" << sendLog;
		return -1;
	}
	ln_get_date_from_sys( &utm) ;
	ln_date_to_zonedate(&utm, &ltm, -1 * timezone + 3600 * daylight); // Adds "only" offset to JD and converts back (see DST below)

	if (setAPLocalTime(ltm.hours, ltm.minutes, (int) ltm.seconds) < 0)
	{
		logStream (MESSAGE_ERROR) << "APGTO::setBasicData setting local time failed" << sendLog;
		return -1;
	}
	if (setCalenderDate(ltm.days, ltm.months, ltm.years) < 0)
	{
		logStream (MESSAGE_ERROR) << "APGTO::setBasicData setting local date failed" << sendLog;
		return -1;
	}
	// RTS2 counts positive to the East, AP positive to the West
	double APlng = fmod (360. - telLongitude->getValueDouble(), 360.0);
	if (setAPSiteLongitude( APlng) < 0)
	{ // AP mount: positive and only to the to west 
		logStream (MESSAGE_ERROR) << "APGTO::setBasicData setting site coordinates failed" << sendLog;
		return -1;
	}
	if (setAPSiteLatitude( telLatitude->getValueDouble()) < 0)
	{
		logStream (MESSAGE_ERROR) << "APGTO::setBasicData setting site coordinates failed" << sendLog;
		return -1;
	}
	if (setAPUTCOffset((int) (timezone / 3600) - daylight) < 0)
	{
		logStream (MESSAGE_ERROR) << "APGTO::setBasicData setting AP UTC offset failed" << sendLog;
		return -1;
	}

	logStream (MESSAGE_DEBUG) << "APGTO::setBasicData performing a cold start" << sendLog;
	if (setAPUnPark() < 0)
	{
		logStream (MESSAGE_ERROR) << "APGTO::setBasicData unparking failed" << sendLog;
		return -1;
	}
	logStream (MESSAGE_DEBUG) << "APGTO::setBasicData unparking (cold start) successful" << sendLog;
  
	return 0 ;
}
// further discussion with Petr required:
// int APGTO::changeMasterState (int new_state)
// {
// 	// do NOT park us during day..
// 	if (
// 		   ((new_state & SERVERD_STATUS_MASK) == SERVERD_SOFT_OFF)
// 		|| ((new_state & SERVERD_STATUS_MASK) == SERVERD_HARD_OFF)
// 		|| ((new_state & SERVERD_STANDBY_MASK) && standbyPark))
// 	{
// 		if ((getState () & TEL_MASK_MOVING) == 0)
// 		  startPark ();
// 	}

// 	if (blockOnStandby->getValueBool () == true)
// 	{
// 		if ((new_state & SERVERD_STATUS_MASK) == SERVERD_SOFT_OFF
// 		  || (new_state & SERVERD_STATUS_MASK) == SERVERD_HARD_OFF
// 		  || (new_state & SERVERD_STANDBY_MASK))
// 			blockMove->setValueBool (true);
// 		else
// 			blockMove->setValueBool (false);
// 	}
// 	// do not call Telescope::changeMasterState()
// 	return Rts2Device::changeMasterState (new_state);
// }
/*!
 * Init mount, connect on given apgto_fd.
 *
 *
 * @return 0 on succes, -1 & set errno otherwise
 */
int APGTO::init ()
{
	int status;
	on_set_HA= 0.;
	force_start= false ;
	slew_state->setValueBool(false) ;
	time(&slew_start_time) ;
	status = TelLX200::init ();
  
	if (status)
		return status;

	tzset ();

	logStream (MESSAGE_DEBUG) << "timezone " << timezone << " daylight " << daylight << sendLog;

	logStream (MESSAGE_DEBUG) << "APGTO::init RS 232 initialization complete" << sendLog;
	return 0;
}

/*!
 * Reads information from controller
 */
int APGTO::initValues ()
{
  int ret = -1 ;
  int flip= -1 ;
  if (ccdDevice)
	addConstValue ("CCD", ccdDevice, "ccd camera device name to monitor for data tacking timeouts");

  strcpy (telType, "APGTO");

  Rts2Config *config = Rts2Config::instance ();
  ret = config->loadFile ();
  if (ret)
    return -1;

  telLongitude->setValueDouble (config->getObserver ()->lng);
  telLatitude->setValueDouble (config->getObserver ()->lat);
  telAltitude->setValueDouble (config->getObservatoryAltitude ());

  if(( ret= setBasicData()) != 0)
    return -1 ;
  if (tel_read_longitude () || tel_read_latitude ())
    return -1;
  if (tel_read_azimuth () || tel_read_altitude ())
    return -1;
  if (getAPVersionNumber() != 0 )
    return -1 ; ;
  if (getAPUTCOffset() != 0)
    return -1 ;
  if (tel_read_local_time() != 0)
    return -1 ;
  if(( ret= tel_read_sidereal_time()) != 0)
    return -1 ;
  if (tel_read_ra () || tel_read_dec ())
    return -1;

  // Check if the sidereal time read from the mount is correct 
  if( (ret= checkSiderealTime( 1./120.)) != 0 ) {

    logStream (MESSAGE_ERROR) << "initValues sidereal time larger than 1./120, exiting" << sendLog;
    //exit (1) ; // do not go beyond, at least for the moment
  }
  logStream (MESSAGE_DEBUG) << "APGTO::initValues ra "
                            << getTelRa() << " dec " 
                            << getTelDec ()
                            << sendLog;


  if(( ret= tel_read_declination_axis()) != 0)
    return -1 ;

  // West: HA + Pi/2 = direction where the declination axis points
  // East: HA - Pi/2
  if( !( strcmp( "West", DECaxis_HAcoordinate->getValue()))) {
    flip = 1 ;
  } else if( !( strcmp( "East", DECaxis_HAcoordinate->getValue()))) {
    flip= 0 ;
  } else {
    logStream (MESSAGE_ERROR) << "APGTO::initValues could not retrieve angle (declination axis, hour axis), exiting" << sendLog;
    exit(1) ;
  }
  telFlip->setValueInteger (flip);

  if (tel_read_ra () || tel_read_dec ())
    return -1;

  // check if the assumption "mount correctly parked" holds true
  // if not true block any sync and slew operations.
  //
  // This is only a formal test based on the idea that the
  // mount has a park position and was parked by RTS2. If the
  // mount is not at this position assume that position is
  // undefined or incorrect. A visual check is required.
  int tracking_mode= TRACK_MODE_SIDEREAL ;  
  if( assume_parked->getValueBool()) {

    struct ln_equ_posn object;
    struct ln_equ_posn park_position;
    object.ra = fmod((getTelRa() + 360.), 360.);
    object.dec= fmod(getTelDec(), 90.) ;

    park_position.ra = fmod(( PARK_POSITION_RA + siderealTime()) + 360., 360.);
    park_position.dec= PARK_POSITION_DEC  ;

    double diff_ra = fabs(fmod(( park_position.ra - object.ra) + 360., 360.));
    
    if( diff_ra >= 180.) {
      diff_ra -=360. ;
    } else if( diff_ra <= -180.) {
      diff_ra += 360. ;
    }

    double diff_dec= fabs(( park_position.dec- object.dec));
    int block= 0 ;
    if(( diff_ra < PARK_POSITION_DIFFERENCE_MAX) && ( diff_dec < PARK_POSITION_DIFFERENCE_MAX)) {
      if( !( strcmp( PARK_POSITION_ANGLE, DECaxis_HAcoordinate->getValue()))) {
	// ok
      } else {
	block= 1 ;
      }
    } else {
      block= 1 ;
    }
    if( block) {
      tracking_mode= TRACK_MODE_ZERO ; 
      block_sync_move->setValueBool(true) ;
      logStream (MESSAGE_ERROR) << "APGTO::initValues block any sync and slew opertion due to wrong initial position, RA:"<< object.ra << " dec: "<< object.dec << " diff ra: "<< diff_ra << "diff dec: " << diff_dec << sendLog;
    } 
  }
  // force_start knocks out APGTO::tel_check_declination_axis ()
  // this is necessary in case the mount shows a wrong
  // angle of HA relative to DEC axis. Before any operation occurs 
  // the mount must be synced **manually**.
  if( force_start== true ) {
    tracking_mode= TRACK_MODE_ZERO ; 
    block_sync_move->setValueBool(true) ; // don't get me twice!
    logStream (MESSAGE_ERROR) << "APGTO::initValues set  BLOCK_SYNC_MOVE to false due to force_start== true, sync manually" << sendLog;
  }
  if(( ret= selectAPTrackingMode(tracking_mode)) < 0 ) { 
    logStream (MESSAGE_ERROR) << "APGTO::initValues setting tracking mode:"<< tracking_mode << " failed." << sendLog;
    return -1;
  }
  move_state = NOTMOVE;
  slew_state->setValueBool(false) ;
  return TelLX200::initValues ();
}

APGTO::~APGTO (void)
{
	delete serConn;
}

int APGTO::processOption (int in_opt)
{
	switch (in_opt)
	{
		case OPT_APGTO_ASSUME_PARKED:
			assume_parked->setValueBool(true);
			break;
		case OPT_APGTO_FORCE_START:
			force_start= true;
			break;
		case OPT_APGTO_CCD_DEVICE:
			ccdDevice = optarg;
			break;
		case OPT_APGTO_KEEP_HORIZON:
			avoidBellowHorizon->setValueBool (true);
			break;
		default:
			return TelLX200::processOption (in_opt);
	}
	return 0;
}

APGTO::APGTO (int in_argc, char **in_argv):TelLX200 (in_argc,in_argv)
{
	createValue (fixedOffsets,             "FIXED_OFFSETS", "fixed (not reseted) offsets, set after first sync", RTS2_VALUE_WRITABLE | RTS2_DT_DEG_DIST);
	fixedOffsets->setValueRaDec (0, 0);
	
	createValue (block_sync_move,          "BLOCK_SYNC_MOVE", "true inhibits any sync, slew", false, RTS2_VALUE_WRITABLE);
	createValue (slew_state,               "SLEW",        "true: mount is slewing",           false);
	createValue (mount_tracking,           "TRACKING",    "true: mount is tracking",          false);
	createValue (transition_while_tracking,"TRANSITION",  "transition while tracking",        false);
	createValue (DECaxis_HAcoordinate,     "DECXHA",      "DEC axis HA coordinate, West/East",false);
	createValue (APslew_rate,              "APSLEWRATE",  "AP slew rate (1200, 900, 600)",    false, RTS2_VALUE_WRITABLE);
	createValue (APmove_rate,              "APMOVERATE",  "AP move rate (600, 64, 12, 1)",    false, RTS2_VALUE_WRITABLE);
	createValue (APlocal_sidereal_time,    "APLOCSIDTIME","AP mount local sidereal time",     true,  RTS2_DT_RA);
	createValue (APlocal_time,             "APLOCTIME",   "AP mount local time",              true,  RTS2_DT_RA);
	createValue (APAltAz,                  "APALTAZ",     "AP mount Alt/Az[deg]",             true,  RTS2_DT_DEGREES | RTS2_VALUE_WRITABLE, 0);
	createValue (APutc_offset,             "APUTCOFFSET", "AP mount UTC offset",              true,  RTS2_DT_RA);
	createValue (APfirmware,               "APVERSION",   "AP mount firmware revision",       true);
	createValue (APlongitude,              "APLONGITUDE", "AP mount longitude",               true,  RTS2_DT_DEGREES);
	createValue (APlatitude,               "APLATITUDE",  "AP mount latitude",                true,  RTS2_DT_DEGREES);
	createValue (assume_parked,            "ASSUME_PARKED", "true check initial position",    false);
	createValue (collision_detection,      "COLLILSION_DETECTION",   "true: mount stop if it collides", true, RTS2_VALUE_WRITABLE);
	createValue (avoidBellowHorizon, "AVOID_HORIZON", "avoid movements bellow horizon", true, RTS2_VALUE_WRITABLE);
	avoidBellowHorizon->setValueBool (false);

	collision_detection->setValueBool(true) ;
	slew_state->setValueBool(false) ;
	block_sync_move->setValueBool(false) ;
	transition_while_tracking->setValueBool(false) ;
	assume_parked->setValueBool(false);

	ccdDevice = NULL;

	addOption (OPT_APGTO_ASSUME_PARKED, "parked",      0, "assume a regularly parked mount");
	addOption (OPT_APGTO_FORCE_START,   "force_start", 0, "start with wrong declination axis orientation");
	addOption (OPT_APGTO_CCD_DEVICE,    "ccd_device",  1, "ccd camera device name to monitor for data tacking timeouts");
	addOption (OPT_APGTO_KEEP_HORIZON,  "avoid-horizon", 0, "avoid movements bellow horizon");
}

int main (int argc, char **argv)
{
	APGTO device = APGTO (argc, argv);
	return device.run ();
}

