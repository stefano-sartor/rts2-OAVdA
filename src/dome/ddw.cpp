/* 
 * Driver for Digital DomeWorks controller.
 * Copyright (C) 2018 Petr Kubanek
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

#include "cupola.h"
#include "connection/serial.h"
#include "math.h"

using namespace rts2dome;

namespace rts2dome
{

typedef enum {IDLE, CLOSING, OPENING, MOVING, HOMING} cmdType;

class DDW:public Cupola
{
	public:
		DDW (int argc, char **argv);
		virtual ~DDW ();

		virtual int idle ();

		virtual int commandAuthorized (rts2core::Connection * conn);

	protected:
		virtual int processOption (int opt);
		virtual int initHardware ();

		virtual int info ();

		virtual int startOpen ();
		virtual long isOpened ();
		virtual int endOpen ();

		virtual int startClose ();
		virtual long isClosed ();
		virtual int endClose ();

		virtual int moveStart ();
		virtual long isMoving ();
		virtual int moveEnd ();

		virtual double getSlitWidth (double alt);

		long getAzDomeOffset(double az);
		long getDomeAzFromTargetAz(double targetAz);
		long getTargetAzFromDomeAz(double domeAz);

		/// remove this once the mount is implemented
		long getTargetAlt() { return 60; }
		
	private:
		rts2core::ConnSerial *sconn;
		const char *devFile;

		int parseInfo (bool V = true);
		int executeCmd (const char *cmd, cmdType newCmd);
		long inProgress (bool opening);
		cmdType cmdInProgress;

		rts2core::ValueInteger *z;
		rts2core::ValueInteger *shutter;
		rts2core::ValueInteger *dticks;

		void setAzimuthTicks(int adaz) { setCurrentAz(getTargetAzFromDomeAz(359*(double)(adaz)/(double)(dticks->getValueInteger())), true); }

		long AzDomeOffsetCoeff[2][3];
		
};

}

DDW::DDW (int argc, char **argv):Cupola (argc, argv)
{
	sconn = NULL;
	devFile = "/dev/ttyUSB0";
	cmdInProgress = IDLE;

	addOption('f', NULL, 1, "path to device, usually /dev/ttyUSB0");

	setIdleInfoInterval(5);

	createValue(z, "Z", "Z progress value", false);
	createValue(shutter, "shutter", "DDW reported shutter state", false);
	shutter->setValueInteger(0);

	createValue(dticks, "dticks", "number of azimuth ticks", false);

	// Azimuth Dome offset coefficients derived for Lowell TiMo
	// using offset(az) = [1]*sin(az+[2])+[3]
	AzDomeOffsetCoeff[1][1] = 4.2772;
	AzDomeOffsetCoeff[1][2] = -21.510;
	AzDomeOffsetCoeff[1][3] = 11.053;	
	AzDomeOffsetCoeff[2][1] = 8.247;
	AzDomeOffsetCoeff[2][2] = -4.908;
	AzDomeOffsetCoeff[2][3] = 20.234;
	
}

DDW::~DDW ()
{
	delete sconn;
}

int DDW::processOption (int opt)
{
	switch (opt)
	{
		case 'f':
			devFile = optarg;
			break;
		default:
			return Cupola::processOption(opt);
	}
	return 0;
}

int DDW::initHardware ()
{
    	sconn = new rts2core::ConnSerial(devFile, this, rts2core::BS9600, rts2core::C8, rts2core::NONE, 25);
    	sconn->setDebug(getDebug ());
    	sconn->init();

	sconn->flushPortIO();

	info();

	return 0;
}

int DDW::info ()
{
	if (cmdInProgress != IDLE)
		return 0;

	sconn->writePort("GINF", 4);
	parseInfo ();
	return Cupola::info();
}

int DDW::idle ()
{
	if (cmdInProgress == HOMING)
	{
		if (inProgress(false) < 0)
			setIdleInfoInterval(5);
	}
	return Cupola::idle();
}

int DDW::commandAuthorized (rts2core::Connection * conn)
{
	if (conn->isCommand ("stop"))
	{
		sconn->writePort("GINF", 4);
		return 0;	
	}
	else if (conn->isCommand ("home"))
	{
		if (cmdInProgress != IDLE && cmdInProgress != OPENING && cmdInProgress != MOVING)
			return -1;
		int rete = executeCmd("GHOM", HOMING);
		if (rete < 0)
		{
			cmdInProgress = IDLE;
			return -1;
		}
		setIdleInfoInterval(0.05);
		return 0;
	}	
	return Cupola::commandAuthorized (conn);
}

int DDW::startOpen ()
{
	if (shutter->getValueInteger() == 2)
		return 0;
	if (cmdInProgress != IDLE)
	{
		sconn->writePort("GINF", 4);
		parseInfo ();
		sconn->flushPortIO();
		char buf[200];
		sconn->readPort(buf, 200);
	}
	int rete = executeCmd ("GOPN", OPENING);
	// when homing is needed, R or L is returned
	switch (rete)
	{
		case 'O':
		case 'R':
		case 'L':
			shutter->setValueInteger(0);
			return 0;
		default:
			cmdInProgress = IDLE;
			return DEVDEM_E_HW;
	}
}

long DDW::isOpened ()
{
	if (cmdInProgress == MOVING)
	{
		return shutter->getValueInteger () == 2 ? -2 : 0;
	}
	if (cmdInProgress != OPENING)
	{
		if (info ())
			return -1;
		return shutter->getValueInteger () == 2 ? -2 : 0;
	}
	return inProgress (false);
}

int DDW::endOpen ()
{
	return 0;
}

int DDW::startClose ()
{
	if (cmdInProgress == CLOSING ||
		(cmdInProgress != OPENING && shutter->getValueInteger() == 1))
		return 0;
	// stop any command in progress
	if (cmdInProgress != IDLE)
	{
		sconn->writePort("GINF", 4);
		parseInfo ();
		sconn->flushPortIO();
		char buf[200];
		sconn->readPort(buf, 200);
	}

	int rete = executeCmd ("GCLS", CLOSING);
	// when homing is needed, R or L is returned
	switch (rete)
	{
		case 'C':
		case 'R':
		case 'L':
			shutter->setValueInteger(0);
			return 0;
		default:
			cmdInProgress = IDLE;
			return DEVDEM_E_HW;
	}
}

long DDW::isClosed ()
{
	if (cmdInProgress != CLOSING)
	{
		if (info ())
			return -1;
		return shutter->getValueInteger () == 1 ? -2 : 0;
	}
	return inProgress (false);
}

int DDW::endClose ()
{
	return 0;
}

int DDW::moveStart ()
{
	// stop any command in progress
	if (cmdInProgress == MOVING)
	{
		sconn->writePort("GINF", 4);
		parseInfo ();
		sconn->flushPortIO();
		char buf[200];
		sconn->readPort(buf, 200);
	}
	if (cmdInProgress == OPENING || cmdInProgress == CLOSING)
	{
		logStream(MESSAGE_ERROR) << "dome busy; ignoring move command " << cmdInProgress << sendLog;
		return DEVDEM_E_HW;
	}
	
	char azbuf[5];
	sconn->flushPortIO();

	if (getDomeAzFromTargetAz(getTargetAz()) < 0)
		snprintf(azbuf, 5, "G%03d",
				 (int) round(360+getDomeAzFromTargetAz(getTargetAz())));
	else
		snprintf(azbuf, 5, "G%03d",
				 (int) round(getDomeAzFromTargetAz(getTargetAz())));
  
		
	int azret = executeCmd(azbuf, MOVING);
	if (azret == 'R' || azret == 'L')
	{
		return Cupola::moveStart();
	}
	if (azret == 'V')
	{
		parseInfo();
		cmdInProgress = IDLE;
		return 0;
	}
	logStream(MESSAGE_WARNING) << "unknow azimuth character " << (char) azret << sendLog;
	cmdInProgress = IDLE;
	return DEVDEM_E_HW;
}

long DDW::isMoving ()
{
	if (cmdInProgress != MOVING)
		return -1;
	return inProgress(false);
}

int DDW::moveEnd ()
{
	return 0;
}

double DDW::getSlitWidth (double alt)
{
	return 5;
}

int DDW::parseInfo (bool V)
{
	shutter->setValueInteger(0);

	char buf[200];
	char *bp = buf;
	memset (buf, 0, sizeof(buf));

	if (sconn->readPort (buf, 200, '\r') == -1)
		return -1;

	int ver;
	int _dticks;
	int home1;
	int coast;
	int adaz;
	int slave;
	int _shutter;
	int dsr_status;
	int home;
	int htick_ccl;
	int htick_clk;
	int upins;
	int weaage;
	int winddir;
	int windspd;
	int temp;
	int humid;
	int wetness;
	int snow;
	int wind_peak;

	int scopeaz;
	int intdz;
	int intoff;

	if (V)
	{
		if (buf[0] != 'V')
			return -1;
		bp++;
	}

	int sret = sscanf(bp, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r",
		&ver, &_dticks, &home1, &coast, &adaz,
		&slave, &_shutter, &dsr_status, &home, &htick_ccl,
		&htick_clk, &upins, &weaage, &winddir, &windspd,
		&temp, &humid, &wetness, &snow, &wind_peak,
		&scopeaz, &intdz, &intoff);

	if (sret != 23)
	{
		logStream(MESSAGE_WARNING) << "cannot parse info string " << sret << sendLog;
		return -1;
	}

	switch (_shutter)
	{
		case 1:
			maskState(DOME_DOME_MASK, DOME_CLOSED, "dome closed");
			break;
		case 2:
			maskState(DOME_DOME_MASK, DOME_OPENED, "dome opened");
			break;
	}

	shutter->setValueInteger(_shutter);

	dticks->setValueInteger(_dticks);

	setAzimuthTicks(adaz);

	usleep (USEC_SEC * 1.5);

	return 0;
}

int DDW::executeCmd (const char *cmd, cmdType newCmd)
{
	if (sconn->writePort (cmd, 4) != 0)
		return -1;
	char repl;
	if (sconn->readPort (repl) != 1)
		return -1;
	cmdInProgress = newCmd;
	return repl;
}

long DDW::inProgress (bool opening)
{
	char rc;
	if (!sconn->readPort(rc))
		return -1;
	switch (rc)
	{
		case 'S':
		case 'T':
			return 100;
		case 'O':
			if (cmdInProgress == OPENING)
				return 100;
			break;
		case 'C':
			if (cmdInProgress == CLOSING)
				return 100;
			break;
		case 'P':
		{
			char pos[5];
			if (sconn->readPort(pos, 4, '\r') == -1)
				return -1;
			setAzimuthTicks(atoi(pos));
			return 100;
		}
		case 'Z':
		{
			char zbuf[4];
			if (sconn->readPort(zbuf, 4, '\r') == -1)
				return -1;
			z->setValueInteger(atoi(zbuf));
			sendValueAll(z);
			return 100;
		}
		case 'V':
		{
			cmdInProgress = IDLE;
			parseInfo (false);
			return -2;
		}
	}
	logStream(MESSAGE_WARNING) << "unknow character during command execution: " << (char) rc << sendLog;
	cmdInProgress = IDLE;
	return -1;
}

long DDW::getAzDomeOffset(double az)
{
	if (getTargetAlt() < 35)
		return AzDomeOffsetCoeff[1][1]*
			sin((az+AzDomeOffsetCoeff[1][2])/180*M_PI)+
			AzDomeOffsetCoeff[1][3];
	else
		return AzDomeOffsetCoeff[2][1]*
			sin((az+AzDomeOffsetCoeff[2][2])/180*M_PI)+
			AzDomeOffsetCoeff[2][3];
}

long DDW::getDomeAzFromTargetAz(double targetAz)
{
	return targetAz - getAzDomeOffset(targetAz);
}

long DDW::getTargetAzFromDomeAz(double domeAz)
{
	return domeAz + getAzDomeOffset(domeAz);
}

int main(int argc, char **argv)
{
	DDW device(argc, argv);
	return device.run();
}
