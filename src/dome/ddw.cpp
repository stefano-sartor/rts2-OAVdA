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

using namespace rts2dome;

namespace rts2dome
{

class DDW:public Cupola
{
	public:
		DDW (int argc, char **argv);
		virtual ~DDW ();

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

		virtual double getSlitWidth (double alt);

	private:
		rts2core::ConnSerial *sconn;
		const char *devFile;

		int parseInfo (bool V = true);
		int executeCmd (const char *cmd);
		long inProgress (bool opeing);
		bool cmdInProgress;

		rts2core::ValueInteger *z;
};

}

DDW::DDW (int argc, char **argv):Cupola (argc, argv)
{
	sconn = NULL;
	devFile = "/dev/ttyUSB0";
	cmdInProgress = false;

	addOption ('f', NULL, 1, "path to device, usually /dev/ttyUSB0");

	setIdleInfoInterval(5);

	createValue (z, "Z", "Z progress value", false);
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

	return 0;
}

int DDW::info ()
{
	if (cmdInProgress)
		return 0;

	sconn->writePort ("GINF", 4);
	parseInfo ();
	return Cupola::info();
}

int DDW::startOpen ()
{
	return executeCmd ("GOPN") == 'O' ? 0 : -1;
}

long DDW::isOpened ()
{
	return inProgress (true);
}

int DDW::endOpen ()
{
	return 0;
}

int DDW::startClose ()
{
	if (cmdInProgress)
		return 0;
	return executeCmd ("GCLS") == 'C' ? 0 : -1;
}

long DDW::isClosed ()
{
	return inProgress (false);
}

int DDW::endClose ()
{
	return 0;
}

double DDW::getSlitWidth (double alt)
{
	return 5;
}

int DDW::parseInfo (bool V)
{
	char buf[200];
	char *bp = buf;
	memset (buf, 0, sizeof(buf));

	if (sconn->readPort (buf, 200, '\r'))
		return -1;

	int ver;
	int dticks;
	int home1;
	int coast;
	int adaz;
	int slave;
	int shutter;
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
		&ver, &dticks, &home1, &coast, &adaz,
		&slave, &shutter, &dsr_status, &home, &htick_ccl,
		&htick_clk, &upins, &weaage, &winddir, &windspd,
		&temp, &humid, &wetness, &snow, &wind_peak,
		&scopeaz, &intdz, &intoff);

	if (sret != 23)
	{
		logStream(MESSAGE_WARNING) << "cannot parse info string " << sret << sendLog;
		return -1;
	}

	usleep (USEC_SEC * 1.5);

	return 0;
}

int DDW::executeCmd (const char *cmd)
{
	if (sconn->writePort (cmd, 4) != 0)
		return -1;
	char repl;
	if (sconn->readPort (repl) != 1)
		return -1;
	cmdInProgress = true;
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
			return 100;
		case 'P':
		{
			char pos[5];
			if (sconn->readPort (pos, 4, '\r') == -1)
				return -1;
			return 100;
		}
		case 'Z':
		{
			char zbuf[4];
			if (sconn->readPort (zbuf, 4, '\r') == -1)
				return -1;
			z->setValueInteger(atoi(zbuf));
			sendValueAll(z);
			return 100;
		}
		case 'V':
		{
			cmdInProgress = false;
			parseInfo (false);
			return -2;
		}
	}
	logStream(MESSAGE_WARNING) << "unknow character during command execution: " << (char) rc << sendLog;
	return -1;
}

int main(int argc, char **argv)
{
	DDW device(argc, argv);
	return device.run();
}
