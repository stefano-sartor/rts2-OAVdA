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

		int parseInfo ();
};

}

DDW::DDW (int argc, char **argv):Cupola (argc, argv)
{
	sconn = NULL;
	devFile = "/dev/ttyUSB0";

	addOption ('f', NULL, 1, "path to device, usually /dev/ttyUSB0");

	setIdleInfoInterval(5);
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
    	sconn = new rts2core::ConnSerial(devFile, this, rts2core::BS9600, rts2core::C8, rts2core::NONE, 50);
    	sconn->setDebug(getDebug ());
    	sconn->init();

	return 0;
}

int DDW::info ()
{
	sconn->writePort ("GINF", 4);
	parseInfo ();
	return Cupola::info();
}

int DDW::startOpen ()
{
        return 0;
}

long DDW::isOpened ()
{
	return 0;
}

int DDW::endOpen ()
{
	return 0;
}

int DDW::startClose ()
{
    	return 0;
}

long DDW::isClosed ()
{
	return 2000;
}

int DDW::endClose ()
{
	return 0;
}

double DDW::getSlitWidth (double alt)
{
	return 5;
}

int DDW::parseInfo ()
{
	char buf[200];
	if (sconn->readPort (buf, 200, '\r'))
		return -1;

	logStream (MESSAGE_DEBUG) << buf << sendLog;

	return 0;
}

int main(int argc, char **argv)
{
	DDW device(argc, argv);
	return device.run();
}
