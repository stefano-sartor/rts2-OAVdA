/**
 * Standalone version of APM filter wheel (ESA TestBed telescope)
 * Copyright (C) 2014 Standa Vitek <standa@vitkovi.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <unistd.h>

#include "teld.h"
#include "configuration.h"
#include "status.h"

#define MAXDATASIZE 1500

namespace rts2teld
{

class PlaneWave:public Telescope
{
	public:
		PlaneWave (int in_argc, char **in_argv);
		virtual ~PlaneWave(void);
		virtual int init();
		virtual int initValues();
		virtual int info();		
	  	virtual int startResync();
		virtual int isMoving();
		virtual int stopMove();

		virtual int startPark();
		virtual int isParking();
		virtual int endPark();
	protected:
		virtual int processOption(int in_opt);

	private:
        // connection parameters
		int sock;
		HostString *host;
		struct sockaddr_in bind_addr;

		// properties
		enum { NOTMOVE, MOVE_HOME, MOVE_REAL } move_state;

		// functions
		char* send_command(const char *command);
		// rts2core::ConnAPM *apm;
/*		int sendUDPMessage (const char * in_message);
		int filterNum;
		int filterSleep;
*/
};

};

using namespace rts2teld;

PlaneWave::PlaneWave(int argc, char **argv):Telescope(argc, argv)
{
	addOption ('e', NULL, 1, "IP and port (separated by :)");
}

PlaneWave::~PlaneWave(void)
{
	close(sock);
}


int PlaneWave::processOption(int in_opt)
{
	switch (in_opt)
	{
		case 'e':
			host = new HostString (optarg, "1000");
			break;
	    // case 's':
		// 	filterSleep = atoi (optarg);
		// 	break;
        //         default:
        //                 return Filterd::processOption (in_opt);
	}
	return 0;
}


int PlaneWave::init()
{
	int status, ret;

	status = Telescope::init();
	if (status)
		return status;

	sock = socket(PF_INET, SOCK_STREAM, 0);
	if (sock < 0)
	{
 		logStream (MESSAGE_ERROR) <<
			"PlaneWave::init unable to create socket: " <<
			strerror (errno) << sendLog;
		return -1;
	}

	/* setup bind address */
	bind_addr.sin_family = PF_INET; // host byte order
	bind_addr.sin_addr.s_addr = inet_addr("10.10.30.119");
	memset(bind_addr.sin_zero, '\0', sizeof bind_addr.sin_zero);

	/* telnet test server */
	//bind_addr.sin_addr.s_addr = inet_addr("127.0.0.1");	
    //bind_addr.sin_port = htons(7891); 

	/* L-500 mount */
	inet_aton("10.10.30.119", &(bind_addr.sin_addr));
	bind_addr.sin_port = htons(8877);

	// connect to PW4 server
    ret = connect(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
	if (ret)
	{
		logStream (MESSAGE_ERROR) << "Unable to connect to mount: " <<
			strerror (errno) << sendLog;
		return -1;
    }

	return 0;
}


int PlaneWave::initValues()
{
	int ret = -1;
	rts2core::Configuration *config = rts2core::Configuration::instance();

	ret = config->loadFile();
	if (ret)
	  return -1;

	setTelLongLat(config->getObserver()->lng, config->getObserver()->lat);
	setTelAltitude(config->getObservatoryAltitude());
	
	return Telescope::initValues();
}


char* PlaneWave::send_command(const char *command)
{
	int numbytes;
	char *buffin;
	buffin = (char*)malloc(MAXDATASIZE);

	logStream (MESSAGE_WARNING) << "sending command: " << command << sendLog;
	
    // send data
	numbytes = send(sock, command, strlen(command), 0);
	if (numbytes < 1)
	{
		logStream (MESSAGE_ERROR) << "Unable to send to mount: " <<
			strerror(errno) << sendLog;	
		//return -1;
		return "ERROR: Unable to send command to mount";
	}

	// receive results
	numbytes = recv(sock, buffin, MAXDATASIZE, 0);

	if (numbytes < 0)
	{
		logStream (MESSAGE_ERROR) << "Unable to read from mount: " <<
			strerror(errno) << sendLog;	
		//return -1;
		return "ERROR: Unable to read from mount";
	}

	return buffin;
}


int PlaneWave::info()
{
	char *retstr, *dummy;
	float ra_apparent_hours, dec_apparent_degs;
	float latitude, longitude, azimuth, altitude, lst;
	float field_angle_rate_degs_per_sec, field_angle_degs;
	int is_slewing, is_tracking, elevation, geometry;
	int ret, sret;

	
	logStream (MESSAGE_WARNING) << "get status" << sendLog;

	retstr = send_command("status\n");

	// first status query will be corrupt; repeat
	if (strlen(retstr) < 400)
	{
		logStream (MESSAGE_WARNING) << "shit failed" << sendLog;
		ret = info();
		return ret;
	}

	printf("%s\n", retstr);
	
	sret = sscanf(retstr, "mount.ra_apparent_hours=%f\nmount.dec_apparent_degs=%f\nmount.is_slewing=%d\nmount.is_tracking=%d\nmount.latitude=%f\nmount.longitude=%\nmount.elevation_meters=%d\nmount.azimuth=%f\naltitude=%f\nmount.lst=%f\nmount.geometry=%d\nmount.field_angle_degs=%f\nmount.field_angle_rate_degs_per_sec=%f\n%s\r",
				  &ra_apparent_hours, &dec_apparent_degs,
				  &is_slewing, &is_tracking, &latitude, &longitude,
				  &elevation, &azimuth, &altitude, &lst, &geometry,
				  &field_angle_degs, &field_angle_rate_degs_per_sec,
				  &dummy);

	logStream (MESSAGE_WARNING) << "ra " << ra_apparent_hours << floor(ra_apparent_hours)*15. +
			 (ra_apparent_hours-floor(ra_apparent_hours)) << sendLog;
	
	setTelRa(floor(ra_apparent_hours)*15. +
			 (ra_apparent_hours-floor(ra_apparent_hours)));	
	setTelDec(dec_apparent_degs);
	
	return Telescope::info();
}


// int EsaFilter::sendUDPMessage (const char * in_message)
// {
// 	unsigned int slen = sizeof (clientaddr);
//         char * status_message = (char *)malloc (20*sizeof (char));

// 	if (getDebug())
// 		logStream (MESSAGE_DEBUG) << "command to controller: " << in_message << sendLog;

// 	sendto (sock, in_message, strlen(in_message), 0, (struct sockaddr *)&servaddr,sizeof(servaddr));

// 	int n = recvfrom (sock, status_message, 20, 0, (struct sockaddr *) &clientaddr, &slen);

// 	if (getDebug())
// 		logStream (MESSAGE_DEBUG) << "reponse from controller: " << status_message << sendLog;

// 	if (n > 4)
// 	{
// 		if (status_message[0] == 'E')
// 		{
// 			// echo from controller "Echo: [cmd]"
// 			return 10;
// 		}
// 	}
// 	else if (n == 4)
// 	{	
// 		filterNum = status_message[3];
// 	}

// 	return 0;
// }

int PlaneWave::startResync()
{
	return 0;
}

int PlaneWave::isMoving()
{
	return 0;
}

int PlaneWave::stopMove()
{
	return 0;
}

int PlaneWave::startPark()
{
	return 0;
}

int PlaneWave::isParking()
{
	return 0;
}

int PlaneWave::endPark()
{
	return 0;
}

int main(int argc, char ** argv)
{
	PlaneWave device(argc, argv);
	return device.run();
}
