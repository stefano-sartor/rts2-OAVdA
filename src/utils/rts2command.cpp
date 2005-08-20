#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <iostream>
#include <fstream>

#include "rts2command.h"

using namespace std;

Rts2Command::Rts2Command (Rts2Block * in_owner)
{
  owner = in_owner;
  text = NULL;
  commandCond = NO_COND;
}

Rts2Command::Rts2Command (Rts2Block * in_owner, char *in_text)
{
  owner = in_owner;
  setCommand (in_text);
  commandCond = NO_COND;
}

int
Rts2Command::setCommand (char *in_text)
{
  text = new char[strlen (in_text) + 1];
  strcpy (text, in_text);
  return 0;
}

Rts2Command::~Rts2Command (void)
{
  delete[]text;
}

int
Rts2Command::send ()
{
  return connection->send (text);
}

int
Rts2Command::commandReturn (int status)
{
  if (connection)
    connection->commandReturn (this, status);
  if (status)
    return commandReturnFailed (status);
  return commandReturnOK ();
}

int
Rts2Command::commandReturnFailed (int status)
{
  return -1;
}

int
Rts2Command::commandReturnOK ()
{
  return -1;
}

/**************************************************************
 *
 * Rts2CommandSendKey implementation.
 *
 *************************************************************/

Rts2CommandSendKey::Rts2CommandSendKey (Rts2Block * in_master, int in_key):Rts2Command
  (in_master)
{
  key = in_key;
}

int
Rts2CommandSendKey::send ()
{
  char *command;
  asprintf (&command, "auth %i %i",
	    owner->getCentraldConn ()->getCentraldId (), key);
  setCommand (command);
  free (command);
  return Rts2Command::send ();
}

/**************************************************************
 * 
 * Rts2CommandAuthorize command
 *
 *************************************************************/

Rts2CommandAuthorize::Rts2CommandAuthorize (Rts2Block * in_master, const char *device_name):Rts2Command
  (in_master)
{
  char *
    command;
  asprintf (&command, "key %s", device_name);
  setCommand (command);
  free (command);
}

/**************************************************************
 *
 * Rts2 device commands
 * 
 *************************************************************/

Rts2CommandBinning::Rts2CommandBinning (Rts2Block * in_master, int binning_v,
					int binning_h):
Rts2Command (in_master)
{
  char *command;
  asprintf (&command, "binning 0 %i %i", binning_v, binning_h);
  setCommand (command);
  free (command);
}

Rts2CommandExposure::Rts2CommandExposure (Rts2Block * in_master,
					  Rts2DevClientCamera * in_camera,
					  exposureType exp_type,
					  float exp_time):
Rts2Command (in_master)
{
  char *command;
  asprintf (&command, "expose 0 %i %f", (exp_type == EXP_LIGHT ? 1 : 0),
	    exp_time);
  setCommand (command);
  free (command);
  camera = in_camera;
  commandCond = NO_EXPOSURE;
}

int
Rts2CommandExposure::commandReturnFailed (int status)
{
  camera->exposureFailed (status);
  return Rts2Command::commandReturnFailed (status);
}

Rts2CommandFilter::Rts2CommandFilter (Rts2Block * in_master, int filter):
Rts2Command (in_master)
{
  char *command;
  asprintf (&command, "filter %i", filter);
  setCommand (command);
  free (command);
}

Rts2CommandCenter::Rts2CommandCenter (Rts2Block * in_master, int chip, int width = -1, int height = -1):Rts2Command
  (in_master)
{
  char *
    command;
  asprintf (&command, "center %i %i %i", chip, width, height);
  setCommand (command);
  free (command);
}

Rts2CommandMove::Rts2CommandMove (Rts2Block * in_master,
				  Rts2DevClientTelescope * in_tel, double ra,
				  double dec):
Rts2Command (in_master)
{
  char *command;
  asprintf (&command, "move %lf %lf", ra, dec);
  setCommand (command);
  free (command);
  tel = in_tel;
}

int
Rts2CommandMove::commandReturnFailed (int status)
{
  tel->moveFailed (status);
  return Rts2Command::commandReturnFailed (status);
}


Rts2CommandResyncMove::Rts2CommandResyncMove (Rts2Block * in_master, Rts2DevClientTelescope * in_tel, double ra, double dec):
Rts2Command (in_master)
{
  char *command;
  asprintf (&command, "resync %lf %lf", ra, dec);
  setCommand (command);
  free (command);
  tel = in_tel;
}

int
Rts2CommandResyncMove::commandReturnFailed (int status)
{
  tel->moveFailed (status);
  return Rts2Command::commandReturnFailed (status);
}

Rts2CommandChange::Rts2CommandChange (Rts2Block * in_master, double ra, double dec):
Rts2Command (in_master)
{
  char *command;
  asprintf (&command, "change %lf %lf", ra, dec);
  setCommand (command);
  free (command);
  tel = NULL;
  commandCond = IN_WAIT_STATE;
}

Rts2CommandChange::Rts2CommandChange (Rts2CommandChange * in_command, Rts2DevClientTelescope * in_tel):Rts2Command
  (in_command)
{
  tel = in_tel;
}

int
Rts2CommandChange::commandReturnFailed (int status)
{
  if (tel)
    tel->moveFailed (status);
  return Rts2Command::commandReturnFailed (status);
}


Rts2CommandCorrect::Rts2CommandCorrect (Rts2Block * in_master, int corr_mark, double ra, double dec, double ra_err, double dec_err):
Rts2Command (in_master)
{
  char *command;
  asprintf (&command, "correct %i %lf %lf %lf %lf", corr_mark, ra_err,
	    dec_err, ra, dec);
  setCommand (command);
  free (command);
}

Rts2CommandChangeFocus::Rts2CommandChangeFocus (Rts2DevClientFocus *
						in_focuser, int in_steps):
Rts2Command (in_focuser->getMaster ())
{
  char *msg;
  focuser = in_focuser;
  asprintf (&msg, "step %i", in_steps);
  setCommand (msg);
  free (msg);
}

int
Rts2CommandChangeFocus::commandReturnFailed (int status)
{
  if (focuser)
    focuser->focusingFailed (status);
  return Rts2Command::commandReturnFailed (status);
}

Rts2CommandExecNext::Rts2CommandExecNext (Rts2Block * in_master, int next_id):
Rts2Command (in_master)
{
  char *command;
  asprintf (&command, "next %i", next_id);
  setCommand (command);
  free (command);
}

Rts2CommandExecGrb::Rts2CommandExecGrb (Rts2Block * in_master, int grb_id):
Rts2Command (in_master)
{
  char *command;
  asprintf (&command, "grb %i", grb_id);
  setCommand (command);
  free (command);
}

Rts2CommandKillAll::Rts2CommandKillAll (Rts2Block * in_master):Rts2Command
  (in_master)
{
  setCommand ("killall");
}
