/*
 * Copyright (C) 2012 Petr Kubanek <petr@kubanek.net>
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

#include "bbconn.h"
#include "bbdb.h"
#include "rts2db/target.h"
#include "configuration.h"

using namespace rts2bb;

ConnBBQueue::ConnBBQueue (rts2core::Block * _master, const char *_exec):rts2script::ConnExe (_master, _exec, false)
{
}

void ConnBBQueue::processCommand (char *cmd)
{
	// create mapping
	if (!strcasecmp (cmd, "mapping"))
	{
		int observatory_id;
		int tar_id;
		int obs_tar_id;

		if (paramNextInteger (&observatory_id) || paramNextInteger (&tar_id) || paramNextInteger (&obs_tar_id))
			return;
		createMapping (observatory_id, tar_id, obs_tar_id);
	}
	else if (!strcasecmp (cmd, "targetinfo"))
	{
		int tar_id;

		if (paramNextInteger (&tar_id))
			return;
	
		rts2db::Target *tar = createTarget (tar_id, rts2core::Configuration::instance ()->getObserver ());

		std::ostringstream os;

		ln_equ_posn pos;
		tar->getPosition (&pos);

		os << '"' << tar->getTargetName () << "\" " << pos.ra << " " << pos.dec;

		writeToProcess (os.str ().c_str ());
	}
	else if (!strcasecmp (cmd, "obsapiurl"))
	{
		int observatory_id;

		if (paramNextInteger (&observatory_id))
			return;


		Observatory obs (observatory_id);
		obs.load ();

		writeToProcess (obs.getURL ());
	}
	else
	{
		rts2script::ConnExe::processCommand (cmd);
	}
}
