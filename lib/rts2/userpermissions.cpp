/* 
 * Class holding user permissions.
 * Copyright (C) 2013 Petr Kubanek, Institute of Physics <kubanek@fzu.cz>
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

#include "userpermissions.h"
#include "utilsfunc.h"
#include <iostream> //DBG
using namespace rts2core;

void UserPermissions::parsePermissions (const char *permissionString)
{
	allowedDevices.clear ();
	if (permissionString == NULL)
		return;
	allowedDevices = SplitStr (std::string (permissionString), " ");
}

bool UserPermissions::canWriteDevice (const std::string &deviceName)
{
	std::cout << "+++++ canWriteDevice ++++" << std::endl; //DBG
 	for (std::vector <std::string>::iterator iter = allowedDevices.begin (); iter != allowedDevices.end(); iter++)
	{
		//DBG
		std::cout << *iter << std::endl;
		//DBG
		// find * for wildcard..
		size_t star = iter->find('*');
		if (star == 0)
		{
			return true;
		}
		else if (star > 0)
		{
			if (iter->substr (0, star - 1) == deviceName.substr (0, star - 1))
				return true;
		}
		else if (*iter == deviceName)
		{
			return true;
		}
	}
	return true; //DBG
	return false;
}
