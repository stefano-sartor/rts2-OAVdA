#!/usr/bin/python
# (C) 2010, Markus Wildi, markus.wildi@one-arcsec.org
#
#   usage 
#   rts_autofocus.py --help
#   
#   see man 1 rts2_autofocus.py
#   see rts2_autofocus_unittest.py for unit tests
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2, or (at your option)
#   any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program; if not, write to the Free Software Foundation,
#   Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
#
#   Or visit http://www.gnu.org/licenses/gpl.html.
#
# required packages:
# wget 'http://argparse.googlecode.com/files/argparse-1.1.zip

__author__ = 'markus.wildi@one-arcsec.org'

import io
import os
import re
import shutil
import string
import sys
import time
import shlex 
import subprocess

import numpy
import pyfits
import rts2comm 
import rts2af 

class main(rts2af.AFScript):
    """define the focus from a series of images"""
    def __init__(self, scriptName='main'):
        self.scriptName= scriptName

    def main(self):

        dc= rts2af.Configuration()

        args= self.arguments(dc)

        configFileName=''
        if( args.fileName):
            configFileName= args.fileName[0]  
        else:
            configFileName= dc.configurationFileName()
            cmd= 'logger no config file specified, taking default' + configFileName
            #print cmd 
            os.system( cmd)

        if(args.verbose):
            print 'config file, taking ' + configFileName

        dc.readConfiguration(configFileName, args.verbose)

        if(args.verbose):
            for  identifier, value in dc.valuesIdentifiers():
                print "actual configuration values :", identifier, '=>', value
        
        #retcode = subprocess.call(["ls", "-l"])
        # Python 2.7
        #subprocess.check_output(["ls", "-l", "/dev/null"])

        #pid = subprocess.Popen(['ls', '-l']).pid
        output1 = subprocess.Popen(['logger', 'hallo 1'], stdout=subprocess.PIPE).communicate()[0]
        #output2 = subprocess.Popen(['sleep', '5'], stdout=subprocess.PIPE).communicate()[0]
        pid= subprocess.Popen(['./schlaf', '5']).pid
        output3 = subprocess.Popen(['logger', 'hallo 2'], stdout=subprocess.PIPE).communicate()[0]
        time.sleep(1)
        print "out "+ output1
# talk to centrald

#        con= rts2comm.Rts2Comm()
#        if( con.isEvening()):
#            cmd= 'logger evening' 
#        else:
#            cmd= 'logger morning'

#        os.system( cmd)



if __name__ == '__main__':
    main(sys.argv[0]).main()



