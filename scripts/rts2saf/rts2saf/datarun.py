#!/usr/bin/python
# (C) 2013, Markus Wildi, markus.wildi@bluewin.ch
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

__author__ = 'markus.wildi@bluewin.ch'
import collections


from rts2saf.sextract import Sextract
from rts2saf.data import DataSxtr

class DataRun(object):
    def __init__(self, debug=False, dSxReference=None, args=None, rt=None, logger=None):
        self.debug=debug
        self.args=args
        self.rt=rt
        self.logger=logger
        self.dataSxtrs=list()
        self.dSxReference=dSxReference
        self.assocFn='/tmp/assoc.lst'


    def buildCats(self, i_nmbrAssc=None):
        i_nmbr=self.dSxReference.fields.index('NUMBER')
        cats=collections.defaultdict(int)
        # count the occurrence of each object
        focPosS=collections.defaultdict(int)
        nmbrs=sorted([ int(x[i_nmbr]) for x in self.dSxReference.catalog ])
        for dSx in self.dataSxtrs:
            for nmbr in nmbrs:

                for nmbrAssc in [ int(x[i_nmbrAssc]) for x in dSx.catalog ]:
                    if nmbr == nmbrAssc:
                        cats[nmbr] +=1
                        focPosS[dSx.focPos] +=1
                        break

        return cats,focPosS

    def dropDSx(self, focPosS=None):
        # drop those catalogs which have to few objects
        # this is a glitch in case two dSx are present with the same focPos
        for focPos, val in focPosS.iteritems():
            if val < self.args.fractObjs * len(self.dSxReference.catalog):
                # remove dSx
                dSxdrp=None
                for dSx in self.dataSxtrs:
                    if focPos == dSx.focPos:
                        dSxdrp=dSx
                        break
                else:
                    continue
                self.dataSxtrs.remove(dSxdrp)
                self.logger.warn('dropDSx: focuser position: {0:5d} dropped dSx, {1:5d}, {2:6.2f} {3:5d}'.format(int(focPos), val, self.args.fractObjs * len(self.dSxReference.catalog), len(self.dSxReference.catalog)))

    def onAlmostImagesAssoc(self):
        # ToDo clarify that -1
        i_nmbrAssc= -1 + self.dataSxtrs[0].fields.index('NUMBER_ASSOC') 
        # build cats
        cats,focPosS=self.buildCats( i_nmbrAssc=i_nmbrAssc)
        # drop
        self.dropDSx(focPosS=focPosS)
        # rebuild the cats
        cats, focPosS=self.buildCats(i_nmbrAssc=i_nmbrAssc)

        foundOnAll=len(self.dataSxtrs)
        assocObjNmbrs=list()
        # identify those objects which are on all images
        for k  in  cats.keys():
            if cats[k] == foundOnAll:
                assocObjNmbrs.append(k) 

        # save the raw values for later analysis
        # initialize data.catalog
        for dSx in self.dataSxtrs:
            dSx.toRawCatalog()
        # copy those catalog entries which are found on all
        for k  in  assocObjNmbrs:
            for dSx in self.dataSxtrs:
                for sx in dSx.rawCatalog:
                    if k == sx[i_nmbrAssc]:
                        dSx.catalog.append(sx)
                        break
                else:
                    self.logger.debug('onAllImages: no break for object: {0}'.format(k))

        # recalculate FWHM, Flux etc.
        i_fwhm= self.dataSxtrs[0].fields.index('FWHM_IMAGE') 

        i_flux=None
        if self.args.flux:
            i_flux= self.dataSxtrs[0].fields.index('FLUX_MAX') 

        for dSx in self.dataSxtrs:
            dSx.fillData(i_fwhm=i_fwhm, i_flux=i_flux)

            if self.args.flux:
                if self.debug: self.logger.debug('onAllImages: {0:5d} {1:8.3f} {2:8.3f} {3:5d}'.format(int(dSx.focPos), dSx.fwhm, dSx.flux, len(dSx.catalog)))
            else:
                if self.debug: self.logger.debug('onAllImages: {0:5d} {1:8.3f} {2:5d}'.format(int(dSx.focPos), dSx.fwhm, len(dSx.catalog)))

        self.logger.info('onAlmostImages: objects: {0}'.format(len(assocObjNmbrs)))


    def onAlmostImages(self):

        focPosS=collections.defaultdict(int)
        for dSx in self.dataSxtrs:
            focPosS[dSx.focPos]=dSx.nObjs

        self.dropDSx(focPosS=focPosS)


    def sextractLoop(self, fitsFns=None):

        for fitsFn in fitsFns:
            rsx= Sextract(debug=self.args.debug, rt=self.rt, logger=self.logger)
            if self.args.flux:
                rsx.appendFluxFields()
                
            if self.dSxReference !=None:
                rsx.appendAssocFields()
                dSx=rsx.sextract(fitsFn=fitsFn, assocFn=self.assocFn)
            else:
                dSx=rsx.sextract(fitsFn=fitsFn, assocFn=None)

            if dSx!=None and dSx.fwhm>0. and dSx.stdFwhm>0.:
                self.dataSxtrs.append(dSx)
            else:
                self.logger.warn('sextractLoop: no result: file: {0}'.format(fitsFn))

    def createAssocList(self, fitsFn=None):
        rsx= Sextract(debug=self.args.sxDebug, createAssoc=self.args.associate, rt=self.rt, logger=self.logger)
        if self.args.flux:
            rsx.appendFluxFields()

        return rsx.sextract(fitsFn=fitsFn, assocFn=self.assocFn)


    def numberOfFocPos(self):
        pos=collections.defaultdict(int)
        for dSx in self.dataSxtrs:
            pos[dSx.focPos] += 1 

        ok=True
        if len(pos) <= self.rt.cfg['MINIMUM_FOCUSER_POSITIONS']:
            self.logger.warn('analyzeRun: to few DIFFERENT focuser positions: {0}<={1} (see MINIMUM_FOCUSER_POSITIONS), continuing'.format(len(pos), self.rt.cfg['MINIMUM_FOCUSER_POSITIONS']))
            if self.debug:
                for p,v in pos.iteritems():
                    self.logger.debug('analyzeRun:{0:5.0f}: {1}'.format(p,v))

            ok=False
        return ok