# -*- coding: utf-8 -*-
"""


@author: Benjamin Mathon

--------------------------------------------------------------------------
Acquisition sur camera IDS uEye, projet encadre IVI
Copyright (C) 2017  Universite Lille 1

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
--------------------------------------------------------------------------
"""

# Importation des modules necessaires
import datetime
import os
import time
import cv2
from ivi import ueye, IS

iKey = -1  # Touche clavier

# Creation et initialisation d'une instance de la classe ueye.camera
cam = ueye.camera()

# Definir le mode d'acquisition (n&b ou couleur)
cam.SetColorMode(IS.SET_CM_BAYER)  # SET_CM_Y8 = niveaux de gris
iChannels = 1  # Nombre de canaux couleur
iBitsPerPixel = 8  # Nombre de bits par pixel

# Affichage de la definition de l'image acquise
[widthMax, heightMax] = cam.GetImageSize()
print 'widthMax = ', widthMax, ', heightMax = ', heightMax

# Definir la position et la definition de la region d'interet (AOI) et affichage
cam.SetAOI(isType=IS.SET_IMAGE_AOI, x=0, y=0, width=widthMax, height=heightMax)
aoiKeys = [2490368, 2555904, 2621440, 2424832, 56, 54, 50, 52]
print "AOI = ", cam.GetAOI()

# Mode de binning (desactive ou vertical et/ou horizontal)
cam.SetBinning(IS.BINNING_DISABLE)

# Affichage de la definition de l'image acquise apres AOI et binning
[width, height] = cam.GetImageSize()
print 'width = ', width, ', height = ', height

# Definir l'horloge pixel (MHz) et affichage
cam.SetPixelClock(30)
print 'pixel clock =', cam.GetPixelClock()

# Definir la cadence d'acquisition (fps) et affichage
cam.SetFrameRate(25)
print 'framerate = ', cam.GetFrameRate()

# Definir le temps d'integration (ms) et affichage
cam.set_exposure_time(10)
print 'exposure time = ', cam.get_exposure_time()

# Definir les gains, principal et des composantes couleur et affichage
cam.set_hardware_gain(0, 4, 0, 13)
print 'gain = ', cam.get_hardware_gain()
print 'red gain = ', cam.get_red_gain()
print 'green gain = ', cam.get_green_gain()
print 'blue gain = ', cam.get_blue_gain()

# Creer la fenetre openCV d'affichage de l'image
cv2.namedWindow('acquisition')

# Allocation memoire d'une image uEye (pointee par camera.image) et activation de cette memoire (l'image acquise va y etre stockee)
cam.AllocImageMem(width, height, iBitsPerPixel)
cam.SetImageMem()

# Representation de la memoire contenant l'image sous la forme d'un tableau 2D numpy
data = cam.toNumpyArray(iChannels)

while iKey != 27:  # tant qu'on n'appuie pas sur la touche 'Echap'
    # Acquerir l'image
    cam.FreezeVideo()

    # Afficher l'image
    cv2.imshow('acquisition', data)

    # Attendre 1 ms que l'utilisateur appuie sur une touche
    iKey = cv2.waitKey(1)

    if iKey == 97:  # touche 'a'
        pc = cam.GetPixelClock()
        if pc > 5:
            cam.SetPixelClock(pc - 1)
        print 'pixel clock =', cam.GetPixelClock()

    if iKey == 122:  # touche 'z'
        pc = cam.GetPixelClock()
        if pc < 40:
            cam.SetPixelClock(pc + 1)
        print 'pixel clock =', cam.GetPixelClock()

    if iKey == 101:  # touche 'e'
        fps = cam.GetFrameRate()
        if fps > 1:
            cam.SetFrameRate(fps - 1)
        print 'fps =', cam.GetFrameRate()

    if iKey == 114:  # touche 'r'
        fps = cam.GetFrameRate()
        if fps < 60:
            cam.SetFrameRate(fps + 1)
        print 'fps =', cam.GetFrameRate()

    if iKey == 116:  # touche 't'
        exp = cam.get_exposure_time()
        if exp > 1:
            cam.set_exposure_time(exp - 1)
        print 'exp =', cam.get_exposure_time()

    if iKey == 121:  # touche 'y'
        exp = cam.get_exposure_time()
        if exp < 100:
            cam.set_exposure_time(exp + 1)
        print 'exp =', cam.get_exposure_time()

    if iKey == 117:  # touche 'u'
        gain = cam.get_hardware_gain()
        if gain > 0:
            cam.set_hardware_gain(gain - 1, cam.get_red_gain(), cam.get_green_gain(), cam.get_blue_gain())
        print 'gain =', cam.get_hardware_gain()

    if iKey == 105:  # touche 'i'
        gain = cam.get_hardware_gain()
        if gain < 255:
            cam.set_hardware_gain(gain + 1, cam.get_red_gain(), cam.get_green_gain(), cam.get_blue_gain())
        print 'gain =', cam.get_hardware_gain()

    if iKey in aoiKeys:  # touches directionelles fleches et numpad
        x, y, w, h = cam.GetAOI()
        cmd = aoiKeys.index(iKey)
        if cmd == 0 and y < heightMax - 1:
            y += 2
        elif cmd == 1 and x < widthMax - 1:
            x += 2
        elif cmd == 2 and y > 0:
            y -= 2
        elif cmd == 3 and x > 0:
            x -= 2
        elif cmd == 4 and h < heightMax:
            h *= 2
        elif cmd == 5 and w < widthMax:
            w *= 2
        elif cmd == 6 and h > 1:
            h /= 2
        elif cmd == 7 and w > 1:
            w /= 2
        cam.SetAOI(isType=IS.SET_IMAGE_AOI, x=x, y=y, width=w, height=h)
        print "aoi: " + str(cam.GetAOI())

    if iKey == 115:  # touche 's'
        if not os.path.isdir('./acquisitions'):
            os.mkdir('./acquisitions')
        t = time.time()
        timestamp = datetime.datetime.fromtimestamp(t).strftime('%Y-%m-%d-%H-%M-%S')
        filename = 'acquisitions/ueye_' + timestamp + '.png'
        cv2.imwrite(filename, data)
        print 'Image saved to ' + filename

# Fermer les fenetres openCV, liberer la memoire et finir sans erreur
cv2.destroyAllWindows()
print 'window closed'
cam.FreeImageMem()
cam.ExitCamera()
