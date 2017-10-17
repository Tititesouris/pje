# -*- coding: utf-8 -*-
"""


@authors: Benjamin Mathon and Olivier Losson

--------------------------------------------------------------------------
Suivi d'objets dans une sequence d'images, projet encadre IVI
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

# Importation des modules
import random
import time
import datetime
import cv2
import numpy as np
import os
import math
from ivi import ueye, IS, TUIO

# Nombre max de blobs, on revient à l'etiquette 0 pour un nouveau blob detecte apres le blob NB_MAX_BLOBS-1
NB_MAX_BLOBS = 1024


class Blob:
    def __init__(self, pt, area, inertia, convexity, axes, angle, circularity, contour):
        self.pt = pt  # coordonnees (x,y) du centre de l'ellipse englobante
        self.area = area  # aire du blob
        self.class_id = np.random.randint(NB_MAX_BLOBS)  # ID du blob
        self.inertia = inertia  # inertie du blob
        self.convexity = convexity  # convexite du blob
        self.axes = axes  # axes de l'ellipse englobante
        self.angle = angle  # angle de l'ellipse englobante
        self.circularity = circularity  # circularite du blob
        self.contour = contour  # contour du blob

    def __repr__(self):
        return "Blob ID: %d x: %d y: %d area: %lf inertia: %lf convexity: %lf circularity: %lf" % (
            self.class_id, int(self.pt[0]), int(self.pt[1]), self.area, self.inertia, self.convexity, self.circularity)


def detect(cvImg, parameters):
    blobsResult = []
    contours, hierarchy = cv2.findContours(cvImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for enum, contour in enumerate(contours):
        if contour.shape[0] >= 5:  # un min de 5 points est necessaire pour construire l'ellipse englobante
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            pt, axes, angle = cv2.fitEllipse(contour)
            hull = cv2.convexHull(contour)
            convexity = float(area) / cv2.contourArea(hull)
            circularity = 4. * np.pi * area / (perimeter * perimeter)
            inertia = np.min(axes) / np.max(axes)

            # filtrages
            if parameters.filterByArea:
                if area < parameters.minArea or area > parameters.maxArea:
                    continue
            if parameters.filterByInertia:
                if inertia < parameters.minInertiaRatio or inertia > parameters.maxInertiaRatio:
                    continue
            if parameters.filterByConvexity:
                if convexity < parameters.minConvexity or convexity > parameters.maxConvexity:
                    continue
            if parameters.filterByCircularity:
                if circularity < parameters.minCircularity or circularity > parameters.maxCircularity:
                    continue

            # Si le blob passe tous les filtres
            # Appel au constructeur de la classe blob
            b = Blob(pt, area, inertia, convexity, axes, angle, circularity, contour)
            # puis ajout à la liste des blobs detectes
            blobsResult.append(b)
    return blobsResult


# Fonction principale
if __name__ == '__main__':
    iKey = -1  # Touche clavier

    iChannels = 1  # Nombre de canaux couleur
    iBitsPerPixel = 8  # Nombre de bits par pixel
    thres = 87  # Seuil pour la binarisation

    iNextID = -1  # Etiquette pour un nouveau blob detecte, a incrementer a chaque detection
    dSeuilDistance = 30.  # Distance max pour association

    # Creation d'une palette de couleurs aleatoires differentes pour l'etiquetage couleur des blobs
    random.seed(int(time.time()))
    colors = [(0, 0, 0)] * 100
    for i in np.arange(100):
        reallyNew = False
        while not reallyNew:
            newColor = (random.randint(0, 256), random.randint(0, 256), random.randint(0, 256))
            reallyNew = newColor not in colors[0:i]
        colors[i] = newColor

    # Initilialisation du serveur TUIO
    server = TUIO.TuioServer()

    # Creation et initialisation d'une instance de la classe ueye.camera
    cam = ueye.camera()

    # Definir le mode d'acquisition (n&b ou couleur)
    cam.SetColorMode(IS.SET_CM_BAYER)

    # Affichage de la definition de l'image
    [width, height] = cam.GetImageSize()
    print 'width = ', width, ', height = ', height

    # Definir l'horloge pixel (MHz) et affichage
    cam.SetPixelClock(30)
    print 'pixel clock =', cam.GetPixelClock()

    # Definir la cadence d'acquisition (fps) et affichage
    cam.SetFrameRate(25)
    print 'framerate = ', cam.GetFrameRate()

    # Definir le temps d'integration (ms) et affichage
    cam.set_exposure_time(5)
    print 'exposure time = ', cam.get_exposure_time()

    # Definir les gains, principal et des composantes couleur et affichage
    cam.set_hardware_gain(0, 0, 0, 0)
    print 'gain = ', cam.get_hardware_gain()
    print 'red gain = ', cam.get_red_gain()
    print 'green gain = ', cam.get_green_gain()
    print 'blue gain = ', cam.get_blue_gain()

    # Allocation memoire d'une image uEye (pointee par camera.image) et activation de cette memoire (l'image acquise va y etre stockee)
    cam.AllocImageMem(width, height, iBitsPerPixel)
    cam.SetImageMem()

    # Extraire un fichier de parametres à partir du SimpleBlobdetector
    params = cv2.SimpleBlobDetector_Params()

    # Regles de filtrage des blobs: a completer/modifier pour affiner la detection
    # Filtrer par aire
    params.filterByArea = True
    params.minArea = 400
    params.maxArea = 3000

    # Filtrer par convexite
    params.filterByConvexity = True
    params.minConvexity = 0.8
    params.maxConvexity = 1.0

    # Representation de la memoire contenant l'image sous la forme d'un tableau 2D numpy
    data = cam.toNumpyArray(iChannels)

    # Acquisition prealable de 5 images
    for i in np.arange(5):
        cam.FreezeVideo()

    # Creer la fenetre openCV d'affichage de l'image
    cv2.namedWindow('acquisition')

    # Liste des anciens blobs
    sOldBlobs = []

    while iKey != 27:  # touche 'Echap'
        # Acquerir l'image
        cam.FreezeVideo()

        # Retourner l'image selon l'axe x
        imgFlip = cv2.flip(data, 1)

        # Lisser l'image pour attenuer l'influence du bruit
        # Necessaire avec findContours pour limiter le nombre de blobs detectes
        imgBlurred = cv2.blur(imgFlip, (3, 3))

        # Binariser l'image
        imgBin = cv2.threshold(imgBlurred, thres, 255, cv2.THRESH_BINARY)[1]

        # Generer une image couleur a partir de 3 plans niveaux de gris
        imgColor = cv2.merge([imgBin, imgBin, imgBin])

        # Detection des nouveaux blobs, attention, imgBin est modifiee apres l'appel de la fonction
        sNewBlobs = detect(imgBin, params)

        # Nombre d'anciens et de nouveaux blobs
        iNewBlobs = len(sNewBlobs)
        iOldBlobs = len(sOldBlobs)

        # S'il n'y a pas de blobs dans la nouvelle liste et s'il y en avait dans l'ancienne...
        if not iNewBlobs and iOldBlobs:
            # ...alors les enlever de la liste TUIO
            server.removeAllTuioCursor()
        else:
            # S'il n'y avait de blobs dans l'ancienne liste...
            if not iOldBlobs:
                # Pour chaque nouveau blob
                for j in np.arange(iNewBlobs):
                    # Generer une nouvelle etiquette et l'affecter au nouveau blob
                    iNextID = (iNextID + 1) % NB_MAX_BLOBS
                    sNewBlobs[j].class_id = iNextID

                    # Ajouter a la liste TUIO
                    cursor = TUIO.TuioCursor(sNewBlobs[j].class_id, sNewBlobs[j].pt[0] / width,
                                             sNewBlobs[j].pt[1] / height)
                    server.addExternalTuioCursor(cursor)
            else:
                # Associer les blobs pour le suivi

                # Matrice de distances generalisee
                dDistance = np.zeros((iOldBlobs, iNewBlobs))

                # Calcul des distances generalisees
                for i in np.arange(iOldBlobs):
                    for j in np.arange(iNewBlobs):
                        a = sNewBlobs[j]
                        b = sOldBlobs[i]
                        euclidian = (b.pt[0] - a.pt[0]) ** 2 + (b.pt[1] - a.pt[1]) ** 2
                        surface = (math.sqrt(b.area) - math.sqrt(a.area)) ** 2
                        dDistance[i, j] = math.sqrt(euclidian + surface)

                if iNewBlobs > iOldBlobs:
                    for i in np.arange(iOldBlobs):
                        minJ = -1
                        min = 99999999999999999999
                        for j in np.arange(iNewBlobs):
                            if dDistance[i, j] < min:
                                min = dDistance[i, j]
                                minJ = j

                        for j in np.arange(iNewBlobs):
                            if j != minJ:
                                dDistance[i, j] = -1
                else:
                    for j in np.arange(iNewBlobs):
                        minI = -1
                        min = 99999999999999999999
                        for i in np.arange(iOldBlobs):
                            if dDistance[i, j] < min:
                                min = dDistance[i, j]
                                minI = i

                        for i in np.arange(iOldBlobs):
                            if i != minI:
                                dDistance[i, j] = -1

                # Elimination des distances min superieures au seuil
                for i in np.arange(iOldBlobs):
                    for j in np.arange(iNewBlobs):
                        if dDistance[i, j] > dSeuilDistance:
                            dDistance[i, j] = -1.

                # Afficher les distances
                # print 'distances = \n', dDistance, "\n"

                # Enlever les anciens blobs non associes de la liste TUIO
                for i in np.arange(iOldBlobs):
                    rem = True
                    for j in np.arange(iNewBlobs):
                        if dDistance[i, j] > -1:
                            rem = False
                    if rem:
                        cur = server.getTuioCursor(sOldBlobs[i].class_id)
                        server.removeExternalTuioCursor(cur)

                # Balayer la liste des nouveaux blobs
                for j in np.arange(iNewBlobs):
                    dMaxColonne = -1.
                    iPosMaxColonne = -1
                    for i in np.arange(iOldBlobs):
                        if dDistance[i, j] > dMaxColonne:
                            dMaxColonne = dDistance[i, j]
                            iPosMaxColonne = i

                    # S'il existe une distance mini dans la colonne
                    if iPosMaxColonne != -1:
                        sNewBlobs[j].class_id = sOldBlobs[iPosMaxColonne].class_id
                        cur = server.getTuioCursor(sNewBlobs[j].class_id)
                        server.updateTuioCursor(cur, sNewBlobs[j].pt[0], sNewBlobs[j].pt[1])
                    else:
                        iNextID = (iNextID + 1) % NB_MAX_BLOBS
                        sNewBlobs[j].class_id = iNextID
                        cur = TUIO.TuioCursor(sNewBlobs[j].class_id, sNewBlobs[j].pt[0], sNewBlobs[j].pt[1])
                        server.addExternalTuioCursor(cur)

        for i in np.arange(0, iNewBlobs):
            k = sNewBlobs[i]

            x = int(k.pt[0])
            y = int(k.pt[1])

            # Remplissage de l'ellipse du blob avec une couleur propre
            cv2.ellipse(imgColor, (k.pt, k.axes, k.angle), color=colors[k.class_id % 100], thickness=-1)

            # Trace d'un point au centre du blob
            cv2.line(imgColor, (x, y), (x, y), (0, 0, 255), thickness=3)

            # Etiquetage du blob avec son ID
            cv2.putText(imgColor, str(k.class_id), (x, y), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255),
                        thickness=1)

        # Afficher l'image
        cv2.imshow('acquisition', imgColor)

        # Envoyer la mise a jour TUIO
        server.commitFrame()

        # Copier les nouveaux blobs dans les anciens
        sOldBlobs = sNewBlobs

        # Attendre le temps d'une periode d'acquisition (en ms) que l'utilisateur appuie sur une touche
        iKey = cv2.waitKey(int(1000. / cam.GetFrameRate()))

        # Sauvegarder des snapshots de l'acquisition
        if iKey == 115:  # touche 's'
            if not os.path.isdir('./acquisitions'):
                os.mkdir('./acquisitions')
            t = time.time()
            timestamp = datetime.datetime.fromtimestamp(t).strftime('%Y-%m-%d-%H-%M-%S')
            filename = 'acquisitions/ueye_' + timestamp + '.png'
            cv2.imwrite(filename, imgColor)
            print 'Image saved to ' + filename

    # Fermer les fenetres openCV, liberer la memoire et finir sans erreur
    cv2.destroyAllWindows()
    print "window closed"
    cam.FreeImageMem()
    cam.ExitCamera()
