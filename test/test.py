# -*- coding: utf-8 -*-
"""
Introduction à OpenCV, projet encadre IVI
@author: Olivier Losson
"""

# Importer les paquets nécessaires
import sys
import cv2
import numpy
import matplotlib.pyplot as plt


def infos(nomImage):
    """Affiche les informations relatives à l'image nomImage."""
    # Nom de la fenêtre d'affichage
    nomFenetre = "image"
    # Lire le fichier image
    image = cv2.imread(nomImage, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    if image is None:  # imread retourne None en cas d'échec (pas d'exception levée)
        sys.exit("Impossible de charger l'image " + nomImage)
    # Créer une fenêtre et y afficher l'image
    cv2.namedWindow(nomFenetre)
    cv2.imshow(nomFenetre, image)
    # Afficher des propriétés de l'image sur la console
    print "Infos sur l'image", nomImage
    print "Nombre de valeurs : ", image.size
    print ("Définition : " + str(image.shape[0]) + "x" + str(image.shape[1]))
    print ("Taille : " + str(image.nbytes))
    print ("Canaux : " + str(image.nbytes / (image.shape[0] * image.shape[1])))
    print ("Profondeur : " + str(image.dtype.itemsize * 8) + "bits")
    minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(image)
    print ("Profondeur : " + str(minVal) + str(maxVal) + str(minLoc) + str(maxLoc) + "bits")
    print "Appuyer sur une touche pour terminer."
    # Sur appui d'une touche, fermer la fenêtre
    cv2.waitKey(0)
    cv2.destroyWindow(nomFenetre)


def seuiller(nomImage, imgbin, iSeuil):
    image = cv2.imread(nomImage, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    imagebin = cv2.imread(imgbin, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    bob, binaire = cv2.threshold(image, iSeuil, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    bob, binaire2 = cv2.threshold(image, iSeuil * 2, 255, cv2.THRESH_BINARY)
    binaire3 = cv2.compare(binaire, imagebin, cv2.CMP_EQ)
    binaire4 = cv2.compare(binaire2, imagebin, cv2.CMP_EQ)

    cv2.namedWindow("boop")
    cv2.imshow("boop", image)
    cv2.waitKey(0)
    cv2.imshow("boop", binaire)
    cv2.waitKey(0)
    cv2.imshow("boop", binaire3)
    cv2.waitKey(0)
    cv2.imshow("boop", binaire2)
    cv2.waitKey(0)
    cv2.imshow("boop", binaire4)
    cv2.waitKey(0)
    cv2.destroyWindow("boop")


def getCanal(nomImage, iCanal):
    image = cv2.imread(nomImage, cv2.CV_LOAD_IMAGE_COLOR)
    filtered = []
    i = 0
    for line in image:
        filtered.append([])
        for pixel in line:
            filtered[i].append([pixel[0], 0, 0])
        i += 1
    filtered = numpy.asarray(filtered)
    #filtered.setflags(write=True)
    #filtered.__internals__ = image.__internals__
    filtered.dtype = image.dtype
    #filtered.max = image.max
    #filtered.min = image.min
    filtered.shape = image.shape
    filtered.size = image.size
    return filtered

img = getCanal("gateaux1.png", 0)
cv2.namedWindow("boop")
cv2.imshow("boop", img)

cv2.waitKey(0)
cv2.destroyWindow("boop")