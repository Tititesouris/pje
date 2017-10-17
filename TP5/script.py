import cv2
import sys
import time
import numpy as np
from ivi import ueye, IS
import random


def lire(nomImage):
    """Retourne l'image en niveaux de gris sur 8 bits nomImage."""
    image = cv2.imread(nomImage, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    if image is None:
        sys.exit("Impossible de charger l'image " + nomImage)
    if image.dtype != 'uint8':
        sys.exit("Type incorrect" + image.dtype + " (uint8 attendu)")
    return image


def create_detector(threshold, area, circularity, convexity, inertia):
    params = cv2.SimpleBlobDetector_Params()
    params.blobColor = 255
    params.minThreshold = threshold[0]
    params.maxThreshold = threshold[1]
    params.thresholdStep = threshold[2]
    params.filterByArea = area[0]
    params.minArea = area[1]
    params.maxArea = area[2]
    params.filterByCircularity = circularity[0]
    params.minCircularity = circularity[1]
    params.maxCircularity = circularity[2]
    params.filterByConvexity = convexity[0]
    params.minConvexity = convexity[1]
    params.maxConvexity = convexity[2]
    params.filterByInertia = inertia[0]
    params.minInertiaRatio = inertia[1]
    params.maxInertiaRatio = inertia[2]

    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3:
        return cv2.SimpleBlobDetector(params)
    return cv2.SimpleBlobDetector_create(params)


def infos_simple_blobs(image):
    detector = create_detector([10, 200, 10], [True, 100, 10000], [True, 0.8, 3.40282346639e+38],
                               [True, 0.5, 2], [True, 0, 3.40282346639e+38])
    keypoints = detector.detect(image)
    print keypoints
    img = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.namedWindow("bob")
    cv2.imshow("bob", img)
    cv2.waitKey(0)
    cv2.destroyWindow("bob")


def seuiller(img, thres=0.0):
    if thres > 0:
        return cv2.threshold(img, thres, 255, cv2.THRESH_BINARY)[1]
    else:
        return cv2.threshold(img, thres, 255, cv2.THRESH_OTSU)[1]


def process_detector(data):
    binImg = seuiller(data)
    detector = create_detector([10, 200, 10], [True, 100, 5000], [True, 0.1, 3.4e+38], [False, 0, 0], [False, 0, 0])
    keypoints = detector.detect(binImg)
    return cv2.drawKeypoints(binImg, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


def palette(nbCouleurs):
    random.seed(int(time.time()))
    colors = [(0, 0, 0)] * nbCouleurs
    for i in np.arange(nbCouleurs):
        reallyNew = False
        while not (reallyNew):
            newColor = (random.randint(0, 256), random.randint(0, 256), random.randint(0, 256))
            reallyNew = newColor not in colors[0:i]
        colors[i] = newColor
    return colors


def process_contours(data):
    binImg = seuiller(data)
    contours, hierarchy = cv2.findContours(binImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contoursBlobs = []
    for contour in contours:
        rect = cv2.boundingRect(contour)
        if cv2.contourArea(contour) > 500 and rect[2] + rect[3] < 120:
            contoursBlobs.append(contour)
    imContours = np.zeros((binImg.shape[0], binImg.shape[1], 3), np.uint8)
    colors = palette(len(contoursBlobs))
    for i, contour in enumerate(contoursBlobs):
        cv2.drawContours(imContours, contoursBlobs, i, colors[i], thickness=3)
    return imContours


def process(data):
    binImg = seuiller(data)
    contours, hierarchy = cv2.findContours(binImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contoursBlobs = []
    for contour in contours:
        rect = cv2.boundingRect(contour)
        if cv2.contourArea(contour) > 500 and rect[2] + rect[3] < 120:
            m = cv2.moments(contour)
            contoursBlobs.append(
                {"center": (int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])), "radius": int(max(rect[2], rect[3]) / 2)})
    imContours = np.zeros((binImg.shape[0], binImg.shape[1], 3), np.uint8)
    colors = palette(len(contoursBlobs))
    for i, contour in enumerate(contoursBlobs):
        cv2.circle(imContours, contour["center"], contour["radius"], colors[i], thickness=3)
    return imContours


def capture():
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
    cam.SetFrameRate(30)
    print 'framerate = ', cam.GetFrameRate()

    # Definir le temps d'integration (ms) et affichage
    cam.set_exposure_time(2)
    print 'exposure time = ', cam.get_exposure_time()

    # Definir les gains, principal et des composantes couleur et affichage
    cam.set_hardware_gain(60, 4, 0, 13)
    print 'gain = ', cam.get_hardware_gain()
    print 'red gain = ', cam.get_red_gain()
    print 'green gain = ', cam.get_green_gain()
    print 'blue gain = ', cam.get_blue_gain()

    cv2.namedWindow('camera')

    # Allocation memoire d'une image uEye (pointee par camera.image) et activation de cette memoire (l'image acquise va y etre stockee)
    cam.AllocImageMem(width, height, iBitsPerPixel)
    cam.SetImageMem()

    # Representation de la memoire contenant l'image sous la forme d'un tableau 2D numpy
    data = cam.toNumpyArray(iChannels)
    iKey = -1

    while iKey != 27:
        # Acquerir l'image
        cam.FreezeVideo()
        processedData = process(data)
        cv2.imshow('camera', processedData)

        iKey = cv2.waitKey(1)

    cv2.destroyAllWindows()
    print 'window closed'
    cam.FreeImageMem()
    cam.ExitCamera()


capture()
