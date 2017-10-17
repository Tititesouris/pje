import cv2
import sys
import matplotlib.pyplot as plt
import matplotlib.colors as col
from cv2.cv import Scalar


def lire(nomImage):
    """Retourne l'image en niveaux de gris sur 8 bits nomImage."""
    image = cv2.imread(nomImage, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    if image is None:
        sys.exit("Impossible de charger l'image " + nomImage)
    if image.dtype != 'uint8':
        sys.exit("Type incorrect" + image.dtype + " (uint8 attendu)")
    return image


def histo(image, roi=None):
    if roi:
        x1 = roi[0]
        y1 = roi[1]
        x2 = x1 + roi[2]
        y2 = y1 + roi[3]
        img = image[y1:y2, x1:x2]
    else:
        img = image
    return cv2.calcHist([img], [0], None, [256], [0, 256])


def seuiller(image, roi=None, threshold=0.0):
    if roi:
        x1 = roi[0]
        y1 = roi[1]
        x2 = x1 + roi[2]
        y2 = y1 + roi[3]
        img = image[y1:y2, x1:x2]
    else:
        img = image
    if threshold > 0:
        return cv2.threshold(img, threshold, 1, cv2.THRESH_BINARY)
    else:
        return cv2.threshold(img, threshold, 1, cv2.THRESH_OTSU)

def filtrer(image):
    return cv2.medianBlur(image, 5)


def filtrerSeuiller(image):
    return seuiller(cv2.medianBlur(image, 5))[1]

img = lire("fingers.png")
roi = [325, 160, 50, 50]
hist = histo(img, roi)
imgFilt = filtrer(img)
imgFiltBin = filtrerSeuiller(img)
imgBin = seuiller(img)[1]

plt.subplot(2, 3, 1)
plt.bar(range(256), hist, 1)

plt.subplot(2, 3, 2)
cv2.rectangle(img, (roi[0], roi[1]), (roi[0] + roi[2], roi[1] + roi[3]), Scalar(255, 255, 255), 3)
plt.imshow(img, cmap="gray")

plt.subplot(2, 3, 3)
plt.imshow(imgFilt, cmap="gray")

plt.subplot(2, 3, 4)
plt.imshow(imgFiltBin, cmap="gray")

plt.subplot(2, 3, 5)
plt.imshow(imgBin, cmap="gray")

plt.show()
