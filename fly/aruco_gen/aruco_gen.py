import cv2
import numpy as np

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

for aruco_id in range(0,20):
    # Generate the marker
    markerImage = np.zeros((600, 600), dtype=np.uint8)
    cv2.aruco.generateImageMarker(aruco_dict, aruco_id, 600, markerImage)

    # Add a white border of 50 pixels
    borderSize = 100
    markerImageWithBorder = cv2.copyMakeBorder(
        markerImage,
        top=borderSize,
        bottom=borderSize,
        left=borderSize,
        right=borderSize,
        borderType=cv2.BORDER_CONSTANT,
        value=[255, 255, 255])

    cv2.imwrite(f"./img/marker_{aruco_id}_by_cv2.aruco.DICT_4X4_50.png", markerImageWithBorder)
