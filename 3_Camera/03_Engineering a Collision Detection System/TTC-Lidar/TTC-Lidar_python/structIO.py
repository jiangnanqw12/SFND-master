import numpy as np
import cv2
from typing import List

from dataStructures import LidarPoint

def readLidarPts(fileName: str, lidarPoints: List[LidarPoint]):
    """
    Reads Lidar points from a file.

    Args:
        fileName: Path to the file.

    Returns:
        A list of LidarPoint objects.
    """

    with open(fileName, 'r',encoding='iso-8859-1') as f:
        for line in f:
            data = line.split()
            x, y, z, r = float(data[0]), float(data[1]), float(data[2]), float(data[3])
            lidarPoints.append(LidarPoint(x, y, z, r))
    return lidarPoints

def writeLidarPts(lidarPoints: List[LidarPoint], fileName: str):
    """
    Writes Lidar points to a file.

    Args:
        lidarPoints: List of LidarPoint objects.
        fileName: Path to the file.
    """
    with open(fileName, 'w') as f:
        for point in lidarPoints:
            f.write('{} {} {} {}\n'.format(point.x, point.y, point.z, point.r))

def readCalibration(fileName: str):
    """
    Reads calibration data from file.

    Args:
        fileName: Path to the file.

    Returns:
        Calibration data as a dictionary.
    """
    with open(fileName, 'r') as f:
        lines = f.readlines()
        P_rect_00 = np.fromstring(lines[0], sep=' ')
        P_rect_00 = P_rect_00.reshape(3, 4)
        R_rect_00 = np.fromstring(lines[1], sep=' ')
        R_rect_00 = R_rect_00.reshape(3, 3)
        cam2world = np.fromstring(lines[2], sep=' ')
        cam2world = cam2world.reshape(3, 4)
    return {'P_rect_00': P_rect_00, 'R_rect_00': R_rect_00, 'cam2world': cam2world}

def readImage(fileName: str) -> np.ndarray:
    """
    Reads an image from file.

    Args:
        fileName: Path to the file.

    Returns:
        An image as a numpy array.
    """
    return cv2.imread(fileName)
