import numpy as np
import cv2
from typing import List

from dataStructures import LidarPoint
from structIO import readLidarPts

def computeTTCLidar(lidarPointsPrev: List[LidarPoint], lidarPointsCurr: List[LidarPoint], TTC: float):
    # auxiliary variables
    dT = 0.1        # time between two measurements in seconds
    laneWidth = 4.0 # assumed width of the ego lane

    # find closest distance to Lidar points within ego lane
    minXPrev = 1e9
    for it in lidarPointsPrev:
        if (it.y >= -1 * laneWidth / 2) and (it.y <= laneWidth / 2):
            minXPrev = min(minXPrev, it.x)

    minXCurr = 1e9
    for it in lidarPointsCurr:
        if (it.y >= -1 * laneWidth / 2) and (it.y <= laneWidth / 2):
            minXCurr = min(minXCurr, it.x)

    # compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr)
    return TTC

if __name__ == '__main__':
    currLidarPts = []
    prevLidarPts = []
    readLidarPts('../dat/C22A5_currLidarPts.dat', currLidarPts)
    readLidarPts('../dat/C22A5_prevLidarPts.dat', prevLidarPts)

    ttc = 0.0
    ttc = computeTTCLidar(prevLidarPts, currLidarPts, ttc)
    print("ttc = {:.2f}s".format(ttc))
