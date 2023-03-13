from typing import List

class LidarPoint:
    """
    A class representing a single lidar point in space.
    """
    def __init__(self, x: float, y: float, z: float, r: float):
        """
        Constructor.

        Args:
            x: x-coordinate in meters.
            y: y-coordinate in meters.
            z: z-coordinate in meters.
            r: Reflectivity of the point.
        """
        self.x = x
        self.y = y
        self.z = z
        self.r = r

class BoundingBox:
    """
    A class representing a 3D box.
    """
    def __init__(self, lidarPoints: List[LidarPoint], cameraPoints: List[float]):
        """
        Constructor.

        Args:
            lidarPoints: List of lidar points enclosed by the box.
            cameraPoints: List of camera image coordinates enclosed by the box.
        """
        self.lidarPoints = lidarPoints
        self.cameraPoints = cameraPoints
        self.lidarBox = None
        self.bbox = None
        self.bboxCorners = None

class DataFrame:
    """
    A class representing a single camera/lidar data frame.
    """
    def __init__(self):
        """
        Constructor.
        """
        self.cameraImg = None
        self.lidarPoints = []
        self.boundingBoxes = []
