import numpy as np

class drivingScenario:
    SampleTime = 0.01

class lanespec:
    def __init__(self, arg):
        pass

class road:
    def __init__(self, scenario, roadCenters, *args, **kwargs):
        pass

class vehicle:
    def __init__(self, scenario, *args, **kwargs):
        pass

def trajectory(vehicle, roadCenters, speed):
    pass

def radarDetectionGenerator(*args, **kwargs):
    def sensor(*inner_args, **inner_kwargs):
        return [], 0, False
    return sensor

class multiObjectTracker:
    def __init__(self, *args, **kwargs):
        pass

def createDemoDisplay(egoCar, sensors):
    def plotLaneMarking(*args):
        pass

    def targetOutlines(*args):
        return [], 0, 0, 0, 0, 'b'

    def plotOutline(*args, **kwargs):
        pass

    def plotDetection(*args):
        pass

    def getTrackPositions(*args):
        return [], []

    def getTrackVelocities(*args):
        return []

    def plotTrack(*args):
        pass

    class coverageAreaPlotter:
        def __init__(self, BEP, *args, **kwargs):
            pass

    class birdsEyePlot:
        def __init__(self, *args, **kwargs):
            self.Parent = None

    class detectionPlotter:
        def __init__(self, BEP, *args, **kwargs):
            pass

    class laneMarkingPlotter:
        def __init__(self, BEP, *args, **kwargs):
            pass

    class trackPlotter:
        def __init__(self, BEP, *args, **kwargs):
            pass

    class outlinePlotter:
        def __init__(self, BEP, *args, **kwargs):
            pass

    return birdsEyePlot()

def advance(scenario):
    return True

def targetPoses(egoCar):
    return [egoCar]

def updateTracks(tracker, detectionClusters, time):
    return []

def clusterDetections(detections, vehicleSize):
    return []

def snapnow():
    pass

def initSimDemoFilter(detection):
    H = [[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]
    filter = trackingKF('MotionModel', '2D Constant Velocity', \
        'State', H.T @ detection.Measurement, \
        'MeasurementModel', H, \
        'StateCovariance', H.T @ detection.MeasurementNoise @ H, \
        'MeasurementNoise', detection.MeasurementNoise)
    return filter
def clusterDetections(detections, vehicleSize):
    N = len(detections)
    distances = np.zeros((N, N))
    for i in range(N):
        for j in range(i + 1, N):
            if detections[i].SensorIndex == detections[j].SensorIndex:
                distances[i, j] = np.linalg.norm(detections[i].Measurement[:2] - detections[j].Measurement[:2])
            else:
                distances[i, j] = np.inf
    leftToCheck = np.arange(N)
    i = 0
    detectionClusters = [None] * N
    while leftToCheck.size != 0:
        underConsideration = leftToCheck[0]  # pick the first element from left To check
        clusterInds = (distances[underConsideration, leftToCheck] < vehicleSize)  # find the item whose distance with underConsideration less than vhicleSize
        detInds = leftToCheck[clusterInds]  # get items' id
        clusterDets = np.array(detections)[detInds]  # get detections points of this cluster
        clusterMeas = np.vstack([clusterDets[i].Measurement for i in range(len(clusterDets))]).T  # get cluster points value
        meas = np.mean(clusterMeas, axis=1)  # compute mean range and velocity
        meas2D = np.array([meas[:2], meas[3:5]])  # convert to 2D
        i += 1
        detectionClusters[i-1] = deepcopy(detections[detInds[0]])  # record cluster information
        detectionClusters[i-1].Measurement = meas2D
        leftToCheck = np.delete(leftToCheck, np.argwhere(clusterInds))  # delete found points
    detectionClusters = detectionClusters[:i]  # delete unused space
    for i in range(len(detectionClusters)):
        measNoise = np.zeros((4, 4))
        measNoise[:2, :2] = vehicleSize**2 * np.eye(2)
        measNoise[2:, 2:] = 100 * vehicleSize**2 * np.eye(2)
        detectionClusters[i].MeasurementNoise = measNoise
    return detectionClusters
def updateBEP(BEP, egoCar, detections, confirmedTracks, psel, vsel):
    lmv, lmf = laneMarkingVertices(egoCar)
    plotLaneMarking(findPlotter(BEP,'DisplayName','lane markings'),lmv,lmf)
    position, yaw, length, width, originOffset, color = targetOutlines(egoCar)
    plotOutline(findPlotter(BEP,'Tag','Ground truth'), position, yaw, length, width, OriginOffset=originOffset, Color=color)
    N = len(detections)
    detPos = np.zeros((N,2))
    isRadar = np.ones(N, dtype=bool)
    for i, detection in enumerate(detections):
        detPos[i,:] = detection.Measurement[0:2]
        if detection.SensorIndex > 6:
            isRadar[i] = False
    plotDetection(findPlotter(BEP,'DisplayName','radar'), detPos[isRadar,:])
    trackIDs = [str(track.TrackID) for track in confirmedTracks]
    tracksPos, tracksCov = getTrackPositions(confirmedTracks, psel)
    tracksVel = getTrackVelocities(confirmedTracks, vsel)
    plotTrack(findPlotter(BEP,'DisplayName','track'), tracksPos, tracksVel, tracksCov, trackIDs)

if __name__ == '__main__':
    scenario = drivingScenario()
    scenario.SampleTime = 0.01
    roadCenters = [[0, 0], [50, 0], [100, 0], [250, 20], [500, 40]]
    road(scenario, roadCenters, 'lanes',None)
    egoCar = vehicle(scenario, 'ClassID', 1)
    trajectory(egoCar, np.subtract(roadCenters[1:], [0, 1.8]), 25) # On right lane
    leadCar = vehicle(scenario, 'ClassID', 1)
    trajectory(leadCar, np.subtract([[70, 0], *roadCenters[2:]], [0, 1.8]), 25) # On right lane
    passingCar = vehicle(scenario, 'ClassID', 1)
    waypoints = [[0, -1.8], [50, 1.8], [100, 1.8], [250, 21.8], [400, 32.2], [500, 38.2]]
    trajectory(passingCar, waypoints, 35)
    chaseCar = vehicle(scenario, 'ClassID', 1)
    trajectory(chaseCar, np.subtract(roadCenters[1:], [0, 1.8]), 25) # On right lane
    sensors = [None] * 6
    sensors[0] = radarDetectionGenerator('SensorIndex', 1, 'Height', 0.2, 'MaxRange', 174,
             'SensorLocation', [egoCar.Wheelbase + egoCar.FrontOverhang, 0], 'FieldOfView', [20, 5])
    sensors[1] = radarDetectionGenerator('SensorIndex', 2, 'Height', 0.2, 'Yaw', 180,
             'SensorLocation', [egoCar.RearOverhang, 0], 'MaxRange', 174, 'FieldOfView', [20, 5])
    sensors[2] = radarDetectionGenerator('SensorIndex', 3, 'Height', 0.2, 'Yaw', 120,
             'SensorLocation', [0, egoCar.Width/2], 'MaxRange', 30, 'ReferenceRange',50,
             'FieldOfView', [90, 5], 'AzimuthResolution' , 10, 'RangeResolution', 1.25)
    sensors[3] = radarDetectionGenerator('SensorIndex', 4, 'Height', 0.2, 'Yaw', -120,
             'SensorLocation', [0, -egoCar.Width/2], 'MaxRange', 30, 'ReferenceRange',50,
             'FieldOfView', [90, 5], 'AzimuthResolution' , 10, 'RangeResolution', 1.25)
    sensors[4] = radarDetectionGenerator('SensorIndex', 5, 'Height', 0.2, 'Yaw', 60,
             'SensorLocation', [egoCar.Wheelbase, egoCar.Width/2], 'MaxRange', 30, 'ReferenceRange',50,
             'FieldOfView', [90, 5], 'AzimuthResolution' , 10, 'RangeResolution', 1.25)
    sensors[5] = radarDetectionGenerator('SensorIndex', 6, 'Height', 0.2, 'Yaw', -60,
             'SensorLocation', [egoCar.Wheelbase, -egoCar.Width/2], 'MaxRange', 30, 'ReferenceRange',50,
             'FieldOfView', [90, 5], 'AzimuthResolution' , 10, 'RangeResolution', 1.25)
    tracker = multiObjectTracker(FilterInitializationFcn=initSimDemoFilter,
                                 AssignmentThreshold=30,
                                 ConfirmationParameters=[4, 5],
                                 NumCoastingUpdates=5)
    positionSelector = [[1, 0, 0, 0], [0, 0, 1, 0]] # Position selector
    velocitySelector = [[0, 1, 0, 0], [0, 0, 0, 1]] # Velocity selector
    BEP = createDemoDisplay(egoCar, sensors)
    toSnap = True
    while scenario.advance() and BEP.Parent:
        time = scenario.SimulationTime
        ta = targetPoses(egoCar)
        detections = []
        isValidTime = [False] * 6
        for i in range(6):
            sensorDets, numValidDets, isValidTime[i] = sensors[i](ta, time)
            if numValidDets:
                detections.extend(sensorDets)
        if any(isValidTime):
            vehicleLength = sensors[0].ActorProfiles.Length
            detectionClusters = clusterDetections(detections, vehicleLength)
            confirmedTracks = updateTracks(tracker, detectionClusters, time)
            updateBEP(BEP, egoCar, detections, confirmedTracks, positionSelector, velocitySelector)
        if ta[0].Position[0] > 0 and toSnap:
            toSnap = False
            snapnow()

