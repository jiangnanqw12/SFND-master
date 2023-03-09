scenario = drivingScenario;
scenario.SampleTime = 0.01;
roadCenters = [0 0; 50 0; 100 0; 250 20; 500 40];
road(scenario, roadCenters, 'lanes', lanespec(2));
egoCar = vehicle(scenario, 'ClassID', 1);
trajectory(egoCar, roadCenters(2:end, :) - [0 1.8], 25); % On right lane
leadCar = vehicle(scenario, 'ClassID', 1);
trajectory(leadCar, [70 0; roadCenters(3:end, :)] - [0 1.8], 25); % On right lane
passingCar = vehicle(scenario, 'ClassID', 1);
waypoints = [0 -1.8; 50 1.8; 100 1.8; 250 21.8; 400 32.2; 500 38.2];
trajectory(passingCar, waypoints, 35);
chaseCar = vehicle(scenario, 'ClassID', 1);
trajectory(chaseCar, [25 0; roadCenters(2:end, :)] - [0 1.8], 25); % On right lane
sensors = cell(6, 1);
sensors{1} = radarDetectionGenerator('SensorIndex', 1, 'Height', 0.2, 'MaxRange', 174, ...
    'SensorLocation', [egoCar.Wheelbase + egoCar.FrontOverhang, 0], 'FieldOfView', [20, 5]);
sensors{2} = radarDetectionGenerator('SensorIndex', 2, 'Height', 0.2, 'Yaw', 180, ...
    'SensorLocation', [egoCar.RearOverhang, 0], 'MaxRange', 174, 'FieldOfView', [20, 5]);
sensors{3} = radarDetectionGenerator('SensorIndex', 3, 'Height', 0.2, 'Yaw', 120, ...
    'SensorLocation', [0, egoCar.Width / 2], 'MaxRange', 30, 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);
sensors{4} = radarDetectionGenerator('SensorIndex', 4, 'Height', 0.2, 'Yaw', -120, ...
    'SensorLocation', [0, -egoCar.Width / 2], 'MaxRange', 30, 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);
sensors{5} = radarDetectionGenerator('SensorIndex', 5, 'Height', 0.2, 'Yaw', 60, ...
    'SensorLocation', [egoCar.Wheelbase, egoCar.Width / 2], 'MaxRange', 30, 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);
sensors{6} = radarDetectionGenerator('SensorIndex', 6, 'Height', 0.2, 'Yaw', -60, ...
    'SensorLocation', [egoCar.Wheelbase, -egoCar.Width / 2], 'MaxRange', 30, 'ReferenceRange', 50, ...
    'FieldOfView', [90, 5], 'AzimuthResolution', 10, 'RangeResolution', 1.25);
tracker = multiObjectTracker('FilterInitializationFcn', @initSimDemoFilter, ...
    'AssignmentThreshold', 30, 'ConfirmationParameters', [4 5], 'NumCoastingUpdates', 5);
positionSelector = [1 0 0 0; 0 0 1 0]; % Position selector
velocitySelector = [0 1 0 0; 0 0 0 1]; % Velocity selector
BEP = createDemoDisplay(egoCar, sensors);
toSnap = true;
while advance(scenario) && ishghandle(BEP.Parent)
    time = scenario.SimulationTime;
    ta = targetPoses(egoCar);
    detections = {};
    isValidTime = false(1, 6);
    for i = 1:6
        [sensorDets, numValidDets, isValidTime(i)] = sensors{i}(ta, time);
        if numValidDets
            detections = [detections; sensorDets]; %#ok<AGROW>
        end
    end
    if any(isValidTime)
        vehicleLength = sensors{1}.ActorProfiles.Length;
        detectionClusters = clusterDetections(detections, vehicleLength);
        confirmedTracks = updateTracks(tracker, detectionClusters, time);
        updateBEP(BEP, egoCar, detections, confirmedTracks, positionSelector, velocitySelector);
    end
    if ta(1).Position(1) > 0 && toSnap
        toSnap = false;
        snapnow
    end
end
function filter = initSimDemoFilter(detection)
    H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1];
    filter = trackingKF('MotionModel', '2D Constant Velocity', ...
        'State', H' * detection.Measurement, ...
        'MeasurementModel', H, ...
        'StateCovariance', H' * detection.MeasurementNoise * H, ...
        'MeasurementNoise', detection.MeasurementNoise);
end
function detectionClusters = clusterDetections(detections, vehicleSize)
    N = numel(detections);
    distances = zeros(N);
    for i = 1:N
        for j = i + 1:N
            if detections{i}.SensorIndex == detections{j}.SensorIndex
                distances(i, j) = norm(detections{i}.Measurement(1:2) - detections{j}.Measurement(1:2));
            else
                distances(i, j) = inf;
            end
        end
    end
    leftToCheck = 1:N;
    i = 0;
    detectionClusters = cell(N, 1);
    while ~isempty(leftToCheck)
        underConsideration = leftToCheck(1);
        clusterInds = (distances(underConsideration, leftToCheck) < vehicleSize);
        disp(N);
        disp(clusterInds);
        detInds = leftToCheck(clusterInds); %get items's id
        clusterDets = [detections{detInds}]; %get detections points of this cluster
        clusterMeas = [clusterDets.Measurement]; %get cluster points value;
        meas = mean(clusterMeas, 2); % 2 refer to column
        meas2D = [meas(1:2); meas(4:5)]; %compute mean range and velocity
        i = i + 1;
        detectionClusters{i} = detections{detInds(1)}; %record cluster information
        detectionClusters{i}.Measurement = meas2D;
        leftToCheck(clusterInds) = []; %delete found points.
    end
    detectionClusters(i + 1:end) = []; %delete unused space
    for i = 1:numel(detectionClusters)
        measNoise(1:2, 1:2) = vehicleSize^2 * eye(2);
        measNoise(3:4, 3:4) = eye(2) * 100 * vehicleSize^2;
        detectionClusters{i}.MeasurementNoise = measNoise;
    end
end
function BEP = createDemoDisplay(egoCar, sensors)
    hFigure = figure('Position', [0, 0, 1200, 640], 'Name', 'Sensor Fusion with Synthetic Data Example');
    movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top
    hCarViewPanel = uipanel(hFigure, 'Position', [0 0 0.5 0.5], 'Title', 'Chase Camera View');
    hCarPlot = axes(hCarViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot);
    hTopViewPanel = uipanel(hFigure, 'Position', [0 0.5 0.5 0.5], 'Title', 'Top View');
    hCarPlot = axes(hTopViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot, 'ViewHeight', 130, 'ViewLocation', [0 0], 'ViewPitch', 90);
    hBEVPanel = uipanel(hFigure, 'Position', [0.5 0 0.5 1], 'Title', 'Bird''s-Eye Plot');
    hBEVPlot = axes(hBEVPanel);
    frontBackLim = 60;
    BEP = birdsEyePlot('Parent', hBEVPlot, 'Xlimits', [-frontBackLim frontBackLim], 'Ylimits', [-35 35]);
    for i = 1:6
        cap = coverageAreaPlotter(BEP, 'FaceColor', 'red', 'EdgeColor', 'red');
        plotCoverageArea(cap, sensors{i}.SensorLocation, ...
            sensors{i}.MaxRange, sensors{i}.Yaw, sensors{i}.FieldOfView(1));
    end
    detectionPlotter(BEP, 'DisplayName', 'radar', 'MarkerEdgeColor', 'red');
    laneMarkingPlotter(BEP, 'DisplayName', 'lane markings');
    trackPlotter(BEP, 'DisplayName', 'track', 'HistoryDepth', 10);
    axis(BEP.Parent, 'equal');
    xlim(BEP.Parent, [-frontBackLim frontBackLim]);
    ylim(BEP.Parent, [-40 40]);
    outlinePlotter(BEP, 'Tag', 'Ground truth');
end
function updateBEP(BEP, egoCar, detections, confirmedTracks, psel, vsel)
    [lmv, lmf] = laneMarkingVertices(egoCar);
    plotLaneMarking(findPlotter(BEP, 'DisplayName', 'lane markings'), lmv, lmf);
    [position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
    plotOutline(findPlotter(BEP, 'Tag', 'Ground truth'), position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);
    N = numel(detections);
    detPos = zeros(N, 2);
    isRadar = true(N, 1);
    for i = 1:N
        detPos(i, :) = detections{i}.Measurement(1:2)';
        if detections{i}.SensorIndex > 6
            isRadar(i) = false;
        end
    end
    plotDetection(findPlotter(BEP, 'DisplayName', 'radar'), detPos(isRadar, :));
    trackIDs = {confirmedTracks.TrackID};
    labels = cellfun(@num2str, trackIDs, 'UniformOutput', false);
    [tracksPos, tracksCov] = getTrackPositions(confirmedTracks, psel);
    tracksVel = getTrackVelocities(confirmedTracks, vsel);
    plotTrack(findPlotter(BEP, 'DisplayName', 'track'), tracksPos, tracksVel, tracksCov, labels);
end
