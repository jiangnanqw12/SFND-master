
import numpy as np

def cluster_detections(detections, vehicle_size):
    N = len(detections)
    distances = np.zeros((N, N))
    for i in range(N):
        for j in range(i+1, N):
            if detections[i].SensorIndex == detections[j].SensorIndex:
                distances[i, j] = np.linalg.norm(detections[i].Measurement[:2] - detections[j].Measurement[:2])
            else:
                distances[i, j] = np.inf
    left_to_check = list(range(N))
    i = 0
    detection_clusters = []
    while left_to_check:
        under_consideration = left_to_check[0]
        cluster_inds = np.where(distances[under_consideration, left_to_check] < vehicle_size)[0]
        det_inds = [left_to_check[k] for k in cluster_inds]
        cluster_dets = [detections[k] for k in det_inds]
        cluster_meas = np.array([det.Measurement for det in cluster_dets]).T
        meas = np.mean(cluster_meas, axis=1)
        meas_2d = np.array([meas[:2], meas[3:5]])
        i += 1
        detection_clusters.append(detections[det_inds[0]])
        detection_clusters[-1].Measurement = meas_2d
        left_to_check = [k for k in left_to_check if k not in det_inds]
    detection_clusters = detection_clusters[:i]
    for cluster in detection_clusters:
        meas_noise = np.zeros((4, 4))
        meas_noise[:2, :2] = vehicle_size**2 * np.eye(2)
        meas_noise[2:, 2:] = 100 * vehicle_size**2 * np.eye(2)
        cluster.MeasurementNoise = meas_noise
    return detection_clusters
