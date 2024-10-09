# detect ARUCO markers and estimate their positions
import numpy as np
import cv2
import os, sys

sys.path.insert(0, "{}/util".format(os.getcwd()))
import util.measure as measure

class aruco_detector:
    def __init__(self, robot, marker_length=0.07):
        self.camera_matrix = robot.camera_matrix
        self.distortion_params = robot.camera_dist

        self.marker_length = marker_length
        self.aruco_params = cv2.aruco.DetectorParameters() # updated to work with newer OpenCV
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100) # updated to work with newer OpenCV
    
    def detect_marker_positions(self, img, flag=False):
        # Perform detection
        corners, ids, rejected = cv2.aruco.detectMarkers(
            img, self.aruco_dict, parameters=self.aruco_params)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.distortion_params)
        # rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.distortion_params) # use this instead if you got a value error

        # if tvecs is not None:
        #     tvecs[:, :, 2] = 0.898 * tvecs[:, :, 2] + 0.0147
        #     tvecs[:, :, 0] = 0.9256 * tvecs[:, :, 0] - 0.0156

        if ids is None:
            return [], img

        # Compute the marker positions
        measurements = []
        seen_ids = []
        for i in range(len(ids)):
            if ids[i] > 10: # remove unknown aruco
                continue

            idi = ids[i,0]
            # Some markers appear multiple times but should only be handled once.
            if idi in seen_ids:
                continue
            else:
                seen_ids.append(idi)

            lm_tvecs = tvecs[ids==idi].T

            #test
            # print(idi)
            # this section of code adjusts lm_tvecs to the center of the aruco markers
            # the for loop is due to lm_tvecs growing in size (possible as it sees multiple faces of a marker)
            for vec in range(0,lm_tvecs.shape[1]):
                temp_tvec = lm_tvecs[:,vec]
                # print("temp")
                # print(temp_tvec)
                rot_mat, _ = cv2.Rodrigues(rvecs[vec])
                transform_mat = np.eye(4)
                transform_mat[0:3, 0:3] = rot_mat

                transform_mat[0:3,3] = temp_tvec.flatten()
                vector = transform_mat @ np.array([[0], [0], [-0.04], [1]])
                # print("vector")
                # print(vector)
                lm_tvecs[:, vec] = vector[0:3].flatten()


            lm_bff2d = np.block([[lm_tvecs[2, :]], [-lm_tvecs[0, :]]])
            lm_bff2d = np.mean(lm_bff2d, axis=1).reshape(-1, 1)

            # this was code testing if we should reduce covariance of measurement based off starting position
            # we didn't see much change so don't allow the flag variable to change
            # hard to really compare runs when they're so dependent on driving style
            lm_measurement = measure.Marker(lm_bff2d, idi)
            measurements.append(lm_measurement)
        
        # Draw markers on image copy
        img_marked = img.copy()
        cv2.aruco.drawDetectedMarkers(img_marked, corners, ids)

        return measurements, img_marked
