"""
Calibration utilities
"""
import os
import sys
import cv2
import logging
import apriltag
import numpy as np

from vito import cam_projections as prjutils

################################################################################
# Pose estimation
################################################################################

def rotx3d180():
    """180 deg rotation around x axis"""
    return np.array([
        [1.0,  0.0,  0.0],
        [0.0, -1.0,  0.0],
        [0.0,  0.0, -1.0]], dtype=np.float64)


def translation_change(t1, t2):
    """L2 distance between the two vectors."""
    return np.linalg.norm(t1 - t2)


def pose_from_apriltag(detection, detector, K, tag_size_mm, return_reprojection_error=False):
    # Camera params: (fx, fy, cx, cy)
    camera_params = (K[0,0], K[1,1], K[0,2], K[1,2])
    pose, initial_reprj_err, final_reprj_err = detector.detection_pose(detection, camera_params, tag_size_mm)

    # We want the tag's/world z-axis pointing up, so R = R_tag * Rx(180)
    Mx180 = np.row_stack((np.column_stack((rotx3d180(), np.array([0., 0., 0.]).reshape((3,1)))), np.array([0., 0., 0., 1.])))
    if return_reprojection_error:
        return prjutils.matmul(pose, Mx180), final_reprj_err
    else:
        return prjutils.matmul(pose, Mx180)


def split_pose(M):
    R = M[:3,:3]
    t = M[:3,3]
    return R, t.reshape((3,1))


def inverse_pose(M):
    return np.linalg.inv(M)


def between_tag_transformation(M1, M2):
    """Computes the transformation from tag 1's coordinate system to tag 2's coord. sys, i.e. [M_inverse tag2 to cam] * [M tag1 to cam]"""
    return prjutils.matmul(np.linalg.inv(M2), M1)


import collections
class TagTransformationGraph:
    def __init__(self):
        self.edges = collections.defaultdict(list)
        self.weights = dict()
        self.tag_transformations = dict()
        self.camera_transformations = dict()

    def add_tag_pose(self, cam_id, tag1, tag2):
        # Extrinsics tag 1 to camera
        id1, M1, err1 = tag1
        self.camera_transformations[(cam_id, id1)] = M1
        # Extrinsics tag 2 to camera
        id2, M2, err2 = tag2
        weight = err1
        if id1 != id2:
            self.camera_transformations[(cam_id, id2)] = M2
            # Transformations between tags
            M1to2 = between_tag_transformation(M1, M2)        
            weight += err2
        else:
            M1to2 = np.eye(4)

        self.edges[id1].append(id2)
        self.weights[(id1, id2)] = weight
        self.tag_transformations[(id1, id2)] = M1to2

    def get_transformation(self, cam_id, tag_id1, tag_id2):
        if tag_id1 == tag_id2:
            if (cam_id, tag_id1) in self.camera_transformations:
                return self.camera_transformations[(cam_id, tag_id1)]
            else:
                return None
        path = self.__dijkstra(tag_id1, tag_id2)
        if not path:
            return None
        # Backtrack:
        # Note that the path is reversed, e.g. tag 2 => tag 5 => tag 0 (world reference frame)
        # So, we need to compute: M_t2->cam * M_t5->t2 * M_t0->t5
        if (cam_id, tag_id1) not in self.camera_transformations:
            return None
        M = self.camera_transformations[(cam_id, tag_id1)]
        
        for i in range(len(path)-1):
            tfrom = path[i]
            tto = path[i+1]
            Mt = self.tag_transformations[(tto, tfrom)]
            M = prjutils.matmul(M, Mt)
            # print('  CAM {}, PATH {} => {}'.format(cam_id, tfrom, tto))
        return M
    
    def __dijkstra(self, tag_from, tag_to):
        # shortest paths is a dict of nodes
        # whose value is a tuple of (previous node, weight)
        shortest_paths = {tag_from: (None, 0)}
        current_node = tag_from
        visited = set()
        
        while current_node != tag_to:
            visited.add(current_node)
            destinations = self.edges[current_node]
            weight_to_current_node = shortest_paths[current_node][1]

            for next_node in destinations:
                weight = self.weights[(current_node, next_node)] + weight_to_current_node
                # weight = 1 + weight_to_current_node
                if next_node not in shortest_paths:
                    shortest_paths[next_node] = (current_node, weight)
                else:
                    current_shortest_weight = shortest_paths[next_node][1]
                    if current_shortest_weight > weight:
                        shortest_paths[next_node] = (current_node, weight)
            
            next_destinations = {node: shortest_paths[node] for node in shortest_paths if node not in visited}
            if not next_destinations:
                return None # No route possible
            # next node is the destination with the lowest weight
            current_node = min(next_destinations, key=lambda k: next_destinations[k][1])
        
        # Work back through destinations in shortest path
        path = []
        while current_node is not None:
            path.append(current_node)
            next_node = shortest_paths[current_node][0]
            current_node = next_node
        # Reverse path
        path = path[::-1]
        return path


import collections
class ExtrinsicsHistory:
    """
    Minimum tag ID = world origin
    """
    def __init__(self, camera_intrinsics, tag_detectors, tag_size_mm, max_history_length, threshold_rotation, threshold_translation):
        """camera_intrinsics = [K_0, K_1, ...]
        tag_detectors = [apriltag detector for cam 0, ... for cam 1, etc.]
        tag_size_mm = scaling factor
        max_history_length - for how many past frames should we store the poses
        threshold_rotation in degrees to consider a tag stable (over time)
        threshold_translation in mm to consider a tag stable (over time)
        """
        self.num_cameras = len(camera_intrinsics)
        self.camera_intrinsics = camera_intrinsics
        self.tag_detectors = tag_detectors
        self.tag_size_mm = tag_size_mm
        self.detection_history = dict()
        self.pose_history = dict()
        self.reprojection_error_history = dict()
        self.center_history = dict()
        self.seen_tag_ids = set() # Stores which tag IDs have been seen (by any camera!)
        self.tag_transformations = TagTransformationGraph()
        self.threshold_rotation = 0.1 # degrees
        self.threshold_translation = 2 # mm
        self.skip_camera = list()
        for cam_id in range(self.num_cameras):
            self.skip_camera.append(camera_intrinsics[cam_id] is None or tag_detectors[cam_id] is None)
            self.detection_history[cam_id] = collections.deque(maxlen=max_history_length)
            self.pose_history[cam_id] = collections.deque(maxlen=max_history_length)
            self.center_history[cam_id] = collections.deque(maxlen=max_history_length)
            self.reprojection_error_history[cam_id] = collections.deque(maxlen=max_history_length)

    def update_tags(self, cam_id, detections):
        """Get pose from tag, store center/reprojection error/transformation"""
        if self.skip_camera[cam_id]:
            logging.getLogger().warning('[ExtrinsicsHistory] update_tags() called with invalid (i.e. skipped) cam_id ({})'.format(cam_id))
            return
        self.detection_history[cam_id].append(detections)
        poses = dict()
        centers = dict()
        errors = dict()
        K = self.camera_intrinsics[cam_id]
        for detection in detections:
            M, err = pose_from_apriltag(detection, self.tag_detectors[cam_id], K, self.tag_size_mm, return_reprojection_error=True)
            poses[detection.tag_id] = M
            centers[detection.tag_id] = detection.center
            errors[detection.tag_id] = err
            self.seen_tag_ids.add(detection.tag_id)

        self.center_history[cam_id].append(centers)
        self.pose_history[cam_id].append(poses)
        self.reprojection_error_history[cam_id].append(errors)


    def update_coordinate_transformations(self):
        # Clear previous transformations
        self.tag_transformations = TagTransformationGraph()
        # Iterate over all cameras, build inter-tag transformation if multiple tags were visible
        
        for cam_id in range(self.num_cameras):
            if self.skip_camera[cam_id]:
                continue
            latest_cam_poses = self.pose_history[cam_id]
            if not latest_cam_poses:
                continue
            latest_cam_poses = latest_cam_poses[-1]
            latest_reprojection_errors = self.reprojection_error_history[cam_id][-1]
            tag_ids = list(latest_cam_poses.keys())
            for from_tag in tag_ids:
                # Transformation between tag ('from') and camera
                Mf = latest_cam_poses[from_tag]
                errf = latest_reprojection_errors[from_tag]
                for to_tag in tag_ids:
                    # Transformation between tag 'to' and camera
                    Mt = latest_cam_poses[to_tag]
                    errt = latest_reprojection_errors[to_tag]
                    self.tag_transformations.add_tag_pose(cam_id, (from_tag, Mf, errf), (to_tag, Mt, errt))
        
    
    def get_pose_changes(self, cam_id):
        #TODO skip_camera!
        pose_history = self.pose_history[cam_id]
        center_history = self.center_history[cam_id]
        rotation_changes = dict()
        translation_changes = dict()
        center_changes = dict()
        for i in range(len(pose_history)-1, 0, -1): # deliberately > 0
            for tag_id in pose_history[i]:
                Rc, tc = split_pose(pose_history[i][tag_id])
                cc = center_history[i][tag_id]
                if tag_id in pose_history[i-1]:
                    Rp, tp = split_pose(pose_history[i-1][tag_id])
                    if tag_id not in rotation_changes:
                        rotation_changes[tag_id] = list()
                    rotation_changes[tag_id].append(prjutils.compare_rotation_matrices(Rp, Rc))

                    if tag_id not in translation_changes:
                        translation_changes[tag_id] = list()
                    translation_changes[tag_id].append(translation_change(tp, tc))

                    cp = center_history[i-1][tag_id]
                    if tag_id not in center_changes:
                        center_changes[tag_id] = list()
                    center_changes[tag_id].append(translation_change(np.array(cc), np.array(cp)))
        def mean(x):
            return sum(x)/len(x)
        return {tag_id: (mean(rotation_changes[tag_id]), mean(translation_changes[tag_id]), mean(center_changes[tag_id])) for tag_id in rotation_changes}
    

    def is_pose_stable(self, cam_id):
        changes = self.get_pose_changes(cam_id)
        if len(changes) == 0:
            return False
        return all([changes[tag_id][0] < self.threshold_rotation and changes[tag_id][1] < self.threshold_translation for tag_id in changes])
        

    def get_change_strings(self, cam_id):
        changes = self.get_pose_changes(cam_id)
        return ['Tag {:2d}: {:.2f} deg, {:.1f} mm, {:.2f} px'.format(tag_id, changes[tag_id][0], changes[tag_id][1], changes[tag_id][2]) for tag_id in changes]


    def get_tag_extrinsics(self, cam_id):
        """Return the extrinsics w.r.t. each detected tag."""
        # TODO maybe robustify temporally
        return self.pose_history[cam_id][-1]


    def get_world_extrinsics(self, cam_id):        
        # Check if we have pose estimates available
        pose = self.pose_history[cam_id][-1]
        if pose is None:
            return None
        # Check which tag defines the world reference frame
        tag_ids = list(pose.keys())
        if len(tag_ids) == 0:
            return None
        tag_id_origin = min(self.seen_tag_ids)
        if tag_id_origin in tag_ids:
            return pose[tag_id_origin]
        else:
            pose_changes = self.get_pose_changes(cam_id)
            # Sort the visible tags by their rotation change and return the best world to camera transformation (i.e. the most stable one)
            for tag_id, rotation_delta in sorted(pose_changes.items(), key=lambda kv: kv[1][0]):
            ### Deprecated (using reprojection error, which might be low for blurred - i.e. moving - tags)
            ### for tag_id, reprj_err in sorted(self.reprojection_error_history[cam_id][-1].items(), key=lambda kv: kv[1]):
                transform = self.tag_transformations.get_transformation(cam_id, tag_id, tag_id_origin)
                ### Debug output
                # print('LOOKING UP cam {}, tag {}, err {}'.format(cam_id, tag_id, rotation_delta))
                if transform is not None:
                    return transform
            return None

#TODO add class which works with vito's capture (check if is depth, etc
class ExtrinsicsAprilTag(object):
    def __init__(self, 
            capture, 
            tag_family, tag_size_mm,
            refine_edges=True,
            refine_decode=True,
            refine_pose=True,
            debug=False,
            quad_decimate=1.0,
            quad_contours=True,
            grid_limits=None,
            pose_history_length=10,
            pose_threshold_rotation=0.1,
            pose_threshold_translation=2.0):
        self._num_streams = capture.num_streams()
        self._intrinsics = list()
        self._detectors = list()
        self._tag_family = tag_family
        self._tag_size_mm = tag_size_mm
        self._pose_history_length = pose_history_length
        self._pose_threshold_rotation = pose_threshold_rotation
        self._pose_threshold_translation = pose_threshold_translation
        self._frame_converter = list()
        for i in range(self._num_streams):
            K = capture.intrinsics(i)
            self._intrinsics.append(K)
            if K is None:
                logging.getLogger().error('[ExtrinsicsAprilTag] No intrinsics for stream "{}"'.format(capture.frame_label(i)))
                self._detectors.append(None)
                self._frame_converter.append(self._cvt_as_is)
            else:
                if capture.is_unknown_type(i) or capture.is_image(i) or capture.is_infrared(i):
                    if capture.is_unknown_type(i):
                        logging.getLogger().warning('[ExtrinsicsAprilTag] Stream "{}" has unknown frame type, assuming it to be monocular!'.format(capture.frame_label(i)))
                    fx = K[0,0]
                    fy = K[1,1]
                    cx = K[0,2]
                    cy = K[1,2]
                    opt = apriltag.DetectorOptions(families=tag_family,
                        refine_edges=refine_edges, refine_decode=refine_decode, refine_pose=refine_decode,
                        debug=debug, quad_decimate=quad_decimate, quad_contours=quad_contours)
                    opt.camera_params = (fx, fy, cx, cy)
                    opt.tag_size = tag_size_mm
                    detector = apriltag.Detector(opt)
                    self._detectors.append(detector)
                    if capture.is_infrared(i):
                        self._frame_converter.append(self._cvt_as_is)
                    elif capture.is_rgb(i):
                        self._frame_converter.append(self._cvt_rgb2gray)
                    else:
                        self._frame_converter.append(self._cvt_rgb2gray)
                else:
                    logging.getLogger().info('[ExtrinsicsAprilTag] Skipping AprilTag detection for stream "{}".'.format(capture.frame_label(i)))
                    self._detectors.append(None)
                    self._frame_converter.append(self._cvt_as_is)
        self._extrinsics_history = ExtrinsicsHistory(self._intrinsics, 
            self._detectors, tag_size_mm, pose_history_length, pose_threshold_rotation, pose_threshold_translation)

    
    def process_frameset(self, frameset):
        assert len(frameset) == len(self._detectors)
        detections = list()
        for i in range(len(frameset)):
            # print('Stream i: ', i, frameset[i].dtype)
            if self._detectors[i] is None:
                detections.append(None)
                continue
            gray = self._frame_converter[i](frameset[i])
            frame_detections = self._detectors[i].detect(gray)
            detections.append(frame_detections)
            self._extrinsics_history.update_tags(i, frame_detections)
            if len(frame_detections) > 0:
                print('Stream i: {} has {} tag detections!'.format(i, len(frame_detections)))
        # Update transformations between tags
        self._extrinsics_history.update_coordinate_transformations()

    def _cvt_as_is(self, frame):
        return frame

    def _cvt_rgb2gray(self, frame):
        return self._color2gray(frame, cv2.COLOR_RGB2GRAY)

    def _cvt_bgr2gray(self, frame):
        return self._color2gray(frame, cv2.COLOR_BGR2GRAY)

    def _color2gray(self, frame, mode):
        if frame.ndim == 2 or frame.shape[2] == 1:
            return frame
        return cv2.cvtColor(frame, mode)
            
    #TODO set up detectors, skip if depth, transformation (R,t) to reference frame
    #TODO expose estimated poses and deviations
    #TODO visualize