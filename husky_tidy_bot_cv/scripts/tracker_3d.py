import numpy as np
import cv2
from kas_utils import get_depth_scale
import lap
from typing import List
import os
import json

class TrackedObject:
    next_tracking_id = 0

    def __init__(self, class_id, tracking_2d_id, pose, frame_id, point_cloud=None):
        self.class_id = class_id
        self.tracking_2d_id = tracking_2d_id
        self.pose = pose
        self.frame_id = frame_id
        self.dimensions = None
        self.point_cloud = point_cloud  # Добавляем point_cloud
        self.tracking_id = -1  
        self.next_tracking_id = TrackedObject.next_tracking_id  
        
        TrackedObject.next_tracking_id += 1
        self.tracklet_len = 1
        self.visible_without_updates = 0
        if point_cloud is not None:
            self.calculate_dimensions() 
        print("Init")
    
    def calculate_dimensions(self):
        if self.point_cloud is not None:
            # Calculate the minimum and maximum coordinates in each dimension
            min_coords = np.min(self.point_cloud, axis=0)
            max_coords = np.max(self.point_cloud, axis=0)
            # Calculate the dimensions as the difference between max and min coordinates
            self.dimensions = max_coords - min_coords

    def activate(self):
        assert self.tracking_id == -1
        self.tracking_id = self.next_tracking_id

    def update(self, update_object):
        assert self.tracking_id != -1
        assert self.class_id == update_object.class_id

        k = self.tracklet_len / (self.tracklet_len + 1)
        max_k = 0.8
        k = min(k, max_k)

        self.tracking_2d_id = update_object.tracking_2d_id
        self.pose = k * self.pose + (1 - k) * update_object.pose
        self.frame_id = update_object.frame_id
        self.point_cloud = update_object.point_cloud  # Обновляем point_cloud
        self.calculate_dimensions()

        self.tracklet_len += 1
        self.visible_without_updates = 0

        print(f'Updated TrackedObject {self.tracking_id} with data from another object.')

    def is_visible(self, camera_pose_inv, depth, K, D, margin=50, radius=10):
        pose_in_camera = np.matmul(camera_pose_inv, np.append(self.pose, 1))[:3]
        pose_z = pose_in_camera[2]
        if pose_z <= 0:
            return False

        pose_in_camera = np.expand_dims(pose_in_camera, axis=(0, 1))
        point, _ = cv2.projectPoints(pose_in_camera, np.zeros((3,)), np.zeros((3,)), K, D)
        point = point[0, 0].astype(int)
        u = point[0]
        v = point[1]
        height, width = depth.shape
        if u < margin or v < margin or u >= width - margin or v >= height - margin:
            return False

        min_u = max(u - radius, 0)
        min_v = max(v - radius, 0)
        max_u = min(u + radius + 1, width)
        max_v = min(v + radius + 1, height)
        depth_selected = depth[min_v:max_v, min_u:max_u]

        y, x = np.mgrid[min_v:max_v, min_u:max_u]
        dist_sqr = (y - v) ** 2 + (x - u) ** 2
        mask = dist_sqr <= radius ** 2
        depth_selected = depth_selected[mask]

        depth_selected = depth_selected * get_depth_scale(depth)

        shift = 0.10
        rate = np.count_nonzero(depth_selected + shift > pose_z) / depth_selected.size

        thresh = 0.5
        visibility = rate > thresh
        
        #print(f'Visibility check for object {self.tracking_id}: {visibility}')
        return visibility

class Tracker3D:
    def __init__(self, erosion_size, K, D):
        self.erosion_size = erosion_size
        if self.erosion_size > 0:
            self.erosion_element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                (2 * self.erosion_size + 1, 2 * self.erosion_size + 1),
                (self.erosion_size, self.erosion_size))
        self.K = K
        self.D = D

        self.frame_id = -1
        self.tracked_objects: List[TrackedObject] = list()

        assert np.all(self.D == 0), "Distorted images are not supported"
        print('Initialized Tracker3D')

    def reset(self):
        self.frame_id = -1
        self.tracked_objects = list()
        TrackedObject.next_tracking_id = 0
        print('Tracker3D reset')

    def save_to_json(self, directory):
        if not os.path.exists(directory):
            os.makedirs(directory)
        
        data = {
            "frame_id": int(self.frame_id),
            "tracked_objects": []
        }
        
        for obj in self.tracked_objects:
            obj_data = {
                "tracking_id": int(obj.tracking_id),
                "class_id": int(obj.class_id),
                "pose": obj.pose.tolist(),
                "point_cloud": [list(map(float, point)) for point in obj.point_cloud] if obj.point_cloud is not None else None,   # Сохраняем point_cloud
                "dimensions": obj.dimensions.tolist() if obj.dimensions is not None else None,
                "frame_id": int(obj.frame_id),
                "tracklet_len": int(obj.tracklet_len),
                "visible_without_updates": int(obj.visible_without_updates)
            }
            data["tracked_objects"].append(obj_data)

        filename = f"{directory}/frame_{self.frame_id}.json"
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)
    
    def update(self, camera_pose, depth, classes_ids, tracking_ids, masks_in_rois, rois):
        self.frame_id += 1

        objects_poses, point_clouds = self._get_objects_poses(depth, masks_in_rois, rois, camera_pose)
        valid = ~np.isnan(objects_poses[:, 0])
        classes_ids = classes_ids[valid]
        if len(tracking_ids) > 0:
            tracking_ids = tracking_ids[valid]
        masks_in_rois = None
        rois = None
        objects_poses = objects_poses[valid]
        point_clouds = [pc for pc, v in zip(point_clouds, valid) if v]

        if len(tracking_ids) > 0:
            new_objects = [TrackedObject(class_id, tracking_2d_id, pose, self.frame_id, point_cloud)
                for class_id, tracking_2d_id, pose, point_cloud in zip(classes_ids, tracking_ids, objects_poses, point_clouds)]
        else:
            new_objects = [TrackedObject(class_id, -1, pose, self.frame_id, point_cloud)
                for class_id, pose, point_cloud in zip(classes_ids, objects_poses, point_clouds)]

        dists = self._compute_distances_matrix(new_objects)
        self._fuse_class_id(dists, new_objects)
        if len(tracking_ids) > 0:
            self._fuse_tracking_2d_id(dists, new_objects)

        new_to_tracked, tracked_to_new = self._match(dists)

        camera_pose_inv = np.linalg.inv(camera_pose)
        for j, (i, tracked_object) in enumerate(zip(tracked_to_new, self.tracked_objects)):
            if i != -1:
                tracked_object.update(new_objects[i])
            elif tracked_object.is_visible(camera_pose_inv, depth, self.K, self.D):
                tracked_object.visible_without_updates += 1

        max_lost_frames = 2
        tracked_objects_keep = list()
        for tracked_object in self.tracked_objects:
            if tracked_object.visible_without_updates <= max_lost_frames:
                tracked_objects_keep.append(tracked_object)
        self.tracked_objects = tracked_objects_keep

        for j, new_object in zip(new_to_tracked, new_objects):
            if j == -1:
                new_object.activate()
                self.tracked_objects.append(new_object)

        self.save_to_json("/resources/data/point_clouds")

    def _get_objects_poses(self, depth, masks_in_rois, rois, camera_pose):
        objects_poses_in_camera, point_clouds_in_camera = self._get_objects_poses_in_camera(
            depth, masks_in_rois, rois)
        R = camera_pose[:3, :3]
        t = camera_pose[:3, 3]
        
        # Применение преобразования к позам объектов
        objects_poses = np.matmul(R, objects_poses_in_camera.T).T + t
        
        # Применение преобразования к облакам точек
        transformed_point_clouds = []
        for point_cloud in point_clouds_in_camera:
            if point_cloud is not None:
                transformed_point_cloud = np.matmul(R, point_cloud.T).T + t
                transformed_point_clouds.append(transformed_point_cloud)
            else:
                transformed_point_clouds.append(None)
        
        return objects_poses, transformed_point_clouds

    def _get_objects_poses_in_camera(self, depth, masks_in_rois, rois):
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]
        depth_scale = get_depth_scale(depth)
        object_poses = list()
        point_clouds = list()
        
        for mask_in_roi, roi in zip(masks_in_rois, rois):
            if self.erosion_size > 0:
                mask_in_roi = cv2.erode(mask_in_roi, self.erosion_element,
                    borderType=cv2.BORDER_CONSTANT, borderValue=0)
            z = depth[roi][mask_in_roi != 0] * depth_scale
            valid = (z > 0) & np.isfinite(z)
            if np.count_nonzero(valid) < 15:
                object_pose = np.array([np.nan] * 3)
                object_poses.append(object_pose)
                point_clouds.append(None)
                continue

            v, u = np.where(mask_in_roi)
            v += roi[0].start
            u += roi[1].start

            z = z[valid]
            u = u[valid]
            v = v[valid]
            x = (u - cx) / fx * z
            y = (v - cy) / fy * z
            
            point_cloud = np.vstack((x, y, z)).T
            object_pose = np.array([x.mean(), y.mean(), z.mean()])
            object_poses.append(object_pose)
            point_clouds.append(point_cloud)

        if len(object_poses) > 0:
            object_poses = np.array(object_poses)
        else:
            object_poses = np.empty((0, 3))

        return object_poses, point_clouds



    def _compute_distances_matrix(self, new_objects: List[TrackedObject]):
        dists = np.empty((len(new_objects), len(self.tracked_objects)), dtype=float)
        for i, new_object in enumerate(new_objects):
            for j, tracked_object in enumerate(self.tracked_objects):
                dist = np.sum(np.square(new_object.pose - tracked_object.pose))
                dists[i, j] = dist
        print(f'Distance matrix computed: {dists}')
        return dists

    def _fuse_class_id(self, dists, new_objects: List[TrackedObject]):
        for i, new_object in enumerate(new_objects):
            for j, tracked_object in enumerate(self.tracked_objects):
                if new_object.class_id != tracked_object.class_id:
                    dists[i, j] = np.inf
        print(f'Class ID fusion applied: {dists}')

    def _fuse_tracking_2d_id(self, dists, new_objects: List[TrackedObject]):
        for i, new_object in enumerate(new_objects):
            for j, tracked_object in enumerate(self.tracked_objects):
                if new_object.tracking_2d_id == tracked_object.tracking_2d_id and \
                        new_object.tracking_2d_id != -1:
                    dists[i, j] = 0
        print(f'Tracking 2D ID fusion applied: {dists}')

    def _match(self, dists):
        if dists.size == 0:
            new_to_tracked = np.full((dists.shape[0],), -1, dtype=int)
            tracked_to_new = np.full((dists.shape[1],), -1, dtype=int)
            print('Empty distance matrix, no matches.')
            return new_to_tracked, tracked_to_new

        max_range = 0.25
        new_to_tracked, tracked_to_new = lap.lapjv(dists, cost_limit=(max_range * max_range), extend_cost=True, return_cost=False)
        print(f'Matching result: {new_to_tracked}, {tracked_to_new}')
        return new_to_tracked, tracked_to_new
