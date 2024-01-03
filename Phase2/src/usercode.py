import numpy as np
import cv2
import bpy
import os
import copy


class state_machine:
    def __init__(self):
        # User code executes every dt time in seconds
        self.dt = 0.050

        # location of each tag
        self.waypoints = [
            [-1.7, 0.8, 1.5],
            [-1.2, -1.6, 1.5],
            [0.9, 0.7, 1.5],
        ]

        # Current waypoint
        self.currentWP = [0, -0.2, 1.5]
        self.acceptableError = 0.1

    def step(self, time, currpos):
        """
        Input: time, current position in blender frame (x, y, z)
        Output: desired position
            Si unit unless specified otherwise
        """
        dX, dY, dZ = self.currentWP
        X, Y, Z = currpos
        distanceError = pow((dX - X), 2) + pow((dY - Y), 2) + pow((dZ - Z), 2)
        if distanceError < self.acceptableError:
            if self.currentWP not in self.waypoints:
                self.currentWP = self.waypoints[0]
            else:
                currentIndex = self.waypoints.index(self.currentWP)
                self.currentWP = self.waypoints[currentIndex + 1]

        xyz_desired = self.currentWP
        return xyz_desired

    def fetchLatestImage(self):
        # Fetch image - renders the camera, saves the rendered image to a file and reads from it.
        path_dir = bpy.data.scenes["Scene"].node_tree.nodes["File Output"].base_path

        # Render Drone Camera
        cam = bpy.data.objects["DownCam"]
        bpy.context.scene.camera = cam
        bpy.context.scene.render.filepath = os.path.join(path_dir, "DownCam_latest.png")
        bpy.ops.render.render(write_still=True)

        return cv2.imread(bpy.context.scene.render.filepath)
