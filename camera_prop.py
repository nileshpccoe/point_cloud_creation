import blenderproc as bproc
import numpy as np
import math
import cv2
import h5py
import os

# Initialize BlenderProc
bproc.init()

# Define circular path parameters
# center = [-1.10, -1.87, 1.12]  
center=[0,0,1.10]# Center point of the circular path
radius = 0.90  # Radius of the circular path
num_positions = 2  # Number of camera positions around the circle


# Set up circular path and camera positions
for i in range(num_positions):
    # Calculate the angle for the camera position
    angle = 2 * math.pi * i / num_positions

    # Calculate the camera location based on the circular path
    x = center[0] + radius * math.cos(angle)
    y = center[1] + radius * math.sin(angle)
    z = center[2]
    location = [x, y, z]

 
    rotation_angle =(math.radians(90), 0, math.atan2(y - center[1], x - center[0]) + math.radians(90))
    #set the camera to world matrix
    cam2world_matrix = bproc.math.build_transformation_mat(location, rotation_angle)
    #Camera resolution
    bproc.camera.set_resolution(image_width=1280, image_height=720)
    bproc.camera.set_intrinsics_from_blender_params(lens=50,lens_unit="MILLIMETERS")
    #add the camera postions
    bproc.camera.add_camera_pose(cam2world_matrix)
    #save the camera extrinsic into txt format 
    np.savetxt(f"camera_extrinsic{i}.txt",cam2world_matrix)

    # writing the images into png format
intrinsic_matrix=bproc.camera.get_intrinsics_as_K_matrix()
np.savetxt("camera_intrinsic.txt", intrinsic_matrix)
