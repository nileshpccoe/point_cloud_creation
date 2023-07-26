import blenderproc as bproc
import numpy as np
import math
import cv2
import h5py
import os

try:
    # Initialize BlenderProc
    bproc.init()

    # Load the object from a Blender file
    obj = bproc.loader.load_blend("C:/Users/Nilesh Patil/OneDrive/Desktop/Desktop Files/living Room .blend")

    # Create a light
    light = bproc.types.Light()
    light.set_type("SUN")
    light.set_location([0,0, 1.99133])
    light.set_energy(30.0)
    # Define circular path parameters
    center = [0, 0, 1.10]  # Center point of the circular path
    radius = 0.90  # Radius of the circular path
    num_positions = 3  # Number of camera positions around the cir cle

    # Set up circular path and camera positions
    for i in range(num_positions):
        # Calculate the angle for the camera position
        angle = 2 * math.pi * i / num_positions

        # Calculate the camera location based on the circular path
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        z = center[2]
        location = [x, y, z]

        rotation_angle = (math.radians(90), 0, math.atan2(y - center[1], x - center[0]) + math.radians(90))
        # Set the camera to world matrix
        cam2world_matrix = bproc.math.build_transformation_mat(location, rotation_angle)

        # Camera resolution
        color_image_width, color_image_height = 1280, 720  # Set the desired color image resolution
        bproc.camera.set_resolution(image_width=color_image_width, image_height=color_image_height)
        bproc.camera.set_intrinsics_from_blender_params(lens=20, lens_unit="MILLIMETERS")

        # Add the camera positions
        bproc.camera.add_camera_pose(cam2world_matrix)
        # Writing the images into PNG format
    

    # Activate depth rendering
    bproc.renderer.enable_depth_output(activate_antialiasing=True)

    data = bproc.renderer.render()

    try:
        # Write the rendering into an HDF5 file
        bproc.writer.write_hdf5("output/", data)
    except Exception as e:
        print("Failed to write HDF5 file:", e)
   
    try:
        # Create the folders
        folder_names = ["rgb_images", "depth_array_resized", "depth_rgb_images","depth_array_unresized_npy","depth_array_npz"]
        current_directory = os.getcwd()
        for folder_name in folder_names:
            folder_path = os.path.join(current_directory, folder_name)
            os.mkdir(folder_path, mode=0o666)
    except OSError as e:
        print("Failed to create folders:", e)

    try:
        # Define the directory containing the HDF5 files
        output_directory = "output/"

        # Get the list of HDF5 files in the directory
        dir_list = os.listdir(output_directory)

        # Process each HDF5 file
        for i, filename in enumerate(dir_list):
            # Open the HDF5 file
            filepath = os.path.join(output_directory, filename)
            with h5py.File(filepath, "r") as f:
                # Retrieve the color
                color = np.array(f["colors"])
                # Convert color to RGB
                color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
                # Save the RGB image as a PNG file
                rgb_filepath = f"rgb_images/rgb_{i+1:04d}.png"
                cv2.imwrite(rgb_filepath, color)

                # Retrieve the depth
                depth = np.array(f["depth"])
                depth_map=depth.copy()

                # Resize the depth image to match the size of the color image
                depth_resized = cv2.resize(depth, (color_image_width, color_image_height))

                # Normalize the depth values to fit within the range [0, 1]
                depth_normalized = cv2.normalize(depth_resized, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)

                # Apply colormap to the depth map
                depth_colormap = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), cv2.COLORMAP_JET)

                # Save the depth colormap as a PNG file
                depth_colormap_filepath = f"depth_rgb_images/depth_rgb_{i+1:04d}.png"
                cv2.imwrite(depth_colormap_filepath, depth_colormap)

                # Save the depth array as a NumPy file
                depth_array_filepath = f"depth_array_resized/depth_resized_{i+1:04d}.npy"
                with open(depth_array_filepath, "wb") as f:
                    np.save(f, depth_resized)
                
                # Save the depth array as a NumPy file
                
                np.save(f"depth_array_unresized_npy/depth_{i+1:04d}",depth_map)
                
                depth_array__unscaled_filepath = f"depth_array_npz/depth_npz_{i+1:04d}.npz"
                np.savez_compressed(depth_array__unscaled_filepath , depth=depth_map, intrinsics=None)
            
                

                # Print progress
                print(f"Processed file {i+1}/{len(dir_list)}")
    except Exception as e:
        print("Failed to process HDF5 files:", e)

except Exception as e:
    print("Error:", e)
