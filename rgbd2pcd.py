import cv2
import open3d as o3d
import numpy as np 
import pathlib



def scale_point_cloud(pcd, scale_factor):
    pcd_scaled = pcd # Create a copy of the original point cloud

    # Scale the X, Y, and Z coordinates of each point by the scale_factor
    pcd_scaled.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) * scale_factor)

    return pcd_scaled

def create_point_cloud_blender(rgb_file, depth_file, intrinsic, extrinsic):

    file_extension=pathlib.Path(depth_file).suffix
    print(file_extension)
    if file_extension==".npz":
        with np.load(depth_file) as depth_npz:
            depthimg = depth_npz['depth']
        
    else:
         with open(depth_file, 'rb') as f:
            depthimg = np.load(f)
    depth_map = depthimg.copy()
    depth_map = np.float32(depth_map)

    rgbImg = cv2.imread(rgb_file)
    rgbImg = cv2.cvtColor(rgbImg, cv2.COLOR_RGB2BGR)

    color_ = o3d.geometry.Image(rgbImg)
    depth_ = o3d.geometry.Image(depth_map)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_, depth_, depth_scale=3.0, depth_trunc=3.00000, convert_rgb_to_intensity=False)

    extrinsic = np.asarray(np.loadtxt(extrinsic, dtype=np.float64))
    camera_intrinsic_mat = np.asarray(np.loadtxt(intrinsic, dtype=np.float64))

    # Convert intrinsic matrix to PinholeCameraIntrinsic
    intrinsic = o3d.camera.PinholeCameraIntrinsic(1280, 720, camera_intrinsic_mat[0][0], camera_intrinsic_mat[1][1], camera_intrinsic_mat[0][2], camera_intrinsic_mat[1][2])
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic, extrinsic)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    R = pcd.get_rotation_matrix_from_xyz((np.pi * 0.5, np.pi * 1.5, 0))
    pcd = pcd.rotate(R, center=(0, 0, 0))
    print(camera_intrinsic_mat)

    # Scale the point cloud  as the original
    scale_factor = 5.0
    pcd_scaled = scale_point_cloud(pcd, scale_factor)

    o3d.visualization.draw_geometries([pcd_scaled])
    return pcd


rgb_file_path="rgb_images/rgb_0001.png"

depth_file_path="depth_array_unresized_npy/depth_0001.npy"


camera_extrinsic_path = "living_room_folder/camera_extrinsic0.txt"
camera_intrinsic_path = "living_room_folder/camera_intrinsic.txt"

point_cloud = create_point_cloud_blender(rgb_file_path, depth_file_path, camera_intrinsic_path, camera_extrinsic_path)


file_path=f"point_cloud_depth_resized_0003.ply"
o3d.io.write_point_cloud(file_path,point_cloud)