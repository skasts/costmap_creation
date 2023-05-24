import open3d as o3d
import numpy as np
from PIL import Image

def pointcloud_to_birdseye(pcd_file, resolution, output_file):
    # Load the PCD file
    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(pcd.points)

    # Cut off points outside a certain rectangle
    boundary = [-100, 100, -100, 100]
    points = points[(points[:, 0] > boundary[0]) & (points[:, 0] < boundary[1])]
    points = points[(points[:, 1] > boundary[2]) & (points[:, 1] < boundary[3])]

    # Cut off points outside a certain height window
    min_height_window = [0.1, 1.0]
    points = points[(points[:, 2] > min_height_window[0]) & (points[:, 2] < min_height_window[1])]
    
    # Transform to bird's-eye view
    bev_points = points[:, :2]  # Remove Z-axis information

    # Define the grid parameters
    min_x = np.min(bev_points[:, 0])
    min_y = np.min(bev_points[:, 1])
    max_x = np.max(bev_points[:, 0])
    max_y = np.max(bev_points[:, 1])
    range_x = max_x - min_x
    range_y = max_y - min_y

    # Map the points to the grid
    grid_size = int(max(range_x, range_y) / resolution)
    bev_grid = np.zeros((grid_size + 1, grid_size + 1))  # Increase size by 1

    for point in bev_points:
        x = int((point[0] - min_x) / range_x * grid_size)
        y = int((point[1] - min_y) / range_y * grid_size)
        bev_grid[y, x] += 1

    # Generate the bird's-eye view image
    image = Image.fromarray(bev_grid.astype(np.uint8) * 255)

    # Save the processed image as PNG
    image.save(output_file)

# Example usage
pointcloud_to_birdseye("Town05.pcd", 0.05, "output.png")
