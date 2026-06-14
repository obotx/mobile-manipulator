import numpy as np
import pyoctomap
import cv2
from scipy.spatial.transform import Rotation as R
import traceback  # Add for better error debugging

class OctomapConverter:
    """Convert depth images to OctoMap for 3D environment mapping."""
    
    def __init__(self, resolution=0.05, max_range=5.0):
        """
        Initialize OctoMap converter.
        
        Args:
            resolution: Voxel size in meters 
            max_range: Maximum sensor range in meters
        """
        self.resolution = float(resolution)
        self.max_range = float(max_range)
        
        try:
            # Create OcTree with specified resolution
            self.octree = pyoctomap.OcTree(self.resolution)
            
            # Set occupancy probabilities (optional)
            try:
                self.octree.setProbHit(0.7)
                self.octree.setProbMiss(0.4)
                self.octree.setClampingThresMin(0.1192)
                self.octree.setClampingThresMax(0.971)
            except:
                pass  # Some pyoctomap versions don't have these methods
            
            print(f"✓ OctoMap OcTree created with resolution {self.resolution}m")
            
        except Exception as e:
            print(f"✗ Failed to create OcTree: {e}")
            self.octree = None
    
    def depth_to_pointcloud(self, depth_image, camera_intrinsics, camera_pose):
        """
        Convert depth image to 3D point cloud in world coordinates.
        
        Returns:
            Nx3 numpy array of 3D points in world coordinates
        """
        try:
            height, width = depth_image.shape
            
            # Validate intrinsics matrix
            if camera_intrinsics.shape != (3, 3):
                print(f"✗ Invalid intrinsics shape: {camera_intrinsics.shape}")
                return np.empty((0, 3), dtype=np.float64)
            
            # Ensure arrays are float64
            depth_image = depth_image.astype(np.float64)
            camera_intrinsics = camera_intrinsics.astype(np.float64)
            camera_pose = camera_pose.astype(np.float64)
            
            fx = float(camera_intrinsics[0, 0])
            fy = float(camera_intrinsics[1, 1])
            cx = float(camera_intrinsics[0, 2])
            cy = float(camera_intrinsics[1, 2])
            
            # Create grid of pixel coordinates (vectorized for speed)
            u, v = np.meshgrid(np.arange(width, dtype=np.float64), 
                               np.arange(height, dtype=np.float64))
            
            # Convert to 3D camera coordinates (vectorized)
            z = depth_image
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            
            # Reshape for easier processing
            x_flat = x.flatten()
            y_flat = y.flatten()
            z_flat = z.flatten()
            
            # Filter out invalid depth values
            valid_mask = (z_flat > 0.1) & (z_flat < self.max_range) & np.isfinite(z_flat)
            
            if not np.any(valid_mask):
                print("✗ No valid depth points found")
                return np.empty((0, 3), dtype=np.float64)
            
            # Extract valid points
            x_valid = x_flat[valid_mask]
            y_valid = y_flat[valid_mask]
            z_valid = z_flat[valid_mask]
            
            # Stack coordinates
            points_camera = np.column_stack([x_valid, y_valid, z_valid])
            
            # Validate camera pose
            if camera_pose.shape != (4, 4):
                print(f"✗ Invalid camera pose shape: {camera_pose.shape}")
                return np.empty((0, 3), dtype=np.float64)
            
            # Transform to world coordinates (vectorized)
            # Add homogeneous coordinate
            ones = np.ones((points_camera.shape[0], 1), dtype=np.float64)
            points_homogeneous = np.hstack([points_camera, ones])
            
            # Transform
            points_world_homogeneous = (camera_pose @ points_homogeneous.T).T
            points_world = points_world_homogeneous[:, :3] / points_world_homogeneous[:, 3:]
            
            print(f"✓ Point cloud generated: {points_world.shape[0]} points")
            return points_world
            
        except Exception as e:
            print(f"✗ Error in depth_to_pointcloud: {e}")
            traceback.print_exc()
            return np.empty((0, 3), dtype=np.float64)
    
    def integrate_pointcloud(self, pointcloud, sensor_origin):
        """
        Integrate point cloud into OctoMap using ray casting.
        
        Args:
            pointcloud: Nx3 numpy array of points in world coordinates
            sensor_origin: 3-element array for sensor position (x, y, z)
        
        Returns:
            bool: True if integration was successful
        """
        if self.octree is None:
            print("✗ Octree not initialized")
            return False
        
        if pointcloud.shape[0] == 0:
            print("✗ Empty point cloud")
            return False
        
        try:
            # Convert inputs to correct types
            pointcloud = pointcloud.astype(np.float64)
            sensor_origin = np.array(sensor_origin, dtype=np.float64).flatten()
            
            if sensor_origin.shape[0] != 3:
                print(f"✗ Invalid sensor origin shape: {sensor_origin.shape}")
                return False
            
            # Check pyoctomap version and use appropriate API
            # Try different insert methods based on pyoctomap version
            
            # Method 1: Try with numpy array directly (newer versions)
            try:
                # Some versions accept numpy arrays directly
                self.octree.insertPointCloud(pointcloud, sensor_origin, 
                                           maxrange=float(self.max_range))
                print(f"✓ Point cloud integrated (numpy array method): {pointcloud.shape[0]} points")
                return True
            except Exception as e1:
                print(f"  Note: numpy array method failed: {e1}")
                
                # Method 2: Try converting to list of tuples (older versions)
                try:
                    # Convert to list of points
                    points_list = []
                    for i in range(pointcloud.shape[0]):
                        point = pointcloud[i]
                        points_list.append((float(point[0]), float(point[1]), float(point[2])))
                    
                    # Convert to numpy array of points
                    points_array = np.array(points_list, dtype=np.float64)
                    
                    # Insert point cloud
                    self.octree.insertPointCloud(points_array, sensor_origin, 
                                               maxrange=float(self.max_range))
                    print(f"✓ Point cloud integrated (list method): {pointcloud.shape[0]} points")
                    return True
                    
                except Exception as e2:
                    print(f"  Note: list method failed: {e2}")
                    
                    # Method 3: Try manual insertion point by point
                    try:
                        print("  Trying point-by-point insertion...")
                        for i in range(0, pointcloud.shape[0], 100):  # Every 100th point
                            if i < pointcloud.shape[0]:
                                point = pointcloud[i]
                                self.octree.insertRay(
                                    sensor_origin,
                                    point,
                                    maxrange=float(self.max_range)
                                )
                        
                        print(f"✓ Point cloud integrated (ray casting): {pointcloud.shape[0]} points")
                        return True
                        
                    except Exception as e3:
                        print(f"✗ All integration methods failed: {e3}")
                        return False
            
        except Exception as e:
            print(f"✗ Error in integrate_pointcloud: {e}")
            traceback.print_exc()
            return False
    
    def integrate_depth_image(self, depth_image, camera_intrinsics, camera_pose):
        """
        Complete pipeline: depth image -> point cloud -> octomap integration.
        
        Returns:
            int: Number of points integrated
        """
        if self.octree is None:
            print("✗ Octree not initialized")
            return 0
        
        try:
            # Extract sensor origin from camera pose
            sensor_origin = camera_pose[:3, 3]
            
            # Convert depth to point cloud
            pointcloud = self.depth_to_pointcloud(depth_image, camera_intrinsics, camera_pose)
            
            if pointcloud.shape[0] == 0:
                return 0
            
            # Integrate into octree
            success = self.integrate_pointcloud(pointcloud, sensor_origin)
            
            if success:
                return pointcloud.shape[0]
            return 0
            
        except Exception as e:
            print(f"✗ Error in integrate_depth_image: {e}")
            return 0
    
    def save_map(self, filename):
        """Save OctoMap to binary .bt file."""
        if self.octree is None:
            print("✗ Octree not initialized, cannot save")
            return False
        
        try:
            success = self.octree.writeBinary(str(filename))
            if success:
                print(f"✓ OctoMap saved to {filename}")
                return True
            else:
                print(f"✗ Failed to save OctoMap to {filename}")
                return False
                
        except Exception as e:
            print(f"✗ Error saving OctoMap: {e}")
            return False
    
    def get_occupied_voxels(self, min_prob=0.5):
        """Get all occupied voxels above probability threshold."""
        if self.octree is None:
            print("✗ Octree not initialized")
            return []
        
        occupied_voxels = []
        
        try:
            # Iterate through leaf nodes
            for it in self.octree.begin_leafs():
                if it.isLeaf():
                    # Get occupancy probability
                    occupancy = it.getOccupancy()
                    if occupancy > min_prob:
                        # Get center coordinates
                        coord = it.getCoordinate()
                        occupied_voxels.append((coord.x(), coord.y(), coord.z()))
            
            return occupied_voxels
            
        except Exception as e:
            print(f"✗ Error getting occupied voxels: {e}")
            return []
    
    def get_map_info(self):
        """Get information about the current OctoMap."""
        if self.octree is None:
            return "Octree not initialized"
        
        try:
            info = []
            info.append(f"Resolution: {self.octree.getResolution()} m")
            info.append(f"Tree depth: {self.octree.getTreeDepth()}")
            info.append(f"Tree size: {self.octree.size()} nodes")
            
            # Try to get memory usage
            try:
                mem_usage = self.octree.memoryUsage()
                info.append(f"Memory usage: {mem_usage / 1024:.1f} KB")
            except:
                pass
            
            return "\n".join(info)
            
        except Exception as e:
            return f"Error getting map info: {e}"