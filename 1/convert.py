import open3d as o3d
pcd = o3d.io.read_point_cloud("000000002078_000900000000.ply")
o3d.io.write_point_cloud("sink_pointcloud.pcd", pcd)
