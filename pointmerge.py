import open3d as o3d
import numpy as np
pcd1 = o3d.io.read_point_cloud("p1.pcd")
pcd2 = o3d.io.read_point_cloud("p2.pcd")
def mergefunc(pcd1,pcd2):
    if pcd2.has_points():
        pcd = o3d.geometry.PointCloud()
        p1_load = np.asarray(pcd1.points)
        p2_load = np.asarray(pcd2.points)
        p3_load = np.concatenate((p1_load,p2_load), axis=0)
        pcd.points = o3d.utility.Vector3dVector(p3_load)
        if pcd2.has_colors()
            p1_color = np.asarray(pcd1.colors)
            p2_color = np.asarray(pcd2.colors)
            p3_color = np.concatenate((p1_color,p2_color), axis=0)
            pcd.colors = o3d.utility.Vector3dVector(p3_color)

        o3d.io.write_point_cloud("saved.ply", pcd)
        o3d.visualization.draw_geometries([pcd])
    else:
        print("pcd2 is empty")
