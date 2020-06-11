import open3d as o3d
import numpy as np
import os
import pye57
pcd1 = o3d.io.read_point_cloud("p1.pcd")
pcd2 = o3d.io.read_point_cloud("p2.pcd")
def mergefunc(pcd1,pcd2):
    if pcd2.has_points():
        pcd = o3d.geometry.PointCloud()
        p1_load = np.asarray(pcd1.points)
        p2_load = np.asarray(pcd2.points)
        p3_load = np.concatenate((p1_load,p2_load), axis=0)
        pcd.points = o3d.utility.Vector3dVector(p3_load)
        if pcd2.has_colors():
            p1_color = np.asarray(pcd1.colors)
            p2_color = np.asarray(pcd2.colors)
            p3_color = np.concatenate((p1_color,p2_color), axis=0)
            pcd.colors = o3d.utility.Vector3dVector(p3_color)

        o3d.io.write_point_cloud("saved.ply", pcd)
        o3d.visualization.draw_geometries([pcd])
    else:
        print("pcd2 is empty")

def ShowScanLoc(file):
        
    pcd = o3d.geometry.PointCloud()
    count = 0
    for e57File in os.listdir(file):
        if e57File.endswith(".e57"):
            with pye57.E57(file + "/" + e57File) as e57:
                
                for idx in range(e57.scan_count):
                    data = e57.read_scan(idx, ignore_missing_fields = True)
                    numPoints = data["cartesianX"].shape[0] # instead of header.point_count as the latter is often too large
                    xyz = np.zeros((numPoints, 3))
                    xyz[:, 0] = np.reshape(data["cartesianX"], -1)
                    xyz[:, 1] = np.reshape(data["cartesianY"], -1)
                    xyz[:, 2] = np.reshape(data["cartesianZ"], -1)
                    tmp = o3d.geometry.PointCloud()
                    tmp.points = o3d.utility.Vector3dVector(xyz)
                    if count == 0:
                        tmp.paint_uniform_color([1, 0.706, 0])
                    else:
                        tmp.paint_uniform_color([0, 0.5, 0.5])
                    o3d.visualization.draw_geometries([tmp])
                    pcd += tmp.voxel_down_sample(0.02)
                    o3d.visualization.draw_geometries([pcd])
                count += 1
    o3d.visualization.draw_geometries([pcd])


if __name__ == "__main__":
    ShowScanLoc("input/")