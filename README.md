# Nano DBSCAN
**Light-weight Density-Based Spatial Clustering of Applications with Noise based on nanoflann.**

## Dependency

- PCL: save and display the result of **Nano DBSCAN**.

## Run

To display the result of **Nano DBSCAN**, use **PCL** to save pointcloud as ".pcd" file.

```bash
git clone https://github.com/zhan994/nano_dbscan.git
cd nano_dbscan
mkdir build && cd build
cmake ..
make

# 2d
./test2d eps[float] min_pts[int] 

# 3d
./test3d eps[float] min_pts[int] 
```

The result of example **test3d**.

![](imgs/test3d.png)

![](imgs/test3d.gif)
