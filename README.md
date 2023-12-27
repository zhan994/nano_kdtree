# Nano Kdtree
**Light-weight Kdtree Algorithms based on nanoflann.**

## Content
- Nano DBSCAN

## Dependency

- PCL: save and display the result.

## Run

To display the result, use **PCL** to save pointcloud as ".pcd" file.

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

The result of example **test_nano_dbscan_3d**.

![](imgs/test_nano_dbscan_3d.png)

![](imgs/test_nano_dbscan_3d.gif)
