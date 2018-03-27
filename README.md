# FastPointCloudSegmentation

This is a modified implementation of the point cloud segmentation method described in [this IEEE paper](http://ieeexplore.ieee.org/document/7989591/).

## Requirements

* g++
* make
* CloudCompare (visualization purposes)

## Getting Started

1. Clone this repository on to your local machine.
2. In the project top level directory, run `make` or `make all`.
3. Upload __build/predicted_clusters.txt__ into CloudCompare.

## Libraries Used

* nanoflann (fast kd-tree C++ library)