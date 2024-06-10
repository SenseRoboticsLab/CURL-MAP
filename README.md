# CURL-MAP
[Paper](https://github.com/SenseRoboticsLab/CURL-MAP/blob/master/paper/LiDAR_ICRA2024.pdf)

The code will be released soon.

## System Architecture
We extend the continuous reconstruction and memory efficiency advantages of [CURL](https://github.com/SenseRoboticsLab/CURL) to large-scale maps, introducing new capabilities for odometry and updatable mapping.
<p align="center">
<img src="./images/ICRA24_cover_fig_page-0001.jpg" alt="framework" width=80%/>
</p> 
With the same CURL-MAP, can produce different resolutions of reconstruction.

## Examples
### 1. Continuous Reconstruction
<p align="center">
<img src="images/1.gif" alt="continuous reconstruction" width=80%/>
</p>

> Newer College Dataset

<p align="center">
<img src="images/2.gif" alt="Newer College" width=80%/>
</p>

> KITTI Dataset

<p align="center">
<img src="images/4.gif" alt="Newer College" width=80%/>
</p>

### 2. Odometry and Mapping

<p align="center">
<img src="images/5.gif" alt="Newer College" width=80%/>
</p>
<p style="text-align: center;">KITTI-05</p>

## Time Analysis

CURL-MAP is able to achieve real-time performance on a CPU in the FAST mode.
<p align="center">
<img src="./images/time_evaluation-cropped_page-0001.jpg" alt="framework" width=80%/>
</p> 