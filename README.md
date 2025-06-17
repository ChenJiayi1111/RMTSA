# RMTSA
## 1.Introduction

Laser 3D scanning technology is currently used in various engineering measurements. Based on this technology, this article develops a software called `RMTSA`(Rapid Mountain Tunnel Section Analyzer) for obtaining mountain tunnel section data and conducting quantitative analysis of over/under excavation. The content includes automatic equidistant continuous cutting of tunnel point cloud data sections, comparison between tunnel section point cloud data and design drawings, and quantitative display of tunnel over/under excavation situations.

## 2.Main functions
`RMTSA` is a tool for obtaining data of tunnel sections  and analyze the over/under excavation situations of the mountain tunnel sections. It mainly implements the following functions:
- Preprocess measurement data, including downsampling and smoothing.
- Tunnel point cloud data trimming and section acquisition.
- Comparison of tunnel measured section point cloud data with design curves.
- Quantitative calculation of over/ under excavation of tunnel sections.
- Generate visual results of the comparison.

## 3. Get started
### 3.1 System requirements

-   `Python 3.8`
    
-   `Open3D 0.19.0`
    
-   `NumPy 1.26.4`
    
-  `Matplotlib 3.9.2`
    
-   `Scipy 1.13.1`

Among them, `Open3D` requires additional installation.
```
pip install open3d
```
### 3.2 Preprocessing of measured point cloud data
Before using `RMTSA`software, the measured point cloud data acquired through SLAM algorithms requires filtering to enhance its quality. Following the filtering process, point cloud data from internal construction equipment, ventilation ducts, and the ground within the tunnel are removed. This step ensures the acquisition of point cloud data specifically for the upper section of the mountain tunnel. The resulting point cloud data serves as the raw data to be processed within `RMTSA`, ultimately leading to the generation of results for the over/under excavation analysis of the tunnel sections.

### 3.3 Select files
Firstly, `RMTSA` will instruct users simply need to import the measured tunnel point cloud data in `pcd` format and the tunnel section design data in `txt` format.
```python  
print("Please select design curve file")  
DESIGN_CURVE_FILE = filedialog.askopenfilename(  
    title="Select Design Curve File",  
    filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]  
)  
print("Please select measurement data file")  
INPUT_FILE = filedialog.askopenfilename(  
    title="Select Point Cloud File",  
    filetypes=[("Point Cloud Data", "*.pcd"), ("All Files", "*.*")]  
)
```
### 3.4 Input basic parameters
Then the user can set the following two parameters to obtain the desired result .
| Parameter        | Description                                                       |
| ---------------- | ----------------------------------------------------------------- |
| SLICE\_INTERVAL  | The spacing between sections, with a default value of 10.0 meters |
| SLICE\_THICKNESS | The data thickness of each section, with a default value of 0.3 meters (a value between 0.3 - 0.5m is recommended) |

```python
SLICE_INTERVAL = input("Please enter section spacing (meters) (Default: 10): ").strip()  
if not SLICE_INTERVAL:  
    SLICE_INTERVAL = 10.0  
else:  
    SLICE_INTERVAL = float(SLICE_INTERVAL)  
  
SLICE_THICKNESS = input("Please enter section thickness (meters) (Default: 0.3): ").strip()  
if not SLICE_THICKNESS:  
    SLICE_THICKNESS = 0.3  
else:  
    SLICE_THICKNESS = float(SLICE_THICKNESS)
```
The `RMTSA` software has many adjustable parameters, such as voxel size. However, these parameters are complex to adjust. Inappropriate values may lead to poor results. It is recommended that users thoroughly understand their point cloud data and the software before making adjustments.
## 4.Example
​Examples are provided in the  `Examples`  folder, and details are explained here:  [Examples](https://github.com/xurongqiao/ZjuMatrix/blob/main/doc/EXAMPLES.md).
## 5.License
RMTSA software is licensed under the [MIT License](https://github.com/ChenJiayi1111/RMTSA/blob/main/LICENSE.txt)




