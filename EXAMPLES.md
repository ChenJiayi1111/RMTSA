# Examples

Two experimentally validated sample datasets, acquired from the Maoliling Tunnel in Taizhou, Zhejiang Province using a `Livox Avia` lidar with point cloud mapping supported by the `FAST-lio` SLAM algorithm, are provided in the  `Examples`  directory: `ExampleOne` folder contains tunnel point cloud data after first lining, while `ExampleTwo` folder comprises tunnel point cloud data after secondary lining, with corresponding tunnel design curve data included in respective folders. There are diverse methods for obtaining design data files. Here, the text file of tunnel section curve design data was acquired through image recognition. Prior to `RMTSA` processing,  actual measured tunnel point cloud data typically undergoes filtering for quality enhancement followed by removal of non-structural components (e.g., construction equipment, ventilation ducts, and tunnel ground surfaces); both examples have undergone this rigorous preprocessing step through `CloudCompare` software, ensuring direct compatibility with the `RMTSA` software.
##
### Example one
![exampleone](https://github.com/ChenJiayi1111/RMTSA/blob/main/imgs/example1.png)
### Example two
![exampletwo](https://github.com/ChenJiayi1111/RMTSA/blob/main/imgs/example2.png)
##
Ultimately, the output results can be classified into three categories: over/under excavation diagrams of  tunnel sections, quantitative calculation results of over/under excavation and the point cloud data of each section. They will all be stored in the same folder.
