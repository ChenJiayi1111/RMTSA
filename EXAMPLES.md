# Examples

Two experimentally validated sample datasets, acquired from the Maoliling Tunnel in Taizhou, Zhejiang Province using a `Livox Avia` lidar with point cloud mapping supported by the `FAST-LIO` SLAM algorithm, are provided in the  `Examples`  directory: `ExampleOne` folder contains tunnel point cloud data after first lining, while `ExampleTwo` folder comprises tunnel point cloud data after secondary lining, with corresponding tunnel design curve data included in respective folders. Prior to `RMTSA` processing,  tunnel point clouds typically undergo filtering for quality enhancement followed by removal of non-structural components (e.g., construction equipment, ventilation ducts, and tunnel ground surfaces); both examples have undergone this rigorous preprocessing pipeline through `CloudCompare`software, ensuring direct compatibility with the `RMTSA` software.
###Example one
![exampleone](https://github.com/ChenJiayi1111/RMTSA/blob/main/imgs/example1.png)
###Example two
![exampletwo](https://github.com/ChenJiayi1111/RMTSA/blob/main/imgs/example2.png)

Ultimately, the output results can be classified into three categories: over/under excavation diagram of  tunnel sections, quantitative calculation results of over/under excavation and the point cloud data of each section. They will all be stored in the same folder.
