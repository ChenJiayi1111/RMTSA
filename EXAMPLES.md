# Examples

Two experimentally validated sample datasets, acquired from the Maoliling Tunnel in Taizhou, Zhejiang Province using a `Livox Avia` lidar with point cloud mapping supported by the `FAST-LIO` SLAM algorithm, are provided in the  `Examples`  directory: `ExampleOne` folder contains tunnel point cloud data after first lining, while `ExampleTwo` folder comprises tunnel point cloud data after secondary lining, with corresponding tunnel design curve data included in respective folders. Prior to `RMTSA` processing,  tunnel point clouds typically undergo filtering for quality enhancement followed by removal of non-structural components (e.g., construction equipment, ventilation ducts, and tunnel ground surfaces); both examples have undergone this rigorous preprocessing pipeline through `CloudCompare`software, ensuring direct compatibility with the `RMTSA` software.

## Exampleone
![Exampleone](https://github.com/ChenJiayi1111/RMTSA/blob/main/imgs/example1.png)
## Exampletwo
![Exampletwo](https://github.com/ChenJiayi1111/RMTSA/blob/main/imgs/example2.png)

Due to the presence of ventilation ducts, electrical cables, and other components inside the tunnel, certain areas of point cloud data are missing. The following figure illustrates the locations of the missing data and the corresponding causes.
## Locations and causes of point cloud data gaps in the tunnel
![Locations and causes of point cloud data gaps in the tunnel](https://github.com/ChenJiayi1111/RMTSA/blob/main/imgs/missingdataexplanation.png)

# Results
Ultimately, the output results can be classified into three categories: over/under excavation diagram of  tunnel sections, quantitative calculation results of over/under excavation and the point cloud data of each section. They will all be stored in the same folder.
## One of the results from Exampleone and the explanation
![One of the results from Exampleone and the explanation](https://github.com/ChenJiayi1111/RMTSA/blob/main/imgs/result1.png)
## One of the results from Exampletwo and the explanation
![One of the results from Exampletwo and the explanation](https://github.com/ChenJiayi1111/RMTSA/blob/main/imgs/result2.png)