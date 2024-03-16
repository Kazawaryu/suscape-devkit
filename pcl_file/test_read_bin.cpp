#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/crop_box.h>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>


using namespace std;

// 用cropbox滤波器，只保留指定范围内的点云
pcl::PointCloud<pcl::PointXYZI>::Ptr getSubCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, float angle)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr sub_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::CropBox<pcl::PointXYZI> boxFilter;
    boxFilter.setInputCloud(cloud);
    boxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    boxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    boxFilter.setRotation(Eigen::Vector3f(0, 0, angle * M_PI / 180));
    boxFilter.filter(*sub_cloud);
    return sub_cloud;
}

// 输入KITTI格式标注框(h, w, l, x, y, z, ry)，输出cropbox滤波器的参数
void getBoxFilterParam(float h, float w, float l, float x, float y, float z, float ry, float &x_min, float &x_max, float &y_min, float &y_max, float &z_min, float &z_max, float &angle)
{
    // 由于KITTI标注框的坐标系与PCL的坐标系不同，需要进行坐标变换
    // KITTI->PCL: Z->X, X->-Y, Y->-Z
    x_min = z - l / 2;
    x_max = z + l / 2;
    y_min = -x - w / 2;
    y_max = -x + w / 2;
    // y_min = z - l / 2;
    // y_max = z + l / 2;
    // x_min = -(-x - w / 2);
    // x_max = -(-x + w / 2);


    z_min = -y;
    z_max = -y + h;
    angle = ry;

    // // 由于KITTI标注框的旋转角度与PCL的旋转角度不同，乘旋转矩阵进行变换
    // // 旋转矩阵为：[cos(angle), -sin(angle), 0; sin(angle), cos(angle), 0; 0, 0, 1]
    // float temp_x_min = x_min * cos(angle * M_PI / 180) - y_min * sin(angle * M_PI / 180);
    // float temp_y_min = x_min * sin(angle * M_PI / 180) + y_min * cos(angle * M_PI / 180);
    // float temp_x_max = x_max * cos(angle * M_PI / 180) - y_max * sin(angle * M_PI / 180);
    // float temp_y_max = x_max * sin(angle * M_PI / 180) + y_max * cos(angle * M_PI / 180);
    // x_min = temp_x_min;
    // y_min = temp_y_min;
    // x_max = temp_x_max;
    // y_max = temp_y_max;
    
    return;
}

// 读取标注框文件，返回n个标注框的参数，标注格式为KITTI，第一个参数为label的种类
// Car:0, Truck:1, Pedestrian:2, Rider:3, Van:4
// 读取每一行，每一行为一个标注框的参数，参数之间用空格分隔，参数格式为lab 0 0 0 0 0 0 0 h w l x y z ry，lab在读取时转换为数字
vector<vector<float>> readLabelFile()
{
    string labelFile = "/home/newDisk/tool/carla_dataset_tool/ada_exp/nuscenes_toolkit/self_tools/temp.txt";
    vector<vector<float>> label;
    // 读取每一行，每一行为一个标注框的参数，参数之间用空格分隔，参数格式为lab 0 0 0 0 0 0 0 h w l x y z ry，lab在读取时转换为数字
    ifstream infile(labelFile);
    string line;
    while (getline(infile, line)) {
        istringstream iss(line);
        vector<float> temp;
        string lab;
        iss >> lab;
        if (lab == "Car") {
            temp.push_back(0);
        } else if (lab == "Truck") {
            temp.push_back(1);
        } else if (lab == "Pedestrian") {
            temp.push_back(2);
        } else if (lab == "Rider") {
            temp.push_back(3);
        } else if (lab == "Van") {
            temp.push_back(4);
        }
        float temp_num;
        while (iss >> temp_num) {
            temp.push_back(temp_num);
        }
        label.push_back(temp);
    }
    return label;


}

int main(int argc, char** argv)
{
	//----------------------待读取的bin文件--------------------------
	// string binFile = "/home/newDisk/nuscene/samples/LIDAR_TOP/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151603547590.pcd.bin";

    // 从命令行输入读取bin文件
    if (argc != 2) {
        cerr << "Usage: " << argv[0] << " <bin file>" << endl;
        exit(EXIT_FAILURE);
    }
    string binFile = argv[1];

	fstream input(binFile.c_str(), ios::in | ios::binary);
	if (!input.good()) {
		cerr << "Could not read file: " << binFile << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
	cout << "Read KITTI point cloud with the name of " << binFile << endl;

	//----从bin文件中读取x、y、z、intensity信息并保存至PointXYZI----
	//XYZI
	pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZI>);

	for (int i = 0; input.good() && !input.eof(); i++) {
		pcl::PointXYZI point;
		input.read((char*)&point.x, 3 * sizeof(float));
		input.read((char*)&point.intensity, sizeof(float));
		pcd_cloud->push_back(point);
	}
	input.close();

    //--------------可视化KITTI数据集的bin点云------------------------
    // 读取标注框文件，并保存bbox的参数

    vector<vector<float>> label = readLabelFile();
    vector<vector<float>> bbox_param;
    for (int i = 0; i < label.size(); i++) {
        float x_min, x_max, y_min, y_max, z_min, z_max, angle;
        getBoxFilterParam(label[i][8], label[i][9], label[i][10], label[i][11], label[i][12], label[i][13], label[i][14], x_min, x_max, y_min, y_max, z_min, z_max, angle);
        bbox_param.push_back({x_min, x_max, y_min, y_max, z_min, z_max, angle});
    } 


    
    
	//获取不带文件路径和后缀的文件名，使pcd文件的名字与bin原始名字相同
	string pcdFile = binFile.substr(0, binFile.rfind(".")) + ".pcd";
	cout << "Read KITTI point cloud with " << (*pcd_cloud).size() << " points, writing to " << pcdFile << endl;

	pcl::PCDWriter writer;
	// writer.write<pcl::PointXYZI>(pcdFile, *pcd_cloud, true);
    writer.write<pcl::PointXYZI>(pcdFile, *pcd_cloud, true);

	//--------------可视化KITTI数据集的bin点云------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>fildColor(pcd_cloud, "intensity");
	viewer->setBackgroundColor(0, 0, 0);
	viewer->setWindowName("pcl_viewer");
	viewer->addText("KITTI point clouds are shown by PCL", 50, 50, 0, 1, 0, "v1_text");
	viewer->addPointCloud<pcl::PointXYZI>(pcd_cloud, fildColor, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    // viewer->addCube(x_min, x_max, y_min, y_max, z_min, z_max, 1, 0, 0, "cube");
    // addLine对每一个标注框进行连线可视化
    for (int i = 0; i < bbox_param.size(); i++) {
        // 乘旋转矩阵进行变换
        float x_min = bbox_param[i][0];
        float x_max = bbox_param[i][1];
        float y_min = bbox_param[i][2];
        float y_max = bbox_param[i][3];
        float z_min = bbox_param[i][4];
        float z_max = bbox_param[i][5];
        float angle = bbox_param[i][6];
        // 标注框绕其中心点旋转，anmgle为旋转角度（弧度）
        float x_center = (x_min + x_max) / 2;
        float y_center = (y_min + y_max) / 2;
        float temp_x_min = (x_min - x_center) * cos(angle-1.57) - (y_min - y_center) * sin(angle-1.57) + x_center;
        float temp_y_min = (x_min - x_center) * sin(angle-1.57) + (y_min - y_center) * cos(angle-1.57) + y_center;
        float temp_x_max = (x_max - x_center) * cos(angle-1.57) - (y_max - y_center) * sin(angle-1.57) + x_center;
        float temp_y_max = (x_max - x_center) * sin(angle-1.57) + (y_max - y_center) * cos(angle-1.57) + y_center;
        x_min = temp_x_min;
        y_min = temp_y_min;
        x_max = temp_x_max;
        y_max = temp_y_max;

        viewer->addLine(pcl::PointXYZ(x_min, y_min, z_min), pcl::PointXYZ(x_max, y_min, z_min), 1, 0, 0, "line1"+(to_string(i)));
        viewer->addLine(pcl::PointXYZ(x_max, y_min, z_min), pcl::PointXYZ(x_max, y_max, z_min), 1, 0, 0, "line2"+(to_string(i)));
        viewer->addLine(pcl::PointXYZ(x_max, y_max, z_min), pcl::PointXYZ(x_min, y_max, z_min), 1, 0, 0, "line3"+(to_string(i)));
        viewer->addLine(pcl::PointXYZ(x_min, y_max, z_min), pcl::PointXYZ(x_min, y_min, z_min), 1, 0, 0, "line4"+(to_string(i)));
        viewer->addLine(pcl::PointXYZ(x_min, y_min, z_max), pcl::PointXYZ(x_max, y_min, z_max), 1, 0, 0, "line5"+(to_string(i)));
        viewer->addLine(pcl::PointXYZ(x_max, y_min, z_max), pcl::PointXYZ(x_max, y_max, z_max), 1, 0, 0, "line6"+(to_string(i)));
        viewer->addLine(pcl::PointXYZ(x_max, y_max, z_max), pcl::PointXYZ(x_min, y_max, z_max), 1, 0, 0, "line7"+(to_string(i)));
        viewer->addLine(pcl::PointXYZ(x_min, y_max, z_max), pcl::PointXYZ(x_min, y_min, z_max), 1, 0, 0, "line8"+(to_string(i)));
        viewer->addLine(pcl::PointXYZ(x_min, y_min, z_min), pcl::PointXYZ(x_min, y_min, z_max), 1, 0, 0, "line9"+(to_string(i)));
        viewer->addLine(pcl::PointXYZ(x_max, y_min, z_min), pcl::PointXYZ(x_max, y_min, z_max), 1, 0, 0, "linea"+(to_string(i)));
        viewer->addLine(pcl::PointXYZ(x_max, y_max, z_min), pcl::PointXYZ(x_max, y_max, z_max), 1, 0, 0, "lineb"+(to_string(i)));
        viewer->addLine(pcl::PointXYZ(x_min, y_max, z_min), pcl::PointXYZ(x_min, y_max, z_max), 1, 0, 0, "linec"+(to_string(i)));

    }

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
