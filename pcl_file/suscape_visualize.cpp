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
#include <jsoncpp/json/json.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr loadPointCloud(const string& binFile) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(binFile, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", binFile.c_str());
        exit(EXIT_FAILURE);
    }
    return cloud;
}

// 从json文件中读取标注框信息，并对子项逐个打印，格式不是KITTI
vector<Json::Value> readLabel(const string& labelFile) {
    
    // 读取json文件
    Json::Reader reader;
    Json::Value root;
    ifstream file(labelFile, ifstream::binary);
    if (!reader.parse(file, root, false)) {
        cerr << "Failed to parse " << labelFile << endl;
        exit(EXIT_FAILURE);
    }
    // json结构如下：
    //{frame,objs:[{obj_id,obj_type,psr:{position:{x,y,z},rotation:{x,y,z},scale:{x,y,z}}},{obj_id,...},...]}
    vector<Json::Value> objs;
    for (int i = 0; i < root["objs"].size(); i++) {
        cout << "obj_id: " << root["objs"][i]["obj_id"].asString() << endl;
        cout << "obj_type: " << root["objs"][i]["obj_type"].asString() << endl;
        cout << "position: " << root["objs"][i]["psr"]["position"]["x"].asFloat() << " " << root["objs"][i]["psr"]["position"]["y"].asFloat() << " " << root["objs"][i]["psr"]["position"]["z"].asFloat() << endl;
        cout << "rotation: " << root["objs"][i]["psr"]["rotation"]["x"].asFloat() << " " << root["objs"][i]["psr"]["rotation"]["y"].asFloat() << " " << root["objs"][i]["psr"]["rotation"]["z"].asFloat() << endl;
        cout << "scale: " << root["objs"][i]["psr"]["scale"]["x"].asFloat() << " " << root["objs"][i]["psr"]["scale"]["y"].asFloat() << " " << root["objs"][i]["psr"]["scale"]["z"].asFloat() << endl;
        objs.push_back(root["objs"][i]);
    }

    return objs;
}

// 读取calib文件，获取标定参数
vector<Json::Value> readCalib(const string& calibFile) {
    vector<Json::Value> calib;
    // 读取json文件
    Json::Reader reader;
    Json::Value root;
    ifstream file(calibFile, ifstream::binary);
    if (!reader.parse(file, root, false)) {
        cerr << "Failed to parse " << calibFile << endl;
        exit(EXIT_FAILURE);
    }
    // 逐个打印子项，并储存到vector中
    cout << "rotation: " << root["rotation"][0].asFloat() << " " << root["rotation"][1].asFloat() << " " << root["rotation"][2].asFloat() << endl;
    cout << "translation: " << root["translation"][0].asFloat() << " " << root["translation"][1].asFloat() << " " << root["translation"][2].asFloat() << endl;
    cout << "color: " << root["color"][0].asFloat() << " " << root["color"][1].asFloat() << " " << root["color"][2].asFloat() << endl;
    cout << "disable: " << root["disable"].asBool() << endl;
    calib.push_back(root);

    return calib;
}

// 根据visualizeBoundingBox方法，生成一个方法以得到标注框的8个顶点坐标，返回值为vector<pcl::PointXYZ>
vector<vector<pcl::PointXYZ>> getBoundingBoxs(vector<Json::Value> objs) {
    vector<vector<pcl::PointXYZ>> boundingBoxs;
    for (int i = 0; i < objs.size(); i++) {
        float x = objs[i]["psr"]["position"]["x"].asFloat();
        float y = objs[i]["psr"]["position"]["y"].asFloat();
        float z = objs[i]["psr"]["position"]["z"].asFloat();
        float rotation_x = objs[i]["psr"]["rotation"]["x"].asFloat();
        float rotation_y = objs[i]["psr"]["rotation"]["y"].asFloat();
        float rotation_z = objs[i]["psr"]["rotation"]["z"].asFloat();
        float scale_x = objs[i]["psr"]["scale"]["x"].asFloat();
        float scale_y = objs[i]["psr"]["scale"]["y"].asFloat();
        float scale_z = objs[i]["psr"]["scale"]["z"].asFloat();
        float rotation = sqrt(rotation_x * rotation_x + rotation_y * rotation_y + rotation_z * rotation_z);
        // 获取8个顶点坐标
        float x1 = x - scale_x / 2;
        float y1 = y - scale_y / 2;
        float z1 = z - scale_z / 2;
        float x2 = x + scale_x / 2;
        float y2 = y - scale_y / 2;
        float z2 = z - scale_z / 2;
        float x3 = x + scale_x / 2;
        float y3 = y + scale_y / 2;
        float z3 = z - scale_z / 2;
        float x4 = x - scale_x / 2;
        float y4 = y + scale_y / 2;
        float z4 = z - scale_z / 2;
        float x5 = x - scale_x / 2;
        float y5 = y - scale_y / 2;
        float z5 = z + scale_z / 2;
        float x6 = x + scale_x / 2;
        float y6 = y - scale_y / 2;
        float z6 = z + scale_z / 2;
        float x7 = x + scale_x / 2;
        float y7 = y + scale_y / 2;
        float z7 = z + scale_z / 2;
        float x8 = x - scale_x / 2;
        float y8 = y + scale_y / 2;
        float z8 = z + scale_z / 2;

        // 乘旋转矩阵进行变换，旋转中心为标注框中心
        float x_center = (x1 + x2 + x3 + x4 + x5 + x6 + x7 + x8) / 8;
        float y_center = (y1 + y2 + y3 + y4 + y5 + y6 + y7 + y8) / 8;
        float temp_x1 = (x1 - x_center) * cos(rotation) - (y1 - y_center) * sin(rotation) + x_center;
        float temp_y1 = (x1 - x_center) * sin(rotation) + (y1 - y_center) * cos(rotation) + y_center;
        float temp_x2 = (x2 - x_center) * cos(rotation) - (y2 - y_center) * sin(rotation) + x_center;
        float temp_y2 = (x2 - x_center) * sin(rotation) + (y2 - y_center) * cos(rotation) + y_center;
        float temp_x3 = (x3 - x_center) * cos(rotation) - (y3 - y_center) * sin(rotation) + x_center;
        float temp_y3 = (x3 - x_center) * sin(rotation) + (y3 - y_center) * cos(rotation) + y_center;
        float temp_x4 = (x4 - x_center) * cos(rotation) - (y4 - y_center) * sin(rotation) + x_center;
        float temp_y4 = (x4 - x_center) * sin(rotation) + (y4 - y_center) * cos(rotation) + y_center;
        float temp_x5 = (x5 - x_center) * cos(rotation) - (y5 - y_center) * sin(rotation) + x_center;
        float temp_y5 = (x5 - x_center) * sin(rotation) + (y5 - y_center) * cos(rotation) + y_center;
        float temp_x6 = (x6 - x_center) * cos(rotation) - (y6 - y_center) * sin(rotation) + x_center;
        float temp_y6 = (x6 - x_center) * sin(rotation) + (y6 - y_center) * cos(rotation) + y_center;
        float temp_x7 = (x7 - x_center) * cos(rotation) - (y7 - y_center) * sin(rotation) + x_center;
        float temp_y7 = (x7 - x_center) * sin(rotation) + (y7 - y_center) * cos(rotation) + y_center;
        float temp_x8 = (x8 - x_center) * cos(rotation) - (y8 - y_center) * sin(rotation) + x_center;
        float temp_y8 = (x8 - x_center) * sin(rotation) + (y8 - y_center) * cos(rotation) + y_center;

        vector<pcl::PointXYZ> boundingBox;
        boundingBox.push_back(pcl::PointXYZ(temp_x1, temp_y1, z1));
        boundingBox.push_back(pcl::PointXYZ(temp_x2, temp_y2, z2));
        boundingBox.push_back(pcl::PointXYZ(temp_x3, temp_y3, z3));
        boundingBox.push_back(pcl::PointXYZ(temp_x4, temp_y4, z4));
        boundingBox.push_back(pcl::PointXYZ(temp_x5, temp_y5, z5));
        boundingBox.push_back(pcl::PointXYZ(temp_x6, temp_y6, z6));
        boundingBox.push_back(pcl::PointXYZ(temp_x7, temp_y7, z7));
        boundingBox.push_back(pcl::PointXYZ(temp_x8, temp_y8, z8));
        boundingBoxs.push_back(boundingBox);
    }

    return boundingBoxs;
}

// 可视化标注框
void visualizeBoundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<Json::Value> objs, vector<Json::Value> calib) {
    // 创建PCL可视化对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(cloud, "z");
    // 设置背景颜色，主色调为蓝色

    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZI>(cloud, fildColor, "sample cloud");

    // 逐个标注框进行可视化
    for (int i = 0; i < objs.size(); i++) {
        float x = objs[i]["psr"]["position"]["x"].asFloat();
        float y = objs[i]["psr"]["position"]["y"].asFloat();
        float z = objs[i]["psr"]["position"]["z"].asFloat();
        float rotation_x = objs[i]["psr"]["rotation"]["x"].asFloat();
        float rotation_y = objs[i]["psr"]["rotation"]["y"].asFloat();
        float rotation_z = objs[i]["psr"]["rotation"]["z"].asFloat();
        float scale_x = objs[i]["psr"]["scale"]["x"].asFloat();
        float scale_y = objs[i]["psr"]["scale"]["y"].asFloat();
        float scale_z = objs[i]["psr"]["scale"]["z"].asFloat();
        float rotation = sqrt(rotation_x * rotation_x + rotation_y * rotation_y + rotation_z * rotation_z);
        float color_r = calib[0]["color"][0].asFloat();
        float color_g = calib[0]["color"][1].asFloat();
        float color_b = calib[0]["color"][2].asFloat();
        float disable = calib[0]["disable"].asBool();
        
        // 获取8个顶点坐标
        float x1 = x - scale_x / 2;
        float y1 = y - scale_y / 2;
        float z1 = z - scale_z / 2;
        float x2 = x + scale_x / 2;
        float y2 = y - scale_y / 2;
        float z2 = z - scale_z / 2;
        float x3 = x + scale_x / 2;
        float y3 = y + scale_y / 2;
        float z3 = z - scale_z / 2;
        float x4 = x - scale_x / 2;
        float y4 = y + scale_y / 2;
        float z4 = z - scale_z / 2;
        float x5 = x - scale_x / 2;
        float y5 = y - scale_y / 2;
        float z5 = z + scale_z / 2;
        float x6 = x + scale_x / 2;
        float y6 = y - scale_y / 2;
        float z6 = z + scale_z / 2;
        float x7 = x + scale_x / 2;
        float y7 = y + scale_y / 2;
        float z7 = z + scale_z / 2;
        float x8 = x - scale_x / 2;
        float y8 = y + scale_y / 2;
        float z8 = z + scale_z / 2;

        // 乘旋转矩阵进行变换，旋转中心为标注框中心
        float x_center = (x1 + x2 + x3 + x4 + x5 + x6 + x7 + x8) / 8;
        float y_center = (y1 + y2 + y3 + y4 + y5 + y6 + y7 + y8) / 8;
        float temp_x1 = (x1 - x_center) * cos(rotation) - (y1 - y_center) * sin(rotation) + x_center;
        float temp_y1 = (x1 - x_center) * sin(rotation) + (y1 - y_center) * cos(rotation) + y_center;
        float temp_x2 = (x2 - x_center) * cos(rotation) - (y2 - y_center) * sin(rotation) + x_center;

        float temp_y2 = (x2 - x_center) * sin(rotation) + (y2 - y_center) * cos(rotation) + y_center;
        float temp_x3 = (x3 - x_center) * cos(rotation) - (y3 - y_center) * sin(rotation) + x_center;
        float temp_y3 = (x3 - x_center) * sin(rotation) + (y3 - y_center) * cos(rotation) + y_center;
        float temp_x4 = (x4 - x_center) * cos(rotation) - (y4 - y_center) * sin(rotation) + x_center;
        float temp_y4 = (x4 - x_center) * sin(rotation) + (y4 - y_center) * cos(rotation) + y_center;

        float temp_x5 = (x5 - x_center) * cos(rotation) - (y5 - y_center) * sin(rotation) + x_center;
        float temp_y5 = (x5 - x_center) * sin(rotation) + (y5 - y_center) * cos(rotation) + y_center;
        float temp_x6 = (x6 - x_center) * cos(rotation) - (y6 - y_center) * sin(rotation) + x_center;
        float temp_y6 = (x6 - x_center) * sin(rotation) + (y6 - y_center) * cos(rotation) + y_center;

        float temp_x7 = (x7 - x_center) * cos(rotation) - (y7 - y_center) * sin(rotation) + x_center;
        float temp_y7 = (x7 - x_center) * sin(rotation) + (y7 - y_center) * cos(rotation) + y_center;
        float temp_x8 = (x8 - x_center) * cos(rotation) - (y8 - y_center) * sin(rotation) + x_center;
        float temp_y8 = (x8 - x_center) * sin(rotation) + (y8 - y_center) * cos(rotation) + y_center;

        x1 = temp_x1;
        y1 = temp_y1;
        x2 = temp_x2;
        y2 = temp_y2;
        x3 = temp_x3;
        y3 = temp_y3;
        x4 = temp_x4;
        y4 = temp_y4;
        x5 = temp_x5;
        y5 = temp_y5;
        x6 = temp_x6;
        y6 = temp_y6;
        x7 = temp_x7;
        y7 = temp_y7;
        x8 = temp_x8;
        y8 = temp_y8;

        viewer->addLine(pcl::PointXYZ(x1, y1, z1), pcl::PointXYZ(x2, y2, z2), color_r, color_g, color_b, "line1"+to_string(i));
        viewer->addLine(pcl::PointXYZ(x2, y2, z2), pcl::PointXYZ(x3, y3, z3), color_r, color_g, color_b, "line2"+to_string(i));
        viewer->addLine(pcl::PointXYZ(x3, y3, z3), pcl::PointXYZ(x4, y4, z4), color_r, color_g, color_b, "line3"+to_string(i));
        viewer->addLine(pcl::PointXYZ(x4, y4, z4), pcl::PointXYZ(x1, y1, z1), color_r, color_g, color_b, "line4"+to_string(i));
        viewer->addLine(pcl::PointXYZ(x5, y5, z5), pcl::PointXYZ(x6, y6, z6), color_r, color_g, color_b, "line5"+to_string(i));
        viewer->addLine(pcl::PointXYZ(x6, y6, z6), pcl::PointXYZ(x7, y7, z7), color_r, color_g, color_b, "line6"+to_string(i));
        viewer->addLine(pcl::PointXYZ(x7, y7, z7), pcl::PointXYZ(x8, y8, z8), color_r, color_g, color_b, "line7"+to_string(i));
        viewer->addLine(pcl::PointXYZ(x8, y8, z8), pcl::PointXYZ(x5, y5, z5), color_r, color_g, color_b, "line8"+to_string(i));
        viewer->addLine(pcl::PointXYZ(x1, y1, z1), pcl::PointXYZ(x5, y5, z5), color_r, color_g, color_b, "line9"+to_string(i));
        viewer->addLine(pcl::PointXYZ(x2, y2, z2), pcl::PointXYZ(x6, y6, z6), color_r, color_g, color_b, "linea"+to_string(i));
        viewer->addLine(pcl::PointXYZ(x3, y3, z3), pcl::PointXYZ(x7, y7, z7), color_r, color_g, color_b, "lineb"+to_string(i));
        viewer->addLine(pcl::PointXYZ(x4, y4, z4), pcl::PointXYZ(x8, y8, z8), color_r, color_g, color_b, "linec"+to_string(i));
    }
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return;
}
void visualizePointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(cloud, "intensity");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZI>(cloud, fildColor, "sample cloud");

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        cerr << "Usage: " << argv[0] << " <bin file>" << endl;
        exit(EXIT_FAILURE);
    }
    string file_index = argv[1];
    string binFile = "/home/newDisk/SUSCape/dataset/lidar/scene-000000/lidar/" + file_index + ".pcd";
    string labelFile = "/home/newDisk/SUSCape/dataset/label/scene-000000/label/" + file_index + ".json";
    string calibFile = "/home/newDisk/SUSCape/dataset/calib/scene-000000/calib/aux_lidar/front.json";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = loadPointCloud(binFile);
    cout << "Loaded " << cloud->points.size() << " points" << endl;

    vector<Json::Value> objs = readLabel(labelFile);
    vector<Json::Value> calib = readCalib(calibFile);

    visualizeBoundingBox(cloud, objs, calib);
    // visualizePointCloud(cloud);
    return 0;
}