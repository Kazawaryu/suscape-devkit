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
#include <pcl/surface/concave_hull.h>

using namespace std;

extern "C" float* callsByPython(const char* file_index, const char* visWhat);
extern "C" int getArrayLength();

int getAlphaCount(pcl::PointCloud<pcl::PointXYZI>::Ptr obj_cloud);

pcl::PointCloud<pcl::PointXYZI>::Ptr loadPointCloud(const string& binFile) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(binFile, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", binFile.c_str());
        exit(EXIT_FAILURE);
    }
    return cloud;
}


vector<Json::Value> readLabel(const string& labelFile) {
    Json::Reader reader;
    Json::Value root;
    ifstream file(labelFile, ifstream::binary);
    if (!reader.parse(file, root, false)) {
        cerr << "Failed to parse " << labelFile << endl;
        exit(EXIT_FAILURE);
    }
    //{frame,objs:[{obj_id,obj_type,psr:{position:{x,y,z},rotation:{x,y,z},scale:{x,y,z}}},{obj_id,...},...]}
    vector<Json::Value> objs;
    for (int i = 0; i < root["objs"].size(); i++) {
        objs.push_back(root["objs"][i]);
    }

    return objs;
}


vector<Json::Value> readCalib(const string& calibFile) {
    vector<Json::Value> calib;
    Json::Reader reader;
    Json::Value root;
    ifstream file(calibFile, ifstream::binary);
    if (!reader.parse(file, root, false)) {
        cerr << "Failed to parse " << calibFile << endl;
        exit(EXIT_FAILURE);
    }

    calib.push_back(root);

    return calib;
}

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

vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> getInnerPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<Json::Value> objs, vector<vector<float>> &result) {
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> innerPoints;
    pcl::CropBox<pcl::PointXYZI> cropBoxFilter;
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


        pcl::PointCloud<pcl::PointXYZI>::Ptr innerCloud(new pcl::PointCloud<pcl::PointXYZI>);
        cropBoxFilter.setInputCloud(cloud);
        Eigen::Vector4f minPoint, maxPoint;
        minPoint[0] = min(temp_x1, min(temp_x2, min(temp_x3, temp_x4)));
        minPoint[1] = min(temp_y1, min(temp_y2, min(temp_y3, temp_y4)));
        minPoint[2] = min(z1, min(z2, min(z3, z4)));
        maxPoint[0] = max(temp_x5, max(temp_x6, max(temp_x7, temp_x8)));
        maxPoint[1] = max(temp_y5, max(temp_y6, max(temp_y7, temp_y8)));
        maxPoint[2] = max(z5, max(z6, max(z7, z8)));
        cropBoxFilter.setMin(minPoint);
        cropBoxFilter.setMax(maxPoint);
        cropBoxFilter.filter(*innerCloud);
        innerPoints.push_back(innerCloud);

        int mesh_count = getAlphaCount(innerCloud);
        float distance = sqrt(x * x + y * y + z * z);
        float bbox_volume = scale_x * scale_y * scale_z;
        
        vector<float> temp;
        float idx_ = std::stof(objs[i]["obj_id"].asString());
        temp.push_back(idx_);
        temp.push_back(distance);
        temp.push_back(mesh_count);
        temp.push_back(bbox_volume);
        result.push_back(temp);
    }
    return innerPoints;
}

int getAlphaCount(pcl::PointCloud<pcl::PointXYZI>::Ptr obj_cloud) {
    pcl::ConcaveHull<pcl::PointXYZI> concaveHull;
    concaveHull.setInputCloud(obj_cloud);
    concaveHull.setAlpha(1.5);
    pcl::PointCloud<pcl::PointXYZI> hull;
    concaveHull.reconstruct(hull);

    return hull.points.size();
}

void visualizeWithBbox(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<Json::Value> objs, vector<Json::Value> calib) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(cloud, "z");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZI>(cloud, fildColor, "sample cloud");
    vector<vector<pcl::PointXYZ>> boundingBoxs = getBoundingBoxs(objs);

    for (int i = 0; i < boundingBoxs.size(); i++) {
        viewer->addLine(pcl::PointXYZ(boundingBoxs[i][0].x, boundingBoxs[i][0].y, boundingBoxs[i][0].z), pcl::PointXYZ(boundingBoxs[i][1].x, boundingBoxs[i][1].y, boundingBoxs[i][1].z), 1.0, 0.0, 0.0, "line1"+to_string(i));
        viewer->addLine(pcl::PointXYZ(boundingBoxs[i][1].x, boundingBoxs[i][1].y, boundingBoxs[i][1].z), pcl::PointXYZ(boundingBoxs[i][2].x, boundingBoxs[i][2].y, boundingBoxs[i][2].z), 1.0, 0.0, 0.0, "line2"+to_string(i));
        viewer->addLine(pcl::PointXYZ(boundingBoxs[i][2].x, boundingBoxs[i][2].y, boundingBoxs[i][2].z), pcl::PointXYZ(boundingBoxs[i][3].x, boundingBoxs[i][3].y, boundingBoxs[i][3].z), 1.0, 0.0, 0.0, "line3"+to_string(i));
        viewer->addLine(pcl::PointXYZ(boundingBoxs[i][3].x, boundingBoxs[i][3].y, boundingBoxs[i][3].z), pcl::PointXYZ(boundingBoxs[i][0].x, boundingBoxs[i][0].y, boundingBoxs[i][0].z), 1.0, 0.0, 0.0, "line4"+to_string(i));
        viewer->addLine(pcl::PointXYZ(boundingBoxs[i][4].x, boundingBoxs[i][4].y, boundingBoxs[i][4].z), pcl::PointXYZ(boundingBoxs[i][5].x, boundingBoxs[i][5].y, boundingBoxs[i][5].z), 1.0, 0.0, 0.0, "line5"+to_string(i));
        viewer->addLine(pcl::PointXYZ(boundingBoxs[i][5].x, boundingBoxs[i][5].y, boundingBoxs[i][5].z), pcl::PointXYZ(boundingBoxs[i][6].x, boundingBoxs[i][6].y, boundingBoxs[i][6].z), 1.0, 0.0, 0.0, "line6"+to_string(i));
        viewer->addLine(pcl::PointXYZ(boundingBoxs[i][6].x, boundingBoxs[i][6].y, boundingBoxs[i][6].z), pcl::PointXYZ(boundingBoxs[i][7].x, boundingBoxs[i][7].y, boundingBoxs[i][7].z), 1.0, 0.0, 0.0, "line7"+to_string(i));
        viewer->addLine(pcl::PointXYZ(boundingBoxs[i][7].x, boundingBoxs[i][7].y, boundingBoxs[i][7].z), pcl::PointXYZ(boundingBoxs[i][4].x, boundingBoxs[i][4].y, boundingBoxs[i][4].z), 1.0, 0.0, 0.0, "line8"+to_string(i));
        viewer->addLine(pcl::PointXYZ(boundingBoxs[i][0].x, boundingBoxs[i][0].y, boundingBoxs[i][0].z), pcl::PointXYZ(boundingBoxs[i][4].x, boundingBoxs[i][4].y, boundingBoxs[i][4].z), 1.0, 0.0, 0.0, "linea"+to_string(i));
        viewer->addLine(pcl::PointXYZ(boundingBoxs[i][1].x, boundingBoxs[i][1].y, boundingBoxs[i][1].z), pcl::PointXYZ(boundingBoxs[i][5].x, boundingBoxs[i][5].y, boundingBoxs[i][5].z), 1.0, 0.0, 0.0, "lineb"+to_string(i));
        viewer->addLine(pcl::PointXYZ(boundingBoxs[i][2].x, boundingBoxs[i][2].y, boundingBoxs[i][2].z), pcl::PointXYZ(boundingBoxs[i][6].x, boundingBoxs[i][6].y, boundingBoxs[i][6].z), 1.0, 0.0, 0.0, "linec"+to_string(i));
        viewer->addLine(pcl::PointXYZ(boundingBoxs[i][3].x, boundingBoxs[i][3].y, boundingBoxs[i][3].z), pcl::PointXYZ(boundingBoxs[i][7].x, boundingBoxs[i][7].y, boundingBoxs[i][7].z), 1.0, 0.0, 0.0, "lined"+to_string(i));
    }
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return;
}

void visualizeInnerPoints(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> innerPoints) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    for (int i = 0; i < innerPoints.size(); i++) {
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(innerPoints[i], "z");
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZI>(innerPoints[i], fildColor, "sample cloud" + to_string(i));
    }
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return;
}

float* callsByPython(const char* file_index, const char* visWhat){
    string binFile = "/home/newDisk/SUSCape/dataset/lidar/scene-000000/lidar/" + string(file_index) + ".pcd";
    string labelFile = "/home/newDisk/SUSCape/dataset/label/scene-000000/label/" + string(file_index) + ".json";
    string calibFile = "/home/newDisk/SUSCape/dataset/calib/scene-000000/calib/aux_lidar/front.json";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = loadPointCloud(binFile);

    vector<Json::Value> objs = readLabel(labelFile);
    vector<Json::Value> calib = readCalib(calibFile);

    vector<vector<float>> result;

    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> innerPoints = getInnerPoints(cloud, objs, result);

    // 将result转换为数组
    float* result_array = new float[result.size() * 4 + 1];
    result_array[0] = result.size() * 4;
    for (int i = 0; i < result.size(); i++) {
        result_array[i * 4 + 1] = result[i][0];
        result_array[i * 4 + 2] = result[i][1];
        result_array[i * 4 + 3] = result[i][2];
        result_array[i * 4 + 4] = result[i][3];
    }
    
    if (string(visWhat) == "bbox") {
        visualizeWithBbox(cloud, objs, calib);
    } else if (string(visWhat) == "inner") {
        visualizeInnerPoints(innerPoints);
    }

    return result_array;
}




int main(int argc, char *argv[])
{
    if (argc != 3) {
        cerr << "Usage: " << argv[0] << " <file_index> <visWhat>" << endl;
        exit(EXIT_FAILURE);
    }
    string file_index = argv[1];
    string visWhat = argv[2];
    string binFile = "/home/newDisk/SUSCape/dataset/lidar/scene-000000/lidar/" + file_index + ".pcd";
    string labelFile = "/home/newDisk/SUSCape/dataset/label/scene-000000/label/" + file_index + ".json";
    string calibFile = "/home/newDisk/SUSCape/dataset/calib/scene-000000/calib/aux_lidar/front.json";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = loadPointCloud(binFile);

    vector<Json::Value> objs = readLabel(labelFile);
    vector<Json::Value> calib = readCalib(calibFile);
    vector<vector<float>> result;

    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> innerPoints = getInnerPoints(cloud, objs, result);

    // 打印结果
    for (int i = 0; i < result.size(); i++) {
        cout << "obj_id: " << result[i][0] << " distance: " << result[i][1] << " count: " << result[i][2] << " bbox_volume: " << result[i][3] << endl;
    }
    
    if (visWhat == "bbox") {
        visualizeWithBbox(cloud, objs, calib);
    } else if (visWhat == "inner") {
        visualizeInnerPoints(innerPoints);
    }

    return 0;
}
