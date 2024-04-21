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

extern "C" float* callsByPython(const char* root_path, const char* file_index, const char* visWhat);
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

struct Point {
    double x;
    double y;
    double z;
};

Point matrixMultiply(const double matrix[3][3], const Point& point) {
    Point result;
    result.x = matrix[0][0] * point.x + matrix[0][1] * point.y + matrix[0][2] * point.z;
    result.y = matrix[1][0] * point.x + matrix[1][1] * point.y + matrix[1][2] * point.z;
    result.z = matrix[2][0] * point.x + matrix[2][1] * point.y + matrix[2][2] * point.z;
    return result;
}

void rotateX(double angle, Point& point) {
    double rotationMatrix[3][3] = {
        {1, 0, 0},
        {0, cos(angle), -sin(angle)},
        {0, sin(angle), cos(angle)}
    };
    point = matrixMultiply(rotationMatrix, point);
}

void rotateY(double angle, Point& point) {
    double rotationMatrix[3][3] = {
        {cos(angle), 0, sin(angle)},
        {0, 1, 0},
        {-sin(angle), 0, cos(angle)}
    };
    point = matrixMultiply(rotationMatrix, point);
}

void rotateZ(double angle, Point& point) {
    double rotationMatrix[3][3] = {
        {cos(angle), -sin(angle), 0},
        {sin(angle), cos(angle), 0},
        {0, 0, 1}
    };
    point = matrixMultiply(rotationMatrix, point);
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

        std::vector<Point> vertices = {
            {x - scale_x / 2, y - scale_y / 2, z - scale_z / 2},
            {x + scale_x / 2, y - scale_y / 2, z - scale_z / 2},
            {x + scale_x / 2, y + scale_y / 2, z - scale_z / 2},
            {x - scale_x / 2, y + scale_y / 2, z - scale_z / 2},
            {x - scale_x / 2, y - scale_y / 2, z + scale_z / 2},
            {x + scale_x / 2, y - scale_y / 2, z + scale_z / 2},
            {x + scale_x / 2, y + scale_y / 2, z + scale_z / 2},
            {x - scale_x / 2, y + scale_y / 2, z + scale_z / 2}
        };

        for (auto& vertex : vertices) {
            vertex.x -= x;
            vertex.y -= y;
            vertex.z -= z;
        }

        for (auto& vertex : vertices) {
            rotateX(rotation_x, vertex);
            rotateY(rotation_y, vertex);
            rotateZ(rotation_z, vertex);
        }

        for (auto& vertex : vertices) {
            vertex.x += x;
            vertex.y += y;
            vertex.z += z;
        }

        vector<pcl::PointXYZ> boundingBox;
        for (auto& vertex : vertices) {
            boundingBox.push_back(pcl::PointXYZ(vertex.x, vertex.y, vertex.z));
        }
        boundingBoxs.push_back(boundingBox);

    }

    return boundingBoxs;
}

vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> getInnerPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<Json::Value> objs, vector<vector<float>> &result) {
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> innerPoints;
    pcl::CropBox<pcl::PointXYZI> cropBoxFilter;
    std::set<string> valid_types = {"Car", "Truck", "Van", "Bus"};
    for (int i = 0; i < objs.size(); i++) {
        string obj_type = objs[i]["obj_type"].asString();
        if (valid_types.count(obj_type) == 0) {
            continue;
        }

        float x = objs[i]["psr"]["position"]["x"].asFloat();
        float y = objs[i]["psr"]["position"]["y"].asFloat();
        float z = objs[i]["psr"]["position"]["z"].asFloat();
        float rotation_x = objs[i]["psr"]["rotation"]["x"].asFloat();
        float rotation_y = objs[i]["psr"]["rotation"]["y"].asFloat();
        float rotation_z = objs[i]["psr"]["rotation"]["z"].asFloat();
        float scale_x = objs[i]["psr"]["scale"]["x"].asFloat();
        float scale_y = objs[i]["psr"]["scale"]["y"].asFloat();
        float scale_z = objs[i]["psr"]["scale"]["z"].asFloat();

        std::vector<Point> vertices = {
            {x - scale_x / 2, y - scale_y / 2, z - scale_z / 2},
            {x + scale_x / 2, y - scale_y / 2, z - scale_z / 2},
            {x + scale_x / 2, y + scale_y / 2, z - scale_z / 2},
            {x - scale_x / 2, y + scale_y / 2, z - scale_z / 2},
            {x - scale_x / 2, y - scale_y / 2, z + scale_z / 2},
            {x + scale_x / 2, y - scale_y / 2, z + scale_z / 2},
            {x + scale_x / 2, y + scale_y / 2, z + scale_z / 2},
            {x - scale_x / 2, y + scale_y / 2, z + scale_z / 2}
        };

        for (auto& vertex : vertices) {
            vertex.x -= x;
            vertex.y -= y;
            vertex.z -= z;
        }

        for (auto& vertex : vertices) {
            rotateX(rotation_x, vertex);
            rotateY(rotation_y, vertex);
            rotateZ(rotation_z, vertex);
        }

        for (auto& vertex : vertices) {
            vertex.x += x;
            vertex.y += y;
            vertex.z += z;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr innerCloud(new pcl::PointCloud<pcl::PointXYZI>);
        cropBoxFilter.setInputCloud(cloud);
        Eigen::Vector4f minPoint, maxPoint;

        cropBoxFilter.setTranslation(Eigen::Vector3f(x, y, z));
        cropBoxFilter.setRotation(Eigen::Vector3f(rotation_x, rotation_y, rotation_z));
        cropBoxFilter.setMin(Eigen::Vector4f(- scale_x / 2, - scale_y / 2, - scale_z / 2, 1.0));
        cropBoxFilter.setMax(Eigen::Vector4f(scale_x / 2, scale_y / 2, scale_z / 2, 1.0));

        cropBoxFilter.filter(*innerCloud);
        
        float distance = sqrt(x * x + y * y + z * z);
        float bbox_volume = scale_x * scale_y * scale_z;
        
        vector<float> temp;
        float idx_ = std::stof(objs[i]["obj_id"].asString());
        temp.push_back(idx_);
        temp.push_back(distance);
        if (innerCloud->points.size() >= 8) {
            // cout << innerCloud->points.size() << endl;
            innerPoints.push_back(innerCloud);
            int mesh_count = getAlphaCount(innerCloud);
            temp.push_back(mesh_count);
        }else{
            temp.push_back(0);
        }
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

void visualizeWithBbox(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, vector<Json::Value> objs) {
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

float* callsByPython(const char* root_path, const char* file_index, const char* visWhat){
    string binFile = string(root_path) + "/lidar/" + string(file_index) + ".pcd";
    string labelFile = string(root_path) + "/label/" + string(file_index) + ".json";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = loadPointCloud(binFile);

    vector<Json::Value> objs = readLabel(labelFile);
    vector<vector<float>> result;

    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> innerPoints = getInnerPoints(cloud, objs, result);

    float* result_array = new float[result.size() * 4 + 1];
    result_array[0] = result.size() * 4;
    for (int i = 0; i < result.size(); i++) {
        result_array[i * 4 + 1] = result[i][0];
        result_array[i * 4 + 2] = result[i][1];
        result_array[i * 4 + 3] = result[i][2];
        result_array[i * 4 + 4] = result[i][3];
    }
    
    if (string(visWhat) == "bbox") {
        visualizeWithBbox(cloud, objs);
    } else if (string(visWhat) == "inner") {
        visualizeInnerPoints(innerPoints);
    }

    return result_array;
}




int main(int argc, char *argv[])
{
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " <file_index> <visWhat>" << endl;
        exit(EXIT_FAILURE);
    }
    string frame_id = argv[1];
    string file_index = argv[2];
    string visWhat = argv[3];
    string binFile = "/home/ghosnp/mirror/mmdet_sandbox/home/dataset/scene-"+frame_id+"/lidar/" + file_index + ".pcd";
    string labelFile = "/home/ghosnp/mirror/mmdet_sandbox/home/dataset/scene-"+frame_id+"/label/" + file_index + ".json";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = loadPointCloud(binFile);

    vector<Json::Value> objs = readLabel(labelFile);
    vector<vector<float>> result;
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> innerPoints = getInnerPoints(cloud, objs, result);

    // 打印结果
    // for (int i = 0; i < result.size(); i++) {
    //     cout << "obj_id: " << result[i][0] << " distance: " << result[i][1] << " count: " << result[i][2] << " bbox_volume: " << result[i][3] << endl;
    // }
    
    if (visWhat == "bbox") {
        visualizeWithBbox(cloud, objs);
    } else if (visWhat == "inner") {
        visualizeInnerPoints(innerPoints);
    }

    return 0;
}
