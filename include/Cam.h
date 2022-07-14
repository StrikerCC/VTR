//
// Created by cheng.chen05 on 7/14/2022.
//

#ifndef VTR_CAM_H
#define VTR_CAM_H
#include <string>
#include <fstream>

#include "Eigen/Eigen"
#include "open3d/Open3D.h"
#include "nlohmann/json.hpp"

class Cam {
public:
    Cam();
    ~Cam();

    open3d::camera::PinholeCameraIntrinsic intrinsic;
    Eigen::Matrix4d extrinsic;

    void SetCamParaFromFile(const std::string& file_path_cam_para);
};


#endif //VTR_CAM_H
