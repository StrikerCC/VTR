//
// Created by cheng.chen05 on 7/14/2022.
//

#include "Cam.h"

Cam::Cam() {

}

Cam::~Cam() {

}

void Cam::SetCamParaFromFile(const std::string &file_path_cam_para) {
    std::string key_resolution = "resolution", key_intrinsic = "intrinsic", key_extrinsic = "extrinsic";
    std::ifstream file_json (file_path_cam_para);
    nlohmann::json json_cam_para;

    // setup camera intrinsic
    file_json >> json_cam_para;
    if (json_cam_para.contains(key_resolution)) {
        this->intrinsic.SetIntrinsics(json_cam_para[key_resolution][0], json_cam_para[key_resolution][1],
                                json_cam_para[key_intrinsic][0][0], json_cam_para[key_intrinsic][1][1],
                                json_cam_para[key_intrinsic][0][2], json_cam_para[key_intrinsic][1][2]);
    }

    // setup camera pose in world frame
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            this->extrinsic(i, j) = (double) json_cam_para[key_extrinsic][i][j];
        }
    }
}
