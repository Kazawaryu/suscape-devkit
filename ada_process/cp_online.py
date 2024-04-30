# 读取txt_path目录下的txt文件, 对orign_path下的同名文件夹操作,将txt文件中的文件复制到save_path下的文件夹中
import os
import shutil

txt_path = "./../s1s2_txts/"
origin_path = "/lab/haoq_lab/30007451/dataset/online/"
save_path = "/lab/haoq_lab/30007451/dataset/online_s1s2/"  # 如果不存在,创建

if not os.path.exists(save_path):
    os.makedirs(save_path)

# 读取txt文件,并记录txt文件名
txt_files = os.listdir(txt_path)
txt_files = [f for f in txt_files if f.endswith(".txt")]
scene_names = [f.split("_")[0] for f in txt_files]

# 读取txt文件内容
for i in range(len(txt_files)):
    with open(os.path.join(txt_path, txt_files[i]), "r") as f:
        lines = f.readlines()
    lines = [l.strip() for l in lines]

    # 复制文件
    scene_path = os.path.join(origin_path, scene_names[i])
    save_scene_path = os.path.join(save_path, scene_names[i])
    # scene_path下有calib  camera  label  lidar  lidar_pose 四个子文件夹
    # ├── calib
    # │   ├── aux_camera
    # │   │   ├── front
    # │   │   ├── front_left
    # │   │   ├── front_right
    # │   │   ├── rear
    # │   │   ├── rear_left
    # │   │   └── rear_right
    # │   ├── aux_lidar
    # │   ├── camera
    # │   │   ├── front
    # │   │   ├── front_left
    # │   │   ├── front_right
    # │   │   ├── rear
    # │   │   ├── rear_left
    # │   │   └── rear_right
    # │   └── radar
    # ├── camera
    # │   ├── front
    # │   ├── front_left
    # │   ├── front_right
    # │   ├── rear
    # │   ├── rear_left
    # │   └── rear_right
    # ├── label
    # ├── lidar
    # └── lidar_pose

    # 对以上的子文件夹, calib全部复制, camera子文件夹中的front, front_left, front_right, rear, rear_left, rear_right检验复制,label lidar lidar_pose检验复制

    # calib
    calib_save_path = os.path.join(save_scene_path, "calib")
    if not os.path.exists(calib_save_path):
        os.makedirs(calib_save_path)
    shutil.copytree(
        os.path.join(scene_path, "calib"),
        os.path.join(calib_save_path),
        dirs_exist_ok=True,
    )
    # 复制calib目录下的所有文
    # for sub in os.listdir(os.path.join(scene_path, "calib")):
    #    shutil.copytree(
    # os.path.join(scene_path, "calib", sub),
    # os.path.join(calib_save_path, sub),
    # dirs_exist_ok=True,
    # )
    print("Finish copy calib: ", scene_names[i])
    # camera
    sub_dir = ["front", "front_left", "front_right", "rear", "rear_left", "rear_right"]
    camera_save_path = os.path.join(save_scene_path, "camera")
    if not os.path.exists(camera_save_path):
        os.makedirs(camera_save_path)
    for sub in sub_dir:
        camera_sub_save_path = os.path.join(camera_save_path, sub)
        if not os.path.exists(camera_sub_save_path):
            os.makedirs(camera_sub_save_path)
        for index in lines:
            # 只复制index对应的文件
            shutil.copy(
                os.path.join(scene_path, "camera", sub, index + ".jpg"),
                camera_sub_save_path,
            )
    print("Finish copy camera: ", scene_names[i])
    # label
    label_save_path = os.path.join(save_scene_path, "label")
    if not os.path.exists(label_save_path):
        os.makedirs(label_save_path)
    for index in lines:
        shutil.copy(os.path.join(scene_path, "label", index + ".json"), label_save_path)
    print("Finish copy label: ", scene_names[i])
    # lidar
    lidar_save_path = os.path.join(save_scene_path, "lidar")
    if not os.path.exists(lidar_save_path):
        os.makedirs(lidar_save_path)
    for index in lines:
        shutil.copy(os.path.join(scene_path, "lidar", index + ".pcd"), lidar_save_path)
    print("Finish copy lidar: ", scene_names[i])
    # lidar_pose
    lidar_pose_save_path = os.path.join(save_scene_path, "lidar_pose")
    if not os.path.exists(lidar_pose_save_path):
        os.makedirs(lidar_pose_save_path)
    for index in lines:
        shutil.copy(
            os.path.join(scene_path, "lidar_pose", index + ".json"),
            lidar_pose_save_path,
        )
    print("Finish copy lidar_pose: ", scene_names[i])
    print("#" * 50)
