import os
import shutil
from multiprocessing import Pool

txt_path = "./../s1s2_txts/"
origin_path = "/lab/haoq_lab/30007451/dataset/online/"
save_path = "/lab/haoq_lab/30007451/dataset/online_s1s2/"

if not os.path.exists(save_path):
    os.makedirs(save_path)

# 读取txt文件,并记录txt文件名
txt_files = os.listdir(txt_path)
txt_files = [f for f in txt_files if f.endswith(".txt")]
scene_names = [f.split("_")[0] for f in txt_files]


def copy_scene(scene_info):
    scene_name, lines = scene_info
    scene_path = os.path.join(origin_path, scene_name)
    save_scene_path = os.path.join(save_path, scene_name)

    # 复制calib目录
    calib_save_path = os.path.join(save_scene_path, "calib")
    if not os.path.exists(calib_save_path):
        os.makedirs(calib_save_path)
    shutil.copytree(
        os.path.join(scene_path, "calib"), calib_save_path, dirs_exist_ok=True
    )

    # 复制camera子目录
    sub_dir = ["front", "front_left", "front_right", "rear", "rear_left", "rear_right"]
    camera_save_path = os.path.join(save_scene_path, "camera")
    if not os.path.exists(camera_save_path):
        os.makedirs(camera_save_path)
    for sub in sub_dir:
        camera_sub_save_path = os.path.join(camera_save_path, sub)
        if not os.path.exists(camera_sub_save_path):
            os.makedirs(camera_sub_save_path)
        for index in lines:
            shutil.copy(
                os.path.join(scene_path, "camera", sub, index + ".jpg"),
                camera_sub_save_path,
            )

    # 复制label、lidar和lidar_pose目录
    for dir_name in ["label", "lidar", "lidar_pose"]:
        save_dir_path = os.path.join(save_scene_path, dir_name)
        if not os.path.exists(save_dir_path):
            os.makedirs(save_dir_path)
        for index in lines:
            shutil.copy(
                os.path.join(
                    scene_path,
                    dir_name,
                    index + ".json" if dir_name != "lidar" else index + ".pcd",
                ),
                save_dir_path,
            )

    print(f"Finish copying {scene_name}")


# 创建进程池并执行任务
scene_info_list = []
for i in range(len(txt_files)):
    with open(os.path.join(txt_path, txt_files[i]), "r") as f:
        lines = [l.strip() for l in f.readlines()]
    scene_info_list.append((scene_names[i], lines))

with Pool(processes=8) as pool:
    pool.map(copy_scene, scene_info_list)
