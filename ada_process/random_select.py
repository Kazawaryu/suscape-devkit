import os

import numpy as np

# 在指定目录下随机选取152个文件夹
root_path = "/lab/haoq_lab/30007451/dataset/suscape/"
scene_list = os.listdir(root_path)
scene_list = np.random.choice(scene_list, 50, replace=False)

# 将选取的文件夹移动到指定目录
save_path = "/lab/haoq_lab/30007451/dataset/ada_val/"
for scene in scene_list:
    os.system(f"mv {os.path.join(root_path, scene)} {save_path}")
    print(f"Moving {scene} to {save_path}")
