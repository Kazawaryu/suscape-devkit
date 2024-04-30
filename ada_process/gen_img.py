# read the sub dir in the path, then write their name in a txt file
import os


def write_txt(path):
    save_path = os.path.join(path, "val.txt")
    with open(save_path, "w") as f:
        for dir in os.listdir(path):
            f.write(dir + "\n")


if __name__ == "__main__":
    path = "/lab/haoq_lab/30007451/dataset/ada_val"
    write_txt(path)
