import os

# 指定要搜索的文件夹路径
root_path = '/home/hxq/urfc_ws/src'

# 定义要搜索的关键字
search_keyword = 'pose'

# 遍历文件夹及其子文件夹中的所有文件
for dirpath, dirnames, filenames in os.walk(root_path):
    for filename in filenames:
        # 如果文件名中包含关键字，则输出该文件的路径
        if search_keyword in filename:
            file_path = os.path.join(dirpath, filename)
            print(file_path)
