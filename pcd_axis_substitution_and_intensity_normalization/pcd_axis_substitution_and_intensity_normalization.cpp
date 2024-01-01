/******************************************************************
读取一个文件夹下的所有pcd文件并保存pcd到另一个文件夹下，
将新的pcd的xyz对应原来点云的y、-x、z，并且将intensity归一化

*******************************************************************
转换前的坐标系，如下：
          z up  y front
            ^    ^
            |   /
            |  /
            | /
            |/
left ------ 0 ------> x right

转换后的坐标系，如下：
             z up  x front
               ^    ^
               |   /
               |  /
               | /
               |/
y left <------ 0 ------ right

*******************************************************************
intensity归一化过程[0, 255] ↦ [0, 1]

*******************************************************************/

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;

int main(int argc, char** argv) {
    std::string source_folder;
    std::string target_folder;
    std::string encoding = "ascii"; // 默认编码方式为ASCII

    // 检查源文件夹参数
    if (argc > 1) {
        source_folder = argv[1];
    } else {
        std::cerr << "Usage: " << argv[0] << " [source folder] [optional: target folder] [optional: encoding (ascii/binary/compressed)]\n";
        return -1;
    }

    // 设置目标文件夹，默认为原目录的上一级的同名文件夹加上"_trans"
    if (argc > 2) {
        target_folder = argv[2];
    } else {
        boost::filesystem::path source_path(source_folder);
        target_folder = source_path.parent_path().string() + "/" + source_path.filename().string() + "_trans";
    }

    // 设置编码方式，默认为ASCII
    if (argc > 3) {
        encoding = argv[3];
    }

    pcl::PCDWriter writer;

    // 检查目标文件夹是否存在，如果不存在则创建
    if (!boost::filesystem::exists(target_folder)) {
        boost::filesystem::create_directories(target_folder);
    }

    // 获取源文件夹中的文件总数
    size_t total_files = std::distance(boost::filesystem::directory_iterator(source_folder),
                                       boost::filesystem::directory_iterator());
    size_t processed_files = 0;

    // 遍历源文件夹中的所有文件
    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator itr(source_folder); itr != end_itr; ++itr) {
        if (boost::filesystem::is_regular_file(itr->path())) {
            std::string current_file = itr->path().string();

            // 检查是否为PCD文件
            if (itr->path().extension() == ".pcd") {
                // 读取PCD文件
                PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
                if (pcl::io::loadPCDFile<pcl::PointXYZI>(current_file, *cloud) == -1) {
                    std::cerr << "Couldn't read file: " << current_file << std::endl;
                    continue;
                }

                // 转换点云数据
                for (auto& point : cloud->points) {
                    float new_x = point.y;  // 新的X是原来的Y
                    float new_y = -point.x; // 新的Y是原来的X的负值
                    point.x = new_x;
                    point.y = new_y;
                    point.intensity /= 255.0; // 假设intensity的范围是0到255，进行归一化
                }

                // 构建新的PCD文件名并保存
                std::string new_file = target_folder + "/" + itr->path().filename().string();

                if (encoding == "binary") {
                    writer.writeBinary(new_file, *cloud);
                } else if (encoding == "compressed") {
                    writer.writeBinaryCompressed(new_file, *cloud);
                } else {
                    writer.writeASCII(new_file, *cloud);
                }

                processed_files++;

                // 打印转换日志和进度
                std::cout << "Processed: " << current_file << " -> " << new_file << " [" << encoding << "]" << std::endl;
                std::cout << "Progress: " << processed_files << "/" << total_files << " files processed.\n";
            }
        }
    }
    std::cout << "Conversion completed. " << processed_files << " out of " << total_files << " files were processed." << std::endl;
    return 0;
}
