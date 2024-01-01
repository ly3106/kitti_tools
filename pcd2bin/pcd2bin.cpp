/*  This code is to transform the .pcd files to .bin files.
*
*   The point cloud files decoded from the .bag file are usually in .pcd format. 
* In order to facilitate the experiment of 3D detection, the format of the KITTI 
* data set needs to be unified, that is, converted into the .bin format.In the 
* .bin file, each point corresponds to four data, which are xyz and intensity.
* 
*   Input file: a .pcd file containing xyz and intensity
*   Output file: a .bin file in KITTI format
*   Code by FSQ 2018.11.5
**************************************************************************************
本程序在原始项目[pcd2bin](https://github.com/leofansq/Tools_RosBag2KITTI/tree/master/pcd2bin)的基础上修改了主程序,
使的程序使用可以更灵活

编译：
```
mkdir build
cd build
cmake ..
make
```

用法：
1 .将pcd文件复制到本文件同级目录下的`pcd`文件夹下，运行
```
./pcd2bin
```
文件会保存在同级目录下的`bin`文件夹下。

2. 指定包含`pcd`文件的路径和`bin`文件的路径
```
./pcd2bin /home/bit202/pcd /home/bit202/bin
```
生成的`bin`文件在`/home/bit202/bin`中
2024-01-01
**************************************************************************************/

#include <iostream>           
#include <pcl/io/pcd_io.h>      
#include <pcl/point_types.h>     
using namespace std;

//Transform PCD 2 BIN
void pcd2bin (string &in_file, string& out_file)
{ 
   //Create a PointCloud value
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  
  //Open the PCD file
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (in_file, *cloud) == -1) 
  {
    PCL_ERROR ("Couldn't read in_file\n");
  }
  //Create & write .bin file
  ofstream bin_file(out_file.c_str(),ios::out|ios::binary|ios::app);
  if(!bin_file.good()) cout<<"Couldn't open "<<out_file<<endl;  

  //PCD 2 BIN 
  cout << "Converting "
            << in_file <<"  to  "<< out_file
            << endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
  	bin_file.write((char*)&cloud->points[i].x,3*sizeof(float)); 
    bin_file.write((char*)&cloud->points[i].intensity,sizeof(float));
    //cout<< 	cloud->points[i]<<endl;
  }
  	
  bin_file.close();
}
static std::vector<std::string> file_lists;

//Read the file lists
void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL){
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0){
            out_filelsits.push_back(ptr->d_name);
        }else{
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type){
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

bool computePairNum(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}

//Sort the file list
void sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;

    std::sort(filists.begin(),filists.end(),computePairNum);
}

int main(int argc, char **argv) {
    // Default paths
    std::string bin_path = "../bin/";
    std::string pcd_path = "../pcd/";

    // Override default paths if arguments are provided
    if (argc > 1) pcd_path = argv[1];
    if (argc > 2) bin_path = argv[2];

    // Ensure paths end with a '/'
    if (pcd_path.back() != '/') pcd_path += '/';
    if (bin_path.back() != '/') bin_path += '/';

    // Check if pcd_path exists
    if (access(pcd_path.c_str(), 0) == -1) {
        cout << "PCD directory does not exist: " << pcd_path << endl;
        // Handle the non-existence as required
        return 1;
    }

    // Check if bin_path exists
    struct stat info;
    if (stat(bin_path.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) { // Check if directory does not exist
        cout << "BIN directory does not exist. Creating: " << bin_path << endl;
        if (mkdir(bin_path.c_str(), 0777) == -1) { // Create directory with all permissions
            cerr << "Error :  " << strerror(errno) << endl;
            return 1;
        }
    }

    // Read and sort file lists of specific type
    std::vector<std::string> file_lists;
    read_filelists(pcd_path, file_lists, "pcd");
    sort_filelists(file_lists, "pcd");

    // PCD2BIN one by one
    for (int i = 0; i < file_lists.size(); ++i) {
        std::string pcd_file = pcd_path + file_lists[i];
        std::string tmp_str = file_lists[i].substr(0, file_lists[i].length() - 4) + ".bin";
        std::string bin_file = bin_path + tmp_str;
        pcd2bin(pcd_file, bin_file);
    }

    return 0;
}