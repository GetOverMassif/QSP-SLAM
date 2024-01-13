#include <iostream>           
#include <pcl/io/pcd_io.h>      
#include <pcl/point_types.h>     
using namespace std;

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
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    bin_file.write((char*)&cloud->points[i].x,3*sizeof(float)); 
    bin_file.write((char*)&cloud->points[i].intensity,sizeof(float));
  }

  bin_file.close();
}

int main(int argc, char **argv)
{
    assert(argc==3);
    std::string pcd_file = std::string(argv[1]);
    std::string bin_file = std::string(argv[2]);
    cout << "pcd_file: " << pcd_file << std::endl;
    cout << "bin_file: " << bin_file << std::endl;
    pcd2bin(pcd_file, bin_file);
    return 0;
}
