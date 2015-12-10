#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>


void show_usage(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
  std::cout << "--side: Side of the cube (mm)" << std::endl;
  std::cout << "--samples: Number of points / side" << std::endl;
  std::cout << "-o: Output file" << std::endl;
}

int main (int argc, char** argv)
{
    //-- input data
    int side = 10;
    int samples = 100;
    std::string filename = "cube.pcd";

    //-- Show usage
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        show_usage(argv[0]);
        return 0;
    }

    //-- Read from terminal arguments
    if (pcl::console::find_switch(argc, argv, "--side"))
        pcl::console::parse_argument(argc, argv, "--side", side);

    if (pcl::console::find_switch(argc, argv, "--samples"))
        pcl::console::parse_argument(argc, argv, "--samples", samples);

    if (pcl::console::find_switch(argc, argv, "-o"))
        pcl::console::parse_argument(argc, argv, "-o", filename);

    //-- Fill in the cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width  = pow(100,3);
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (int k = 0; k < samples; k++)
        for (int j = 0; j < samples; j++)
            for (int i = 0; i < samples; i++)
            {
                cloud->points[i+j*samples+k*samples].x = i * samples;
                cloud->points[i+j*samples+k*samples].y = j * samples;
                cloud->points[i+j*samples+k*samples].z = k * samples;
            }

    //-- Save to file
    pcl::io::savePCDFileASCII(filename, *cloud);

    return 0;
}
