#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/geometry.h>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <string>

void print_usage()
{
	std::cout << "Usage: crop_pointcloud R csv_file [csv_file ...]" << std::endl;
}

int main(int argc, char* argv[])
{
	if (argc < 3) {
		std::cout << "argc = " << argc << std::endl;
		print_usage();
		for (int i =0; i < argc; i++)
			std::cout << argv[i] << std::endl;
		return 1;
	}

	// Radius of interest
	float r = boost::lexical_cast<float>(argv[1]);

	// Loop over all input files
	for (int j = 2; j < argc; j++)
	{
		// Input files
		boost::filesystem::path input(argv[j]);
		auto path = input.parent_path();
		auto file = input.filename();

		std::cout << "Input: " << input << std::endl;
		std::cout << "Path: " << path << std::endl;
		std::cout << "File: " << file << std::endl;

		// First: datapoint number is the first 3 digits of the filename
		auto datapoint = file.string().substr(0, 3);
		std::cout << "Datapoint: " << datapoint << std::endl;

		// Open CSV file and loop over lines
		std::ifstream fp(input.string());
		std::string line;
		while(getline(fp, line))
		{
			// Grab the relevant information from the line
			auto frame_str = line.substr(0, 1); // !! This will break if we use more than 10 frames...
			auto x_str = line.substr(353, 13);
			auto y_str = line.substr(367, 13);
			auto z_str = line.substr(381, 13);
			std::cout << "Frame: " << frame_str << std::endl;
			std::cout << "X" << x_str << std::endl;
			std::cout << "Y" << y_str << std::endl;
			std::cout << "Z" << z_str << std::endl;

			float x = boost::lexical_cast<float>(x_str);
			float y = boost::lexical_cast<float>(y_str);
			float z = boost::lexical_cast<float>(z_str);

			pcl::PointXYZRGB center;
			center.x = x;
			center.y = y;
			center.z = z;

			// Construct the PCD filename
			std::string pcd_prefix = path.string() + "/" + datapoint
					+ "/" + datapoint + "_"
					+ "0" + frame_str + "_";
			std::string raw_pcd = pcd_prefix + "raw_points.pcd";
			std::string crop_pcd = pcd_prefix + "cropped.pcd";

			// Read PCD File
			std::cout << "Loading PCD: " << raw_pcd << std::endl;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(raw_pcd, *cloud) == -1)
			{
				std::cout << "Error! Couldn't load PCD file." << std::endl;
				return 1;
			}

			// Crop pointcloud around XYZ
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZRGB>);
			for (int i = 0; i < cloud->points.size(); i++)
			{
				if (pcl::geometry::distance(center, cloud->points[i]) < r)
				{
					cropped->points.push_back(cloud->points[i]);
				}
			}
			std::cout << "Points after filter: " << cropped->points.size() << std::endl;

			// Save new PCD file
			std::cout << "Writing file: " << crop_pcd << std::endl;
			pcl::io::savePCDFileBinary(crop_pcd, *cropped);

			std::cout << std::endl;
		}
		std::cout << std::endl;
	}

	return 0;

}
