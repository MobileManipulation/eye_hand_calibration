#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/geometry.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <string>

void print_usage()
{
	std::cout << "Usage: plane_fit merged_csv" << std::endl;
}

int main(int argc, char* argv[])
{
	if (argc < 2) {
		std::cout << "argc = " << argc << std::endl;
		print_usage();
		for (int i =0; i < argc; i++)
			std::cout << argv[i] << std::endl;
		return 1;
	}

	// Input file
	boost::filesystem::path input(argv[1]);
	auto path = input.parent_path();
	auto file = input.filename();

	std::cout << "Input: " << input << std::endl;
	std::cout << "Path: " << path << std::endl;
	std::cout << "File: " << file << std::endl;

  // Set up output file
  boost::filesystem::path output("plane_fit.csv");
  auto output_path = path / output;
  std::ofstream ofs(output_path.string());

	// Open CSV file and loop over lines
	std::ifstream fp(input.string());
	std::string line;
	while(getline(fp, line))
	{
    boost::char_separator<char> sep(",");
    boost::tokenizer<boost::char_separator<char>> tok(line, sep);

    auto it = tok.begin();
    std::string cluster(*it++);
    std::string datapoint(*it++);
    std::string frame(*it++);

    // Sanitize input...
    if (frame.length() < 2)
      frame = "0" + frame;

    boost::filesystem::path pcd(cluster + "/" + datapoint
                    + "/" + datapoint + "_" + frame + "_cropped.pcd");

    auto pcd_path = path / pcd;
    std::cout << pcd_path << boost::filesystem::exists(pcd_path) << std::endl;

		// Read PCD File
		std::cout << "Loading PCD: " << pcd_path << std::endl;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_path.string(), *cloud) == -1)
		{
			std::cout << "Error! Couldn't load PCD file." << std::endl;
			return 1;
		}

    // Perform plane fit
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return (-1);
    }

    std::cout << "Model Coefficients: " << *coefficients << std::endl;

		// Save plane fit parameters
    ofs << cluster << "," << datapoint << "," << frame;
    for (auto i = 0; i < 4; i++)
      ofs << "," << coefficients->values[i];
    ofs << std::endl;
	}
	return 0;
}
