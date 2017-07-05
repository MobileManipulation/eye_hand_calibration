#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/geometry.h>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include <iostream>
#include <string>
#include <vector>

void print_usage()
{
	std::cout << "Usage: In experiment dir:" << std::endl;
	std::cout << "./csv_merge1 out_file" << std::endl;
}

int main(int argc, char* argv[])
{
	std::string out_file(argc == 2 ? argv[1] : "merged.csv");
	std::ofstream ofs(out_file);

	// Loop over all cluster directories
	boost::filesystem::path current_dir(".");
	for(auto& cluster : boost::make_iterator_range(boost::filesystem::directory_iterator(current_dir), {}))
	{
		if(boost::filesystem::is_directory(cluster))
		{
			auto cluster_num = cluster.path().filename().string();
			std::cout << "Cluster: " << cluster_num << std::endl;

			// Loop over all datapoints in the cluster
			for(auto& csv : boost::make_iterator_range(boost::filesystem::directory_iterator(cluster.path()), {}))
			{
				if(csv.path().extension() == ".csv")
				{
					auto csv_file = csv.path().filename();
					auto dp_num = csv_file.string().substr(0, 3);
					std::cout << "CSV file: " << csv_file.string() << "; dp_num = " << dp_num << std::endl;

					// Open CSV file and loop over lines
					std::ifstream ifs(csv.path().string());
					std::string line;
					while(getline(ifs, line))
					{
						// Grab the relevant information from the line
						// Things we need:
						// pan/tilt
						// joint angles
						// AR tag location (XYZ-quat)

						// std::vector<std::string> out_columns;

						// out_columns.push_back(cluster_num);
						// out_columns.push_back(dp_num);
						//
						// out_columns.push_back(line.substr())
						//
						// auto frame_str = line.substr(0, 1); // !! This will break if we use more than 10 frames...
						// auto ar_x = line.substr(353, 13);
						// auto ar_y = line.substr(367, 13);
						// auto ar_z = line.substr(381, 13);
						// auto ar_ax = line.substr(395, 13);
						// auto ar_ay = line.substr(408, 13);
						// auto ar_az = line.substr(422, 13);
						// auto ar_aw = line.substr(436, 13);
						ofs << cluster_num << "," << dp_num << "," << line << std::endl;
					}
					ifs.close();
				}
			}
			std::cout << std::endl;
		}
	}
	ofs.close();
	return 0;
}
