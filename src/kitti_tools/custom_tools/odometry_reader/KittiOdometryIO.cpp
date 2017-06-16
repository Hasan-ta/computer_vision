#include "KittiOdometryIO.hpp"

namespace perception{
namespace kitti{


KittiOdometryIO::KittiOdometryIO(const std::string&& path) : parentDirectory_(path)
{
	setDataPath(parentDirectory_);
}

KittiOdometryIO::setDataPath(const std::string& path)
{

{
	using namespace boost::filesystem;

	for (directory_iterator itr(parent_directory); itr!=directory_iterator(); ++itr)
	{
	    if(itr->path().filename() == "sequences" && is_directory(*itr))
	    {
	        training_avail = true; current_path_valid = true;
	        training_path = parent_directory + "/training";
	        list_raw_sequences();
	    }
	    if(itr->path().filename() == "testing" && is_directory(*itr))
	    {
	        testing_avail = true;
	        testing_path = parent_directory + "/testing";
	    }
	}
	if(!current_path_valid)
	    throw std::runtime_error("Invalid Kitti Flow Data Path");
	if(!testing_avail)
	    std::cerr << "WARNING: Path only contains training data. Test data won't be read\n";

}

}
}