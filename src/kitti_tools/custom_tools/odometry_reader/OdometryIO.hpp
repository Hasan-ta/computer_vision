#include <opencv2/core/core.hpp>
#include <exception>
#include <boost/filesystem.hpp>


namespace perception{
namespace kiiti{

class KittiOdometryIO{

public:
	KiitiOdometryIO();


	virtual ~KittiOdometryIO();


	KittiOdometryIO(const std::string&& path) : parentDirectory_(path)


	void setDataPath(const std::string& path);

private:

	std::string parentDirectory_;

};

}
}