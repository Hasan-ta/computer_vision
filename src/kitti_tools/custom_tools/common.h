#include <string>
#include <vector>
#include <sstream>

#define DEBUG

namespace perception{
namespace kitti{

    template <typename T> inline
    T stringToNumber ( const std::string &Text, T defValue = T() )
    {
        std::stringstream ss;
        for ( std::string::const_iterator i=Text.begin(); i!=Text.end(); ++i )
            if ( isdigit(*i) || *i=='e' || *i=='-' || *i=='+' || *i=='.' )
                ss << *i;
        T result;
				#ifdef DEBUG
					std::string ss_string(ss.str());
				#endif
        return ss >> result ? result : defValue;
    }

    /***************************************************************************************
    Separate a string to individual components
    input: 
        - s: input string to be separated
        - separator: the specific chracter used to separate components (usually a comma ",")
    returns:
        - a vector of strings containing separated elements
    ***************************************************************************************/

    inline std::vector<std::string> split(const std::string& s, char delim)
    {
        std::vector<std::string> tokens;
				std::stringstream ss(s);
				std::string item;
				while (std::getline(ss, item, delim)) {
				    tokens.push_back(item);
				}
				return tokens;
    }


}
} 
    
