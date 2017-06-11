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

    inline std::vector<std::string> split(const std::string& s, char separator)
    {
        std::vector<std::string> ret;
        typedef std::string::size_type string_size;
        string_size i =0;
        while(i != s.size())
        {
            while(i != s.size() && s[i] == separator)
                ++i;

            string_size j = i;
            while(j != s.size() && s[j] != separator)
                ++j;

            if(i != j)
            {
                ret.push_back(s.substr(i,j-i));
                i=j;
            }
        }
        return ret;
    }


}
} 
    
