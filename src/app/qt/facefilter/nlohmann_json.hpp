#ifndef nlohmann_json_hpp
#define nlohmann_json_hpp

#ifdef __ANDROID__
// Workaround for Android NDK builds (version r10e) that does not support std::to_string and std::strold so far
namespace std
{
   template <typename T>
   std::string to_string(T Value)
   {
       std::ostringstream TempStream;
       TempStream << Value;
       return TempStream.str();
   }

   inline long double strtold(const char * str, char ** str_end)
   {
       return strtod(str, str_end);
   }

   int stoi( const std::string& str, std::size_t* pos = 0, int base = 10 )
   {
        const char* begin = str.c_str() ;
        char* end = nullptr ;
        long value = std::strtol( begin, &end, base ) ;

        if( errno == ERANGE || value > std::numeric_limits<int>::max() )
        {
            throw std::out_of_range( "stoi: out ofrange" ) ;
        }

        if( end == str.c_str() )
        {
            throw std::invalid_argument( "stoi: invalid argument" ) ;
        }

        if(pos) *pos = end - begin ;

        return value ;
   }
}
#endif

#include "nlohmann/json.hpp" // nlohman-json

#endif // nlohmann_json
