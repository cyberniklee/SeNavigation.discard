#include <cstdio>  // for EOF
#include <string>
#include <sstream>
#include <vector>
#include "ArrayParser.h"

namespace NS_CostMap
{
  
  /** @brief Parse a vector of vector of floats from a string.
   * @param input
   * @param error_return
   * Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...] */
  std::vector<std::vector<float> >
  parseVVF (const std::string& input, std::string& error_return)
  {
    std::vector<std::vector<float> > result;
    
    std::stringstream input_ss (input);
    int depth = 0;
    std::vector<float> current_vector;
    while (!!input_ss && !input_ss.eof ())
    {
      switch (input_ss.peek ())
      {
        case EOF:
          break;
        case '[':
          depth++;
          if (depth > 2)
          {
            error_return = "Array depth greater than 2";
            return result;
          }
          input_ss.get ();
          current_vector.clear ();
          break;
        case ']':
          depth--;
          if (depth < 0)
          {
            error_return = "More close ] than open [";
            return result;
          }
          input_ss.get ();
          if (depth == 1)
          {
            result.push_back (current_vector);
          }
          break;
        case ',':
        case ' ':
        case '\t':
          input_ss.get ();
          break;
        default:  // All other characters should be part of the numbers.
          if (depth != 2)
          {
            std::stringstream err_ss;
            err_ss << "Numbers at depth other than 2. Char was '"
                << char (input_ss.peek ()) << "'.";
            error_return = err_ss.str ();
            return result;
          }
          float value;
          input_ss >> value;
          if (!!input_ss)
          {
            current_vector.push_back (value);
          }
          break;
      }
    }
    
    if (depth != 0)
    {
      error_return = "Unterminated vector string.";
    }
    else
    {
      error_return = "";
    }
    
    return result;
  }

}  // end namespace costmap_2d
