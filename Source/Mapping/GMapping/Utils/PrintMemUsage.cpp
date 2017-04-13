#include "PrintMemUsage.h"

namespace NS_GMapping
{
  
  using namespace std;
  void
  printmemusage ()
  {
    pid_t pid = getpid ();
    char procfilename[1000];
    sprintf (procfilename, "/proc/%d/status", pid);
    ifstream is (procfilename);
    string line;
    while (is)
    {
      is >> line;
      if (line == "VmData:")
      {
        is >> line;
        cout << "#VmData:\t" << line << endl;
      }
      if (line == "VmSize:")
      {
        is >> line;
        cout << "#VmSize:\t" << line << endl;
      }
      
    }
  }

}
;

