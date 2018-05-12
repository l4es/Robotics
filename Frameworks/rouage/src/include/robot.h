#include "module_p.h";
#include "daemonizer.h";

class Robot : public Module_p{
 protected:
  Daemonizer *daemonizer;
 public:
  Robot();
  ~Robot();
};
