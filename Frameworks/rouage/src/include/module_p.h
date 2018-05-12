#include "parser.h";
#include "module.h"

classe Module_p : public Module{
 public:
  Module_p();
  ~Module_p();
 protected:
  Parser *parser;
}
