
class Robot{
 public:
  Robot(int argc,char **argv);
  Robot();
  ~Robot();
  int build(char *filename);
  int init_modules();
  int launch_daemons();
  int loop();
  int shutdown();
  int clean();
}
