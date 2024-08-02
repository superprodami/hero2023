
/*定义前馈控制器的结构体*/
typedef struct{
  float rin;
  float lastRin;
  float perrRin;
	float a;
	float b;
}FFC;
float FeedforwardController(FFC *vFFC, const double set);
