
/*����ǰ���������Ľṹ��*/
typedef struct{
  float rin;
  float lastRin;
  float perrRin;
	float a;
	float b;
}FFC;
float FeedforwardController(FFC *vFFC, const double set);
