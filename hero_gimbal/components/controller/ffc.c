#include "ffc.h"

/*实现前馈控制器*/
float FeedforwardController(FFC *vFFC, const double set)
{
  float result;
  vFFC->rin=set;
  result=vFFC->a*(vFFC->rin-vFFC->lastRin)+vFFC->b*(vFFC->rin-2*vFFC->lastRin+vFFC->perrRin);
 
  vFFC->perrRin= vFFC->lastRin;
  vFFC->lastRin= vFFC->rin;
  return result;
}
