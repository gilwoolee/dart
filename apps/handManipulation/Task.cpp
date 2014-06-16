#include "Task.h"
using namespace std;

namespace tasks {
	Task::Task(dynamics::SkeletonDynamics *_model) 
		:mModel(_model)
	{
		mFinish = false;
	}
} // namespace tasks
