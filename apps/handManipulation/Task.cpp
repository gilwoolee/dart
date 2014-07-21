#include "Task.h"

namespace tasks {
Task::Task(dart::dynamics::Skeleton* _model)
  : mModel(_model),
    mFinish(false)
{
}
} // namespace tasks
