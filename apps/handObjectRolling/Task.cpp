#include "Task.h"

namespace tasks {

//==============================================================================
Task::Task(dart::dynamics::Skeleton* _model)
  : mModel(_model),
    mFinish(false)
{
}

//==============================================================================
Task::~Task()
{
}

//==============================================================================
VectorXd Task::getTorque() const
{
  return mTorque;
}

//==============================================================================
void Task::evalTorque()
{
}

//==============================================================================
MatrixXd Task::getNullSpace() const
{
  return MatrixXd::Zero(1,1);
}

//==============================================================================
MatrixXd Task::getTaskSpace() const
{
  return MatrixXd::Zero(1,1);
}

//==============================================================================
void Task::setDependTask(Task* _task)
{
  mDependTask = _task;
}

//==============================================================================
void Task::setPriority(int _priority)
{
  mPriority = _priority;
}

//==============================================================================
void Task::evalTaskFinish()
{
}

//==============================================================================
void Task::setName(const std::string& _name)
{
  mName = _name;
}

//==============================================================================
const std::string& Task::getName() const
{
  return mName;
}

} // namespace tasks
