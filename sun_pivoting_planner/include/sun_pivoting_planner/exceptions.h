#ifndef SUN_PIVOTING_PLANNER_EXCEPTIONS
#define SUN_PIVOTING_PLANNER_EXCEPTIONS

#include <stdexcept>

namespace sun
{

class pivoting_planner_error : public std::runtime_error
{
public:
  pivoting_planner_error(std::string const& msg) : std::runtime_error(msg)
  {
  }
};

class unknown_frame_transform : public pivoting_planner_error
{
public:
  unknown_frame_transform(std::string const& msg) : pivoting_planner_error(msg)
  {
  }
};

class attached_collision_object_not_found : public pivoting_planner_error
{
public:
  attached_collision_object_not_found(std::string const& msg) : pivoting_planner_error(msg)
  {
  }
};

class collision_object_not_found : public pivoting_planner_error
{
public:
  collision_object_not_found(std::string const& msg) : pivoting_planner_error(msg)
  {
  }
};

class invalid_gravity_pivoting_angle : public pivoting_planner_error
{
public:
  invalid_gravity_pivoting_angle(std::string const& msg) : pivoting_planner_error(msg)
  {
  }
};

}  // namespace sun

#endif