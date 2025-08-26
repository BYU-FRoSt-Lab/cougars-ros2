# Task.msg
uint32 task_id

TaskType task_type
TaskPriority priority
TaskStatus status
builtin_interfaces/Time time_started
builtin_interfaces/Time time_ended


# task specific information
Waypoint waypoint
uint32 duration

# Nested tasks
Task[] subtasks
