# // Implemented tasks:
# // - Manual Mission
# // - Waypoint Navigation

# // Planned Tasks:
# // - Go to point
# // - Loiter
# // Remote Control

# Task Prioritization:
#  Emergency    -   Requires special override to switch from this
#  Navigation   -   Latest navigation sent is task being carried out
#  Reporting    -   Just reporting, not sure if this node will actually take car of this


# Use an action server

import rclpy
from task_interface.msg import Task, TaskType, TaskPriority, TaskStatus, TaskAction
from task_interface.srv import TaskCommand
from rclpy.node import Node

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager')
        
        TASK_PUBLISH_RATE = 0.5

        self.next_task_id = 0
        self.task_command_service = self.create_service(TaskCommand, 'task_command', self.task_callback)
        self.current_task_publisher = self.create_publisher(Task, 'current_task', 10)
        self.task_manager = TaskManager(self.get_logger(), self.get_clock)
        # timer for current task publisher
        self.current_task_timer = self.create_timer(TASK_PUBLISH_RATE, self.publish_current_task)

    def publish_current_task(self):
        current_task = self.task_manager.get_current_task()
        if current_task:
            self.current_task_publisher.publish(current_task)

    def assign_task_id(self, task):
        task.id = self.next_task_id
        self.next_task_id += 1
        return task.id

    def task_callback(self, request, response):
        action = request.task_action
        match action:
            case TaskAction.COMPLETE:
                response.success = self.task_manager.complete_task(request.task_index)
            case TaskAction.CANCLETASK:
                response.success = self.task_manager.cancel_task(request.task_index)
            case TaskAction.GETTASK:
                response.tasktask = self.task_manager.get_task(request.task_index)
                if task:
                    response.success = True
                else:
                    response.success = False
                return response
            case TaskAction.GETTASKLIST:
                response.task_list = self.task_manager.get_task_list()
                response.success = True
                return response
            case TaskAction.OVERRIDE:
                response.success = self.task_manager.override_emergency_task()
            case TaskAction.APPEND:
                self.assign_task_id(request.task)
                response.success = self.task_manager.append_task(request.task)
            case TaskAction.INSERT:
                self.assign_task_id(request.task)
                response.success = self.task_manager.insert_task(request.task, request.index)
            case _:
                self.get_logger().warning(f"Unknown task action: {action}")
                response.success = False
        return response


class TaskManager():
    def __init__(self, logger, get_clock_func):
        self.task_list = []
        self.task_log = []
        self.logger = logger
        self.get_clock = get_clock_func

    def append(self, task):
        self.task_list.append(task)

    def insert(self, task, index):
        if 0 <= index <= len(self.task_list):
            self.task_list.insert(index, task)
            return True
        return False

    def cancel(self, task):
        if task.priority == TaskPriority.EMERGENCY:
            self.logger.warning("Cannot cancel an emergency task.")
            return False
        if task in self.task_list:
            task.status = TaskStatus.CANCELLED
            task.time_completed = self.get_clock().now()
            self.task_log.append(task)
            self.task_list.remove(task)
            return True
        return False

    def start(self, task):
        if task is not None and task in self.task_list:
            task.status = TaskStatus.RUNNING
            task.time_started = self.get_clock().now()
            return True
        elif task is None:
            # Create and start default task
            default_task = Task()
            default_task.task_type = TaskType.DEFAULT
            default_task.priority = TaskPriority.NAVIGATION
            default_task.status = TaskStatus.RUNNING
            default_task.time_started = self.get_clock().now()
            self.task_list.append(default_task)
            return True
        return False
            

    def complete(self, task):
        if task.priority == TaskPriority.EMERGENCY:
            self.logger.warning("Cannot complete an emergency task.")
            return False
        if task in self.task_list:
            task.status = TaskStatus.COMPLETED
            task.time_ended = self.get_clock().now()
            self.task_log.append(task)
            self.task_list.remove(task)
            self.start(self.get_current_task())
            return True
        return False

    def override_emergency_task(self):
        task = self.get_current_task()
        if task and task.priority == TaskPriority.EMERGENCY:
            self.logger.warning("Overriding emergency task.")
            self.task_list.remove(task)
            task.time_ended = self.get_clock().now()
            task.status = TaskStatus.CANCELLED  # Add status update
            self.task_log.append(task)
            self.start(self.get_current_task())
            return True
        return False

    def get_current_task(self):
        return self.task_list[0] if self.task_list else None

    def get_task(self, index):
        if 0 <= index < len(self.task_list):
            return self.task_list[index]
        return None

    def get_task_list(self):
        return self.task_list

    def append_task(self, task):
        self.task_list.append(task)
        return True

    def insert_task(self, task, index):
        if 0 < index <= len(self.task_list):
            self.task_list.insert(index, task)
            return True
        elif index == 0:
            self.task_list.insert(1, task)
            if self.cancel(self.get_current_task()):
                return True
        return False

    def complete_task(self, task_index):
        if 0 <= task_index < len(self.task_list):
            task = self.task_list[task_index]
            return self.complete(task)
        return False

    def cancel_task(self, task_index):
        if 0 <= task_index < len(self.task_list):
            task = self.task_list[task_index]
            success = self.cancel(task)
            if task_index == 0:  # If we cancelled the current task
                self.start(self.get_current_task())
            return success
        return False


def main(args=None):
    rclpy.init(args=args)
    task_manager_node = TaskManagerNode()
    rclpy.spin(task_manager_node)
    task_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
