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
from frost_interfaces import Task, TaskType, TaskPriority, TaskStatus
from rclpy.node import Node
from std_msgs.msg import Int8

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.task_manager = TaskManager()
        self.task_subscriber = self.create_subscription(Int8, 'task', self.task_callback, 10)
        self.task_publisher = self.create_publisher(Int8, 'current_task', 10)

    def task_callback(self, msg):
        task = self.task_manager.get_task(msg.data)
        if task:
            self.task_manager.start(task)
            self.task_publisher.publish(Int8(task.id))

class TaskManager():
    def __init__(self):

        self.task_list = []
        self.task_log = []

    def add(self, task):
        self.task_list.append(task)

    def cancel(self, task):
        if task.priority == TaskPriority.EMERGENCY:
            self.get_logger().warning("Cannot cancel an emergency task.")
            return
        if task in self.task_list:
            task.status = TaskStatus.CANCELLED
            task.time_completed = self.get_clock().now()
            self.task_log.append(task)
            self.task_list.remove(task)

    def start(self, task):
        if task in self.task_list:
            task.status = TaskStatus.RUNNING
            task.time_started = self.get_clock().now()
            # publish task
        if task is None:
            pass
            # publish default task

    def complete(self, task):
        if task.priority == TaskPriority.EMERGENCY:
            self.get_logger().warning("Cannot complete an emergency task.")
            return
        if task in self.task_list:
            task.status = TaskStatus.COMPLETED
            task.time_ended = self.get_clock().now()
            self.task_log.append(task)
            self.task_list.remove(task)

    def override_emergency_task(self):
        task = self.get_current_task()
        if task.priority == TaskPriority.EMERGENCY:
            self.get_logger().warning("Overriding emergency task.")
            self.task_list.remove(task)
            task.time_ended = self.get_clock().now()
            self.task_log.append(task)
            self.start(self.get_current_task())

    def get_current_task(self):
        return self.task_list[0] if self.task_list else None

    def cancel_task(self, task_index):
        if 0 <= task_index < len(self.task_list):
            task = self.task_list[task_index]
            self.cancel(task)
            if task_index == 0:
                self.start(self.get_current_task())
