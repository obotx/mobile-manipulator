"""
Orchestrates a sequence of tasks, advancing to the next when the current completes.
"""
from typing import List, Optional
from tasks.base import Task, TaskOutput


class TaskSequence:
    """
    Runs a list of tasks in order.
    When one task completes, automatically advances to the next.
    """

    def __init__(self):
        self._tasks: List[Task] = []
        self._current_index: int = 0
        self._started: bool = False
        self._complete: bool = False

    def add(self, task: Task) -> None:
        """Append a task to the sequence."""
        self._tasks.append(task)

    def start(self, current_time: float) -> None:
        """Start the sequence (activates the first task)."""
        if not self._tasks:
            self._complete = True
            return
        self._started = True
        self._current_index = 0
        self._tasks[0].on_enter(current_time)

    def update(self, current_time: float, dt: float) -> TaskOutput:
        """
        Step the current task. If complete, advance to the next.

        Returns:
            TaskOutput from the currently active task.
        """
        if not self._started or self._complete:
            return TaskOutput()

        current_task = self._tasks[self._current_index]
        output = current_task.update(current_time, dt)

        if current_task.is_complete(current_time):
            current_task.on_exit()
            self._current_index += 1
            if self._current_index >= len(self._tasks):
                self._complete = True
            else:
                self._tasks[self._current_index].on_enter(current_time)

        return output

    @property
    def is_complete(self) -> bool:
        return self._complete

    @property
    def current_task_name(self) -> Optional[str]:
        if not self._started or self._complete:
            return None
        return self._tasks[self._current_index].name

    @property
    def progress(self) -> float:
        """Returns progress as a fraction [0, 1]."""
        if not self._tasks:
            return 1.0
        return self._current_index / len(self._tasks)