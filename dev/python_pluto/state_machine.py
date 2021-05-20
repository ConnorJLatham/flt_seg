import time
from datetime import datetime
from create_threads import loop

class StateMachine:

    def __init__(self, initial_state):
        self.current_state = initial_state
        self.queue_state = initial_state
        self.next_state = initial_state
        self.sys_start_time = time.time()
        self.state_list = []
        self.control_freq = 20.0

    @loop(rate=20.0)
    def update_mission(self, **kwargs):
        self.state_list[self.current_state].state_update()
        self.current_state = self.next_state

    @property
    def sys_time(self):
        return (time.time() - self.sys_start_time)

class State:

    def __init__(self, state_machine, **kwargs):
        self.phase = 'enter'
        self.tasks = {
            'enter': [],
            'active': [],
            'exit': [],
        }
        self.state_machine = state_machine
        self.tasks['enter'] = kwargs.get('enter_tasks', [])
        self.tasks['active'] = kwargs.get('active_tasks', [])
        self.tasks['exit'] = kwargs.get('exit_tasks', [])

    def state_update(self):
        # get mission time
        mission_time = self.sys_time

        # enter tasks, then put ourselves into active
        if self.phase == 'enter':
            for task in self.tasks['enter']:
                task.perform(mission_time)
            self.phase = 'active'  # python short circuits so we should be safe here

        # go into active after enter tasks, evaluate based on time deltas
        elif self.phase == 'active':
            for task in self.tasks[self.phase]:
                if mission_time - task.task_time >= task.task_delta:
                    task.perform(mission_time)

        # run exit tasks, set state up for next entrance, push state machine along
        else:
            for task in self.tasks['exit']:
                task.perform(mission_time)
            self.state_machine.next_state = self.state_machine.queue_state
            self.phase = 'enter'

        # if the state machine calls to move into a new state, change our phase to begin leaving
        if self.state_machine.current_state != self.state_machine.queue_state:
            self.phase = 'exit'

    @property
    def sys_time(self):
        return self.state_machine.sys_time

class Task:
    def __init__(self, task_func, task_delta=1.0):
        self.task_func = task_func
        self.task_time = 0.0
        self.task_delta = task_delta

    def perform(self, task_time):
        self.task_time = task_time
        self.task_func()