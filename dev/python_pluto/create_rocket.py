from defined_states import define_states
from state_machine import StateMachine
from queue import Queue
from threading import Thread
from datetime import datetime
import os
from create_threads import (
    collect_all_data,
    store_data,
    evaluate_devices,
    commandler,
    format_csv_file
)
from data_format import data_to_send





class Rocket(StateMachine):

    pad_accel_vals = [0.0, 0.0, 0.0, 0.0, 0.0]
    path = 'missions/'
    alarm_q = None
    cmd_q = None
    data_collect_rate_q = None

    def __init__(self, state):
        super().__init__(state)
        self.boot_rocket()

    def boot_rocket(self):
        print('Booting rocket')
        define_states(self)
        self.path = self.path + f'{datetime.now().strftime("%Y_%m_%d_%H_%M_%S")}/'
        self.dir = os.mkdir(self.path)
        self.init_files()
        self.start_threads()

    def get_sensors(self, data_rate=1.0):
        print(f'{self.sys_time}: the sys up time is {self.sys_time}')
        if self.sys_time > 5:
            self.queue_state = 1

    def init_sensors(self):
        print(f'{self.sys_time}: starting up sensors!')

    def pad_accel_wait(self):
        accel = 0.0 if self.sys_time < 6.0 else 10
        print(f'[{self.sys_time}]: Accel is reading: {accel}')
        self.pad_accel_vals.append(accel)
        self.pad_accel_vals = self.pad_accel_vals[-5:]
        print(f'full pad accel list: {self.pad_accel_vals}')
        if all(val > 3.0 for val in self.pad_accel_vals):
            print('rocket has taken off!')
            self.queue_state = 2

    def control_cycle(self):
        self.get_commands()
        self.get_alarms()
        self.update_mission()

    def get_alarms(self):
        if self.alarm_q.qsize() > 1:
            print(self.alarm_q.get())

    def get_commands(self):
        pass

    def init_files(self):
        self.storage_file = open(self.path + f'/storage_file.csv', 'w')
        self.csv_writer = format_csv_file(data_to_send, self.storage_file)
        self.control_process_log = open(self.path + f'/control_process_log.txt', 'w')
        self.command_log = open(self.path + f'/command_log.txt', 'w')
        self.alarms_log = open(self.path + f'/alarms_log.txt', 'w')

    def write_to_log(self, info, log):
        log.write(f'[{self.sys_time}]: {info}')

    def start_threads(self):
        data_q1 = Queue()
        data_q2 = Queue()
        data_q3 = Queue()
        alarm_q1 = Queue()
        cmd_q1 = Queue()

        self.alarm_q = alarm_q1
        self.cmd_q = cmd_q1

        t2 = Thread(target=evaluate_devices, args=(data_q2, alarm_q1,))
        t3 = Thread(target=commandler, args=(cmd_q1,))
        t4 = Thread(target=store_data, args=(self.storage_file, self.csv_writer, self))

        self.threads = [t2, t3, t4]
        for thread in self.threads:
            thread.daemon = True

        # t2.start()
        t4.start()

rocket = Rocket(0)


if __name__=='__main__':
    while True:
        rocket.control_cycle()
