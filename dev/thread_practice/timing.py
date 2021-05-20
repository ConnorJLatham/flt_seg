from queue import Queue
from threading import Thread
from datetime import datetime
import os
import time
import random
import csv
import functools
import sys

path = 'data/'
path = path + f'{datetime.now().strftime("%Y_%m_%d_%H_%M_%S")}/'
dir = os.mkdir(path)
file = None
data_to_send = {
        'time': 0.0,
        't_delta': 0.0,
        'freq': 0.0,
        'a_1': 0.0,
        'g_1': 0.0,
        'm_1': 0.0,
        'a_2': 0.0,
        'g_2': 0.0,
        'm_2': 0.0,
        'a_3': 0.0,
        'g_3': 0.0,
        'm_3': 0.0,
        'z_1': 0.0
    }


def thread_func(rate=50.0):
    collect_rate = rate
    def deco_func(func):
        @functools.wraps(func)
        def inner(*args, **kwargs):
            t_2 = 1.0
            t_1 = 0.0
            t_adjust = 0.0

            while True:
                t_delta = t_2 - t_1
                t_freq = 1 / t_delta
                t_1 = time.time()
                # print(f'time delta: {t_delta}')

                func(*args, rate=t_freq, **kwargs)

                if t_freq < collect_rate:
                    t_adjust += 0.00005
                if t_freq > collect_rate:
                    t_adjust -= 0.00005

                time.sleep(1 / collect_rate - t_adjust)
                t_2 = time.time()
        return inner
    return deco_func

def init_files(path):
    storage_file = open(path + f'{str(time.time())[-5:]}_storage_file.csv', 'w')
    process_file = open(path + f'{str(time.time())[-5:]}_process_file.txt', 'w')
    return storage_file, process_file

def start_threads(file):
    data_q1 = Queue()
    data_q2 = Queue()
    data_q3 = Queue()
    alarm_q1 = Queue()
    cmd_q1 = Queue()

    alarm_q = alarm_q1
    cmd_q = cmd_q1
    data_collect_rate_q = data_q3

    t1 = Thread(target=collect_all_data, args=(data_q1, data_q2, data_q3))
    # t2 = Thread(target=evaluate_devices, args=(data_q2, alarm_q1,))
    # t3 = Thread(target=commandler, args=(cmd_q1,))
    t4 = Thread(target=store_data, args=(data_q1, file,))

    threads = [t1, t4]
    for thread in threads:
        thread.daemon = True

    t1.start()
    t4.start()


@thread_func(rate=50.0)
def collect_all_data(data_q1, data_q2, data_q3, **kwargs):

    data_to_send['time'] = time.time()
    # data_to_send['t_delta'] = t_delta
    data_to_send['freq'] = kwargs.get('rate', 0.0)
    data_to_send['a_1'] = random.uniform(0, 5)
    data_to_send['g_1'] = random.uniform(0, 5)
    data_to_send['m_1'] = random.uniform(0, 5)
    data_to_send['a_2'] = random.uniform(0, 5)
    data_to_send['g_2'] = random.uniform(0, 5)
    data_to_send['m_2'] = random.uniform(0, 5)
    data_to_send['a_3'] = random.uniform(0, 5)
    data_to_send['g_3'] = random.uniform(0, 5)
    data_to_send['m_3'] = random.uniform(0, 5)
    data_to_send['z_1'] = random.uniform(0, 5)

    data_q1.put(data_to_send)

@thread_func(rate=100.0)
def store_data(data_q1, storage_file, **kwargs):

    # print(data_q1.qsize())
    if data_q1.qsize() >= 1:
        data = data_q1.get()
        writer.writerow(data)
        storage_file.flush()




file1, file2 = init_files(path)
fieldnames = data_to_send.keys()
writer = csv.DictWriter(file1, fieldnames=fieldnames)
writer.writeheader()
start_threads(file1)
sys.setswitchinterval(5e-4)  # just sorta works?

t1 = 0.0
while True:
    t_adjust = 0.0
    control_rate = 25
    t2 = time.time()
    t_delta = t2 - t1

    file2.write(f'Delta: {t_delta}\n')
    file2.flush()

    list_y = [random.uniform(0, 1000) for i in range(0, 1000)]
    fun_stuff = sum(list_y)

    if 1/t_delta <= .95*control_rate:
        t_adjust += 0.0001
    if 1/t_delta >= 1.05*control_rate:
        t_adjust -= 0.0001

    t1 = time.time()
    time.sleep((1/control_rate) - t_adjust)

