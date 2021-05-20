import time
import random
import csv
import functools
from in_out import (
    collect_accel,
    collect_gyro,
    collect_mag,
    collect_all_data
)
import sys

sys.setswitchinterval(5e-4)  # just sorta works?


def loop(rate=50.0):
    collect_rate = rate

    def adjust_time(freq, adjustment):
        if freq < collect_rate:
            adjustment -= 0.00005
        elif freq > collect_rate:
            adjustment += 0.00005

        return adjustment

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

                func(*args, freq=t_freq, **kwargs)

                t_adjust = adjust_time(t_freq, t_adjust)
                time.sleep(1 / collect_rate + t_adjust)
                t_2 = time.time()
        return inner
    return deco_func

# may still want this when we are on i2c bus, have to see
# @thread_func(rate=1.0)
# def collect_all_data(data_q1, data_q2, rkt, **kwargs):
#     data = {}
#
#     print(rkt.current_state)
#
#     data['time'] = time.time()
#     data['freq'] = kwargs.get('freq', 0.0)
#     data['a_1'], data['a_2'], data['a_3'] = collect_accel()
#     data['g_1'], data['g_2'], data['g_3'] = collect_gyro()
#     data['m_1'], data['m_2'], data['m_3'] = collect_mag()
#     data['z_1'] = random.uniform(0, 5)
#
#     data_q1.put(data)

def format_csv_file(data_format, storage_file):
    fieldnames = data_format.keys()
    writer = csv.DictWriter(storage_file, fieldnames=fieldnames)
    writer.writeheader()
    return writer

@loop(rate=50.0)
def store_data(storage_file, writer, rkt, **kwargs):
    data = collect_all_data(rkt)
    data['freq'] = kwargs.get('freq', 0.0)
    writer.writerow(data)
    storage_file.flush()

@loop(rate=1.0)
def evaluate_devices(data_q2, alarm_q1, **kwargs):
    if data_q2.qsize() >= 1:
        data_to_eval = data_q2.get()
        if data_to_eval['a_1'][0] > 25.0:
            alarm = 'oh shit your accel is freaking out'
            alarm_q1.put(alarm)


def commandler():
    # does command exist in configs?
    pass