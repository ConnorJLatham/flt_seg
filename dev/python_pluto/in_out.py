import random
import time

def collect_accel():
    # do something
    a_1 = [random.uniform(0, 5) for i in range(0, 3)]
    return a_1

def collect_gyro():
    # do something
    g_1 = [random.uniform(0, 5) for i in range(0, 3)]
    return g_1

def collect_mag():
    # do something
    m_1 = [random.uniform(0, 5) for i in range(0, 3)]
    return m_1

def collect_all_data(rkt, **kwargs):
    data = {}

    print(rkt.current_state)

    data['time'] = time.time()
    data['freq'] = kwargs.get('freq', 0.0)
    data['a_1'], data['a_2'], data['a_3'] = collect_accel()
    data['g_1'], data['g_2'], data['g_3'] = collect_gyro()
    data['m_1'], data['m_2'], data['m_3'] = collect_mag()
    data['z_1'] = random.uniform(0, 5)

    return data