from queue import Queue
import time
from threading import Thread
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# A thread that produces data
def alarmer(out_q):
    while True:
        # Produce some data
        value = random.randint(0, 20)
        if value > 15:
            data = f'watch out! your value is {value}'
        else:
            data = None
        if data:
            out_q.put(data)
        time.sleep(.1)

def commander(out_q):
    while True:
        data = 'another thread appears!'
        time.sleep(3)
        out_q.put(data)

# A thread that consumes data
def rocket(in_q):
    while True:
        # Get some data
        time.sleep(1)
        data = in_q.get()
        # Process the data
        print(data)
        print(in_q.qsize())
        while in_q.qsize() > 5:
            data = in_q.get()
            print(data)

def data_gatherer(out_q1, out_q2):
    while True:
        time.sleep(0.1)
        data = random.randint(0, 20)
        print(f'all your data is {data}')
        out_q1.put(data)
        out_q2.put(data)


def data_collector(in_q, file):
    while True:
        data = in_q.get()
        print(f'hey found your data! {data}')
        file.write(f'{data},')
        file.flush()
        time.sleep(.01)

def data_displayer(in_q):
    data = []
    fig = plt.figure()
    ax = fig.add_subplot(111)

    def animate_plot(i):
        print(in_q.qsize())
        data.append(in_q.get())
        x = [x for x in range(0, len(data))]
        y = data
        ax.clear()
        ax.plot(x, y)

    ani = animation.FuncAnimation(fig, animate_plot, interval=10)
    plt.show()

class fun_stuff:
    cool = 3

    @staticmethod
    def running(cool):
        while True:
            time.sleep(1)
            print(cool)

okay = fun_stuff()

# Create the shared queue and launch both threads
q = Queue()
data_q1 = Queue()
data_q2 = Queue()
data_file = open('data_file.csv', 'w')
t1 = Thread(target=rocket, args=(q,))
t2 = Thread(target=alarmer, args=(q,))
t3 = Thread(target=commander, args=(q,))
t4 = Thread(target=data_gatherer, args=(data_q1, data_q2))
t5 = Thread(target=data_collector, args=(data_q1, data_file))
t6 = Thread(target=data_displayer, args=(data_q2, ))
t7 = Thread(target=fun_stuff.running, args=(3))
# t1.start()
# t2.start()
# t3.start()
# t4.start()
# t5.start()
# t6.start()
t7.start()