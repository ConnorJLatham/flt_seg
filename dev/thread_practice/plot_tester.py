import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time


data = []
fig = plt.figure()
ax = fig.add_subplot(111)

def animate_plot(i):
    x = [x for x in range(0, len(data))]
    print(x)
    y = data
    print(y)
    ax.clear()
    ax.plot(x, y)
    data.append(random.randint(10, 20))

# while True:
#     time.sleep(0.1)


ani = animation.FuncAnimation(fig, animate_plot, interval=1000)
plt.show()
