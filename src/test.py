import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Create some sample data
x = [i * 0.1 for i in range(200)]
y_data = [[math.sin(xi), math.cos(xi), math.tan(xi)] for xi in x]
for i in range(3, 10):
    y_data.append([math.sin(xi * (i + 1)) for xi in x])

# Set up the figure and axis
fig, ax = plt.subplots()
ax.set_xlim(0, 2 * math.pi)
ax.set_ylim(-1.5, 1.5)

# Define the initialization function
def init():
    lines = []
    for i in range(len(y_data)):
        line, = ax.plot([], [], lw=2)
        lines.append(line)
    return lines

# Define the animation function
def animate(i, lines):
    y = []
    for j in range(len(y_data)):
        y_j = []
        for k in range(i):
            if k < len(y_data[j]):
                y_j.append(y_data[j][k])
            else:
                y_j.append(None)
        y.append(y_j)
    for j, line in enumerate(lines):
        line.set_data(x[:i], y[j])
    return lines

# Create the animation object
lines = init()
ani = FuncAnimation(fig, animate, frames=len(x), fargs=(lines,), blit=True, interval=10)

# Show the plot
plt.show()