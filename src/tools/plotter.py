import serial
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
import matplotlib.animation as animation
import time


# Serial port informations
COM = 'COM4'
BAUD = 74880


# Theme
background_color = "0.2"
text_color = "white"
line_color = "0.4"
plt.rcParams.update({
    "figure.facecolor": background_color,
    "axes.facecolor": background_color,
    "axes.edgecolor": text_color,
    "axes.labelcolor": text_color,
    "text.color": text_color,
    "xtick.color": text_color,
    "ytick.color": text_color,
    "grid.color": line_color,
    "figure.dpi": 100,
    "axes.grid": True,
    "font.size": 8,
    "grid.linestyle": "--",
    "lines.linewidth": 1,
    "lines.markersize": 2
})


# Variables and constants
min_yaxis = -5
max_yaxis = 5
NBR_VALUES = 1000
NBR_SIGNALS = 11
myTime = time.time_ns()

# create a new figure, with one axis
fig = plt.figure()
ax = plt.axes()

# set the axis limits and labels
ax.set_ylim(min_yaxis, max_yaxis)
ax.set_xlim(0, NBR_VALUES)
ax.set_ylabel('Pressure')
# ax.legend()

# Create an empty list for each signal variables for storing data
x_data = [i for i in range(NBR_VALUES)]
y_data = [[0] * NBR_VALUES for _ in range(NBR_SIGNALS)]

# create an empty line for each signal
lines = [ax.plot([], [])[0] for _ in range(len(y_data))]

# Open the serial port
ser = serial.Serial(COM, BAUD, timeout=1)


# Textbox
def updateMinYAxis(expression):
    if (expression.isnumeric()) :
        global min_yaxis
        min_yaxis = int(expression)
        ax.set_ylim(min_yaxis, max_yaxis)

def updateMaxYAxis(expression):
    if (expression.isnumeric()) :
        global max_yaxis
        max_yaxis = int(expression)
        ax.set_ylim(min_yaxis, max_yaxis)

axbox_min = fig.add_axes((0.2, 0.9, 0.2, 0.075))
text_box_min = TextBox(axbox_min, "Y Min", textalignment="center", hovercolor=background_color, color=background_color)
text_box_min.on_submit(updateMinYAxis)

axbox_max = fig.add_axes((0.6, 0.9, 0.2, 0.075))
text_box_max = TextBox(axbox_max, "Y Max", textalignment="center", hovercolor=background_color, color=background_color)
text_box_max.on_submit(updateMaxYAxis)



def readData():

     # Read a line of data from the serial port
    try:
        # Read and parse data from the serial port
        serial_data = ser.readline()
        data = str(serial_data.decode('utf-8').strip('\r\n'))
        values = data.split(',')
        
        if len(values) == NBR_SIGNALS:
            try:
                # convert the data to float to avoid an issue with matplotlib
                new_data = [float(value) for value in values]
                return new_data

            except ValueError:
                print("Value Error")
                return 0
            except IndexError:
                print("Index Error")
                return 0
        else:
            return 0
    except UnicodeDecodeError:
        print("Unicode Decode Error")
        return 0



# animate function. This is called sequentially
def animate(frame):
    data = readData()
    if data:
        # Append new data to the existing lists and remove the oldest one
        for i in range(NBR_SIGNALS):
            y_data[i].append(data[i])
            y_data[i].pop(0)

        # Update the plot data
        for i in range(NBR_SIGNALS):
            line = lines[i]
            line.set_data(x_data, y_data[i])

    return lines


# call the animation and display it
ani = animation.FuncAnimation(fig, animate, frames=NBR_VALUES, blit=True, interval=1)
plt.show()






