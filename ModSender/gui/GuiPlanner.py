import matplotlib.pyplot as plt
import numpy as np


def draw_zigzag(ax, x_start, y_start, length, num_zigs, angle):
    # Convert acute angle to radians
    angle_rad = np.radians(angle)

    # Calculate the zigzag step size
    zigzag_step = length / (2 * num_zigs)

    # Calculate the horizontal and vertical distances between zigzags
    horizontal_distance = zigzag_step
    vertical_distance = horizontal_distance * np.tan(angle_rad)

    # Calculate the coordinates for the zigzag pattern
    x_coords = [x_start]
    y_coords = [y_start]

    for i in range(num_zigs):
        x1 = x_coords[-1] + horizontal_distance
        y1 = y_coords[-1] + vertical_distance
        x2 = x1 + horizontal_distance
        y2 = y1 - vertical_distance
        x_coords.extend([x1, x2])
        y_coords.extend([y1, y2])

    # Draw the zigzag pattern
    ax.plot(x_coords, y_coords, 'k-')

def draw_zigzag_arrows(ax, x_start, y_start, length, num_zigs, angle):
    # Convert angle to radians
    angle_rad = np.radians(angle)

    # Calculate the zigzag step size
    zigzag_step = length / (2 * num_zigs)

    # Calculate the horizontal and vertical distances between zigzags
    horizontal_distance = zigzag_step
    vertical_distance = horizontal_distance * np.tan(angle_rad)

    # Draw the zigzag pattern using arrows
    for i in range(num_zigs):
        x1 = x_start + i * 2 * horizontal_distance
        y1 = y_start
        x2 = x1 + horizontal_distance
        y2 = y_start + vertical_distance
        x3 = x2 + horizontal_distance
        y3 = y_start
        ax.arrow(x1, y1, x2 - x1, y2 - y1, head_width=0.1, head_length=0.2, fc='k', ec='k')
        ax.arrow(x2, y2, x3 - x2, y3 - y2, head_width=0.1, head_length=0.2, fc='k', ec='k')


# Define the rectangle parameters
x_start = 1
y_start = 1
width = 51  # 168 ft
height = 17  # 56 ft
num_zigs = 6
scaling_factor = 0.5  # Set the scaling factor for the smaller zigzag


def draw_rectangle(ax, x_start, y_start, width, height, color='k-'):
    x_end = x_start + width
    y_end = y_start + height

    # Draw the top horizontal line
    ax.plot([x_start, x_end], [y_start, y_start], color)

    # Draw the bottom horizontal line
    ax.plot([x_start, x_end], [y_end, y_end], color)

    # Draw the left vertical line
    ax.plot([x_start, x_start], [y_start, y_end], color)

    # Draw the right vertical line
    ax.plot([x_end, x_end], [y_start, y_end], color)


def draw_goal(ax, x, y, width=2, color='b'):
    ax.plot([x, x], [y-width/2, y+width/2], color)

def draw_field(ax):
    draw_rectangle(ax, -width/2, -height/2, width, height, 'k')  # Main field
    draw_rectangle(ax, -width / 2 - 12, -height / 2, 12, height, '--k') # extended field

    # draw green goals
    draw_goal(ax, 6,0, color='g')
    draw_goal(ax, 2*6, 6, color='g')
    draw_goal(ax, 2*6, -6, color='g')
    # draw orange goals
    draw_goal(ax, -6, 0, color='r')
    draw_goal(ax, -2 * 6, 6, color='r')
    draw_goal(ax, -2 * 6, -6, color='r')


if __name__ == "__main__":
    # Create a figure and axis
    fig, ax = plt.subplots()

    draw_field(ax)

    # draw zigzag
    draw_zigzag_arrows(ax,0,0,5,3,80)

    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid()
    plt.show()