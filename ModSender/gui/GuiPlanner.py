import matplotlib.pyplot as plt


def draw_zigzag(ax, x_start, y_start, width, height, num_zigs, scaling_factor):
    x_end = x_start + width
    y_end = y_start + height

    # Calculate the zigzag step size
    zigzag_step = width / (2 * num_zigs)

    # Define a list of colors for the zigzag lines
    zigzag_colors = ['red', 'green', 'blue', 'purple', 'orange', 'pink']

    # Draw the top horizontal line
    ax.plot([x_start, x_end], [y_start, y_start], color=zigzag_colors[0])

    # Draw the bottom horizontal line
    ax.plot([x_start, x_end], [y_end, y_end], color=zigzag_colors[1])

    # Draw the left vertical line
    ax.plot([x_start, x_start], [y_start, y_end], color=zigzag_colors[2])

    # Draw the right vertical line
    ax.plot([x_end, x_end], [y_start, y_end], color=zigzag_colors[3])

    # Draw the larger zigzag pattern
    for i in range(num_zigs):
        x1 = x_start + i * 2 * zigzag_step
        y1 = y_start
        x2 = x1 + zigzag_step
        y2 = y_start + height
        x3 = x2 + zigzag_step
        y3 = y_start
        ax.plot([x1, x2, x3], [y1, y2, y3], color=zigzag_colors[i % len(zigzag_colors)])

    # Draw the smaller zigzag pattern with scaling factor
    for i in range(num_zigs):
        x1 = x_start + i * 2 * zigzag_step + (zigzag_step * scaling_factor)
        y1 = y_start + (height * scaling_factor)
        x2 = x1 + (zigzag_step * scaling_factor)
        y2 = y_start + height
        x3 = x2 + (zigzag_step * scaling_factor)
        y3 = y_start + (height * scaling_factor)
        ax.plot([x1, x2, x3], [y1, y2, y3], color=zigzag_colors[i % len(zigzag_colors)])




# Define the rectangle parameters
x_start = 1
y_start = 1
width = 51  # 168
height = 56
num_zigs = 6
scaling_factor = 0.5  # Set the scaling factor for the smaller zigzag




if __name__ == "__main__":
    # Create a figure and axis
    fig, ax = plt.subplots()

    # Call the function to draw the zigzags inside the rectangle
    draw_zigzag(ax, x_start, y_start, width, height, num_zigs, scaling_factor)

    # Set a different color for the rectangle
    rect = plt.Rectangle((x_start, y_start), width, height, fill=False, color='blue', linewidth=2)
    ax.add_patch(rect)

    # Set axis limits and show the plot
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 5)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()