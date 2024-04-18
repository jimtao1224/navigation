import numpy as np
import matplotlib.pyplot as plt

def generate_maze(size):
    # Initialize the maze with all cells as walls
    maze = np.zeros((size, size), dtype=bool)
    maze[1:-1, 1:-1][np.random.random((size-2, size-2)) > 0.25] = True
    maze[1, 1] = True  # Start point
    maze[-2, -2] = True  # End point

    return maze

def plot_maze(maze):
    plt.figure(figsize=(10, 10))
    plt.imshow(maze, cmap='gray')  # 'binary' can also be used for black and white
    plt.axis('off')  # Turn off the axis labels
    plt.show()

# Set the size of the maze
size = 100
maze = generate_maze(size)
plot_maze(maze)
