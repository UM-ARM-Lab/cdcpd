import numpy as np


def build_line(length=1.0, num_nodes=50):
    verts = np.zeros((num_nodes, 3), dtype=np.float32)
    verts[:, 0] = np.linspace(0, length, num_nodes)
    edges = np.empty((num_nodes - 1, 2), dtype=np.uint32)
    edges[:, 0] = range(0, num_nodes - 1)
    edges[:, 1] = range(1, num_nodes)
    return verts, edges


def build_rectangle(width=0.45, height=0.32, width_num_node=23, height_num_node=17):
    """
    Row major, row corresponds to width
    """
    #width_num_node = int(np.round(width / grid_size)) + 1
    #height_num_node = int(np.round(height / grid_size)) + 1

    print("Creating a rectangular grid with the following parameters:")
    print("Width:", width)
    print("Height:", height)
    print("W nodes::", width_num_node)
    print("H nodes:", height_num_node)

    def xy_to_index(x_idx, y_idx):
        # Assumes the following layout in imagespace - 0 is to the top left of the image
        #
        #        0          cloth_x_size+0    ...  cloth_y_size*cloth_x_size - cloth_x_size + 0
        #        1          cloth_x_size+1    ...  cloth_y_size*cloth_x_size - cloth_x_size + 1
        #        2          cloth_x_size+2    ...  cloth_y_size*cloth_x_size - cloth_x_size + 2
        #       ...
        #  cloth_x_size-1   cloth_x_size*2-1  ...  cloth_y_size*cloth_x_size - 1
        # return x_idx * width_num_node + y_idx
        return y_idx * height_num_node + x_idx

    verts = np.zeros((width_num_node * height_num_node, 3), dtype=np.float32)
    edges_temp = []

    for x in range(height_num_node):
        for y in range(width_num_node):
            curr_idx = xy_to_index(x, y)
            verts[curr_idx, 0] = x * height / (height_num_node - 1)
            verts[curr_idx, 1] = y * width / (width_num_node - 1)

            if x + 1 < height_num_node:
                edges_temp.append([curr_idx, xy_to_index(x + 1, y)])
            if y + 1 < width_num_node:
                edges_temp.append([curr_idx, xy_to_index(x, y + 1)])
    edges = np.array(edges_temp, dtype=np.uint32)
    return verts, edges
