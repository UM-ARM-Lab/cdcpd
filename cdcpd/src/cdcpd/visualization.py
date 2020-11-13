import numpy as np
from matplotlib import cm
import glplotlib.glplot as glplt
from itertools import cycle
import time


def gaussian_kernel_any(X, Y, beta):
    N, _ = X.shape
    M, D = Y.shape
    norm_mat = np.empty((N, M))
    for i in range(M):
        diff = X - Y[i]
        norm_mat[:, i] = np.sum(diff * diff, axis=1)
    return np.exp(-norm_mat / (2 * beta))


class CPDCallback:
    def __init__(self, offset, template, grid=None, num_correspondence=5):
        self.offset = offset
        self.template_item = glplt.scatter_generic(pos=template+self.offset, color=(1, 0, 1, 1), size=0.005, pxMode=False)
        self.source_item = None
        self.target_item = None
        self.correspondence_item = None
        self.grid_item = None
        self.grid_verts = None
        self.grid_edges = None
        self.grid_kernel = None
        if grid is not None:
            self.grid_verts, self.grid_edges, beta = grid
            self.grid_kernel = gaussian_kernel_any(self.grid_verts, template, beta)
            self.grid_item = glplt.edge_set(self.grid_verts + self.offset, self.grid_edges, width=0.1, color=(1,1,1,1))
        self.num_correspondence = num_correspondence
        self.coord_item = glplt.grid_generic()
        glplt.show()

    def __del__(self):
        glplt.remove_item(self.source_item)
        glplt.remove_item(self.target_item)
        glplt.remove_item(self.template_item)
        glplt.remove_item(self.grid_item)
        glplt.remove_item(self.coord_item)
        glplt.remove_item(self.correspondence_item)

    def callback(self, iteration, error, X, Y, W=None, P=None):
        # self.offset = -X.mean(axis=0)
        if self.source_item is None:
            self.source_item = glplt.scatter_generic(pos=Y+self.offset, color=(1, 0, 0, 1), size=0.005, pxMode=False)
        else:
            self.source_item.setData(pos=Y+self.offset)

        if self.target_item is None:
            self.target_item = glplt.scatter_generic(X+self.offset, color=(0, 1, 1, 1), size=0.002, pxMode=False)
        else:
            self.target_item.setData(pos=X+self.offset)

        if W is not None and self.grid_kernel is not None:
            new_grid_verts = self.grid_verts + np.dot(self.grid_kernel, W) + self.offset
            glplt.update_edge_set(self.grid_item, new_grid_verts, self.grid_edges, width=0.1)

        if P is not None and self.num_correspondence > 0:
            # P = np.copy(P)
            # row_sums = P.max(axis=1)
            # P /= row_sums[:, np.newaxis]
            # p_mask = P > self.p_threshold
            k_largest = P.shape[1] - self.num_correspondence
            mask_idxs = np.argpartition(P, k_largest, axis=1)[:, k_largest:]
            p_mask = np.zeros(P.shape, dtype=np.bool)
            for i in range(mask_idxs.shape[0]):
                p_mask[i, mask_idxs[i]] = True
            idx_pairs = np.transpose(np.mgrid[0:P.shape[0], 0:P.shape[1]], axes=[1, 2, 0])
            edge_idxs = idx_pairs[p_mask]
            p_values = P[p_mask]
            cmap = cm.get_cmap('viridis')
            corr_colors = cmap(np.sqrt(p_values))
            corr_lines = np.empty((p_values.shape[0] * 2, Y.shape[-1]))
            corr_lines[0::2] = Y[edge_idxs[:, 0]]
            corr_lines[1::2] = X[edge_idxs[:, 1]]
            corr_line_colors = np.empty((corr_colors.shape[0] * 2, corr_colors.shape[1]))
            corr_line_colors[0::2] = corr_colors
            corr_line_colors[1::2] = corr_colors
            if self.correspondence_item is None:
                self.correspondence_item = glplt.line_generic(pos=corr_lines+self.offset, color=corr_line_colors, width=1.0,  mode='lines')
                self.correspondence_item.setGLOptions('opaque')
            else:
                self.correspondence_item.setData(pos=corr_lines+self.offset, color=corr_line_colors)

        time.sleep(1.0)


class CPDPlot:
    def __init__(self, width, height, offset=np.zeros(3), line_width=0.1, y_max=50):
        self.line_width = line_width
        self.y_max = y_max
        self.width = width
        self.height = height
        self.offset = offset
        self.rotate = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        self.cursor = None
        self.X = None
        self.Y = None

        self.plot_empty()

    def transform(self, line):
        return line @ self.rotate.T + self.offset

    def plot_empty(self):
        x_axis = np.zeros((2, 3), dtype=np.float32)
        y_axis = np.zeros((2, 3), dtype=np.float32)
        x_axis[1, 0] = self.width
        y_axis[1, 1] = self.height

        glplt.line_generic(self.transform(x_axis), color=(1,1,0,1), width=self.line_width)
        glplt.line_generic(self.transform(y_axis), color=(1,0,1,1), width=self.line_width)
        self.cursor = glplt.line_generic(self.transform(y_axis), color=(1,0,0,1), width=self.line_width)

    def plot(self, Y, X=None, highlight=None):
        if X is None:
            X = np.linspace(0, self.width, len(Y))
        self.X = np.array(X)
        self.Y = np.array(Y)
        line = np.zeros((self.Y.shape[0], 3), dtype=np.float32)
        line[:, 0] = self.X * (self.width / (self.X.max() - self.X.min()))
        line[:, 1] = self.Y * (self.height / self.y_max)
        color = (1,1,1,1)
        if highlight is not None:
            color = np.ones((self.Y.shape[0], 4), dtype=np.float32)
            for i in range(len(highlight)):
                if highlight[i]:
                    color[i] = np.array([1,1,0,1])

        glplt.line_generic(self.transform(line), color=color, width=self.line_width)

    def update_cursor(self, index):
        cursor_line = np.zeros((2, 3), dtype=np.float32)
        cursor_line[1, 1] = self.height
        cursor_line[:, 0] = float(index) / float(len(self.Y)) * self.width
        self.cursor.setData(pos=self.transform(cursor_line))


def generate_nd_grid(grid_dims=(10, 10, 10), subdiv=0):
    def valid_idx(nd_idx):
        for i in range(len(nd_idx)):
            if not (0 <= nd_idx[i] < grid_dims[i]):
                return False
        return True

    dim_vectors = [np.arange(0, x) for x in grid_dims]
    # points = np.array(np.meshgrid(*dim_vectors, sparse=False))\
    #     .reshape(len(grid_dims), -1, order='C').T.astype(np.float32)
    points = np.empty((np.prod(grid_dims), len(grid_dims)), dtype=np.float32)
    edges = []
    for nd_idx in np.ndindex(grid_dims):
        curr_idx = np.ravel_multi_index(nd_idx, dims=grid_dims, order='C')
        points[curr_idx] = nd_idx
        for i in range(len(grid_dims)):
            next_nd_idx = np.array(nd_idx)
            next_nd_idx[i] += 1
            if valid_idx(next_nd_idx):
                next_idx = np.ravel_multi_index(next_nd_idx, dims=grid_dims, order='C')
                edges.append((curr_idx, next_idx))

    if subdiv > 0:
        subdiv_points = []
        new_edges = []
        base_offset = points.shape[0]
        for edge in edges:
            point_pair = points[edge, :]
            new_points = np.array([np.linspace(start, stop, subdiv + 2, dtype=np.float32
                                               )[1:-1] for start, stop in point_pair.T]).T
            start_idx = base_offset + len(subdiv_points)
            end_idx = start_idx + new_points.shape[0] - 1
            # start and end edge to original points
            new_edges.append((edge[0], start_idx))
            new_edges.append((end_idx, edge[1]))
            # edge within new points
            for idx in range(start_idx, end_idx):
                new_edges.append((idx, idx + 1))
            subdiv_points.extend(new_points)
        points = np.vstack((points, np.array(subdiv_points)))
        edges = new_edges

    result_edges = np.array(edges, dtype=np.int64)
    return points, result_edges


def simple_visualize(input_arr,
                     tracking_result_history,
                     template_edges,
                     key_func,
                     filter_point_cloud=False,
                     target_fps=30):

    offset = -tracking_result_history[0].mean(axis=0)

    glplt.clear()
    glplt.show()
    glplt.grid_generic()
    tracking_result_edges_item = glplt.edge_set(
        tracking_result_history[0] + offset,
        template_edges,
        color=(1, 1, 1, 1), width=2.0)
    tracking_result_points_item = glplt.scatter_generic(
        tracking_result_history[0] + offset,
        color=(1, 0, 0, 1), size=0.01, pxMode=False)

    point_cloud_item = None
    desired_frame_time = 1 / target_fps
    last_time = time.time()
    for i in cycle(range(len(tracking_result_history))):
        print(i)
        point_cloud_img, color_img = input_arr[i]
        tracking_result = tracking_result_history[i]

        # generate mask
        mask_img = key_func(point_cloud_img, color_img)
        filtered_point_cloud = point_cloud_img
        if filter_point_cloud:
            filtered_point_cloud = point_cloud_img[mask_img]

        # update point cloud
        if point_cloud_item is None:
            point_cloud_item = glplt.point_cloud(
                filtered_point_cloud + offset,
                color_img,
                size=0.9, pxMode=True)
        else:
            glplt.update_point_cloud(
                point_cloud_item,
                filtered_point_cloud + offset,
                color_img)

        glplt.update_edge_set(
            tracking_result_edges_item,
            tracking_result + offset, template_edges)
        tracking_result_points_item.setData(
            pos=tracking_result + offset)

        curr_time = time.time()
        curr_frame_time = curr_time - last_time
        last_time = curr_time
        if curr_frame_time < desired_frame_time:
            time.sleep(desired_frame_time - curr_frame_time)
