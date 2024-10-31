"""
test cases
"""
import pickle
import numpy as np

np.random.seed(42)

# c_mat: matrix of distance between stations (in time step), station id 0 represents the depot
# 存储所有站点之间的距离（以时间步为单位），0 表示仓库。这个矩阵用于计算车辆在站点之间的行驶时间，进而决定每条调度路径的可行性
# c_mat
c_mat = np.random.randint(2, 5, size=[26, 26])  # discrete time step (in 10 min)
for i in range(c_mat.shape[0]):
    c_mat[i][i] = 0
    for j in range(i+1, c_mat.shape[0]):
        c_mat[j][i] = c_mat[i][j]

# ei_arr
# array[e, t_0, t, x_s, x_c]
# t_0 represents start time, t represents end time  x_s
# ei_s_arr = np.random.uniform(0, 50, size=(30, 1, 49, 51, 51))
# ei_c_arr = np.random.uniform(0, 50, size=(30, 1, 49, 51, 51))
# 第 1 维（30）：代表站点数量，例如，这里有 30 个站点。
# 第 2 维（1）：代表时间的起点时间段，通常在预测时间序列中，时间的起点被固定为一个单位时间段。
# 第 3 维（49）：代表时间序列中的时段数，表示从起点开始，往后延续的 49 个时间段。每个时间段可以是一个具体的时间单位（如10分钟、1小时等）。
# 第 4 维（51）：代表站点的自身库存状态，这里假设库存可以从 0 到 50，即共有 51 个不同的库存量。
# 第 5 维（51）：代表竞争对手的库存状态，同样假设库存范围为 0 到 50。
with open(r'D:\Desktop\Multi-platform EBSS operations\multi-platform-relocation\expectation_calculation\EI_s_array_multi.pkl', 'rb') as f:
    ei_s_arr = pickle.load(f)
with open(r'D:\Desktop\Multi-platform EBSS operations\multi-platform-relocation\expectation_calculation\EI_c_array_multi.pkl', 'rb') as f:
    ei_c_arr = pickle.load(f)

# eip_arr
# array[e, x_s, x_c, n]
# esd_arr = np.random.uniform(-10, 100, size=(30, 51, 51, 6))
# 第 1 维（30）：代表站点数量，即有 30 个站点。
# 第 2 维（51）：表示站点的自身库存状态范围，从 0 到 50，共有 51 种库存量。
# 第 3 维（51）：表示竞争对手的库存状态范围，同样从 0 到 50，共有 51 种库存量。
# 第 4 维（6）：代表时间段数或时间的某个滚动窗口，在这种情况下可能表示预测需求满足的未来 6 个时间段（如接下来的 6 小时、6 个 10 分钟等单位）
# eip_arr[:, :, :, 3] = 0
# mapping: {-30, -20, -10, 10, 20, 30}
with open(r'D:\Desktop\Multi-platform EBSS operations\multi-platform-relocation\expectation_calculation\ESD_array_multi.pkl', 'rb') as f:
    esd_arr = pickle.load(f)

case1 = {
    'num_of_van': 1,
    'van_location': [0],
    'van_dis_left': [0],
    'van_load': [0],
    'c_s': 40,
    'c_v': 50,
    'cur_t': 14 * 6,
    't_p': 2 * 6,
    't_f': 4 * 6,
    't_roll': 1 * 6,
    'c_mat': c_mat,
    'ei_s_arr': ei_s_arr,
    'ei_c_arr': ei_c_arr,
    'esd_arr': esd_arr,
    'x_s_arr': [30 for _ in range(c_mat.shape[0]-1)],
    'x_c_arr': [60 for _ in range(c_mat.shape[0]-1)],
    'alpha': 5.0,
    'mode': 'multi',
    'time_limit': 30,
}

# case2 = {
#     'num_of_van': 5,
#     'van_location': [0 for van in range(5)],
#     'van_dis_left': [0 for van in range(5)],
#     'van_load': [0 for van in range(5)],
#     'c_s': 50,
#     'c_v': 30,
#     't_p': 2 * 6,
#     't_f': 6 * 6,
#     't_roll': 1 * 6,
#     'c_mat': c_mat,
#     'ei_s_arr': ei_s_arr,
#     'ei_c_arr': ei_c_arr,
#     'eip_arr': esd_arr,
#     'x_s_arr': [10 for _ in range(c_mat.shape[0]-1)],
#     'x_c_arr': [10 for _ in range(c_mat.shape[0]-1)],
#     'alpha': 1.0,
# }
#
# case3 = {
#     'num_of_van': 5,
#     'van_location': [0 for van in range(5)],
#     'van_dis_left': [0 for van in range(5)],
#     'van_load': [0 for van in range(5)],
#     'c_s': 50,
#     'c_v': 30,
#     't_p': 2 * 6,
#     't_f': 6 * 6,
#     't_roll': 1 * 6,
#     'c_mat': c_mat,
#     'ei_s_arr': ei_s_arr,
#     'ei_c_arr': ei_c_arr,
#     'eip_arr': esd_arr,
#     'x_s_arr': [10 for _ in range(c_mat.shape[0]-1)],
#     'x_c_arr': [10 for _ in range(c_mat.shape[0]-1)],
#     'alpha': 1.0,
# }

