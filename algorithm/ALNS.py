import logging

import numpy as np
import random
import time
import copy
import math

import algorithm.ALNSParams as Params
from algorithm.ALNSParams import RE_START_T, RE_END_T

from solution import Solution
from algorithm.RouteCompute import RouteComputer
# from DestroyOperator.WorstDestroy import WorstDestroy
from algorithm.DestroyOperator import RandomDestroy
# from DestroyOperator.ShawDestroy import ShawDestroy
# from RepairOperator.GreedyRepair import GreedyRepair as GreedyRepair
from algorithm.RepairOperator import RandomRepair

# from RepairOperator.RegretRepair import RegretRepair as RegretRepair

random.seed(42)

logging.basicConfig(level=logging.WARNING,
                    format='%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s')


def updateDict(dic, destroy, repair, val):
    """
    update dict with value (using times and operator score)
    :param dic: the dict to be updated
    :param destroy: destroy operator
    :param repair: repair operator
    :param val: value
    :return: dict
    """
    dic[destroy] += val
    dic[repair] += val
    return dic


def createDict():
    # key = [RandomDestroy, ShawDestroy, WorstDestroy, GreedyRepair, RandomRepair, RegretRepair]
    key = [RandomDestroy, RandomRepair]
    value = np.zeros(6, int)
    dic = dict(zip(key, value))
    return dic


class Problem:
    # temp_init = 100
    # destroyList = [RandomDestroy, ShawDestroy, WorstDestroy]  # destroy算子列表
    destroyList = [RandomDestroy]
    # repairList = [GreedyRepair, RandomRepair, RegretRepair]  # repair算子列表
    repairList = [RandomRepair]

    def __init__(self, num_of_van: int, van_location: list, van_dis_left: list, van_load: list, c_s: int, c_v: int,
                 cur_t: int, t_p: int, t_f: int, t_roll: int, c_mat: np.ndarray, ei_s_arr: np.ndarray,
                 ei_c_arr: np.ndarray,
                 esd_arr: np.ndarray, x_s_arr: list, x_c_arr: list, alpha: float, plot: bool, mode: str,
                 time_limit: int):


        """

        :param num_of_van: number of relocation vans (RV)  重新分配车辆（RV）的数量。
        :param van_location: starting points of the RV   每辆车的起始位置
        :param van_dis_left: left time step of each RV  每辆车的剩余时间步数
        :param van_load: load of each van  每辆车的当前载重
        :param c_s: capacity of stations  站点的容量上限
        :param c_v: capacity of relocation van  车辆的最大载重
        :param cur_t: current time step (in 10 min)  当前时间步
        :param t_p: duration of planning horizon (in 10 min)  规划时间窗的持续时间
        :param t_f: duration of forecasting horizon (in 10 min)  预测时间窗的持续时间
        :param t_roll: duration of rolling step (in 10 min)  滚动步长
        :param c_mat: matrix of distance between stations (in time step), station id 0 represents the depot   存储所有站点之间的距离（以时间步为单位），0 表示仓库。这个矩阵用于计算车辆在站点之间的行驶时间，进而决定每条调度路径的可行性
        :param ei_s_arr: array of Expected Inventory for self  预计在各站点未来时间窗内的单车库存，用于判断是否需要对站点进行重新分配调度。
        :param ei_c_arr: array of Expected Inventory for competitor  预测竞争对手单车在站点的库存，用于在多方环境中对比调度策略的效果。
        :param esd_arr: array of Expected Satisfied Demand  预期满足需求量，用于预测在不同调度策略下，某个站点的需求满足情况
        :param x_s_arr: original number of x_s at planning point 在调度开始时的站点自有单车数量，作为调度基准
        :param x_c_arr: original number of x_c at planning point  在调度开始时的竞争对手单车数量，用于计算 multi 模式下的满足需求情况
        :param alpha: weight of relocation cost  权衡重新分配成本与需求满足情况的参数，用于控制调度的偏好（例如更关注成本或更关注需求满足）
        :param plot: whether to plot the result
        :param mode: 'multi' or 'single  multi 模式考虑竞争对手库存，single 模式则不考虑。
        :param time_limit: max run time for single opt (in seconds)  指定单次优化的最大运行时间，以秒为单位

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
            'x_s_arr': [30 for _ in range(c_mat.shape[0] - 1)],
            'x_c_arr': [60 for _ in range(c_mat.shape[0] - 1)],
            'alpha': 5.0,
            'mode': 'multi',
            'time_limit': 30,
                            }
        """
        assert len(van_location) == len(van_dis_left) == num_of_van

        # algorithm params
        self.params = Params.parameter()
        self.end_temp = self.params.init_temp * self.params.t  # end temperature
        self.operator_time = createDict()  # 记录每个算子选中次数
        self.operator_score = createDict()  # 记录每个算子的得分
        # self.destroyNr = int(self.params.drate * self.numberOfNode)  # destroy的点的数量
        # self.destroyList = [RandomDestroy, ShawDestroy, WorstDestroy]  # destroy算子列表
        # self.repairList = [GreedyRepair, RandomRepair, RegretRepair]  # repair算子列表
        self.weight_destroy = np.array([1 for _ in range(len(self.destroyList))], dtype=float)  # 每个destroy算子的权重
        self.weight_repair = np.array([1 for _ in range(len(self.repairList))], dtype=float)  # 每个repair算子的权重

        # instance params
        self.num_of_van = num_of_van
        self.num_of_station = c_mat.shape[0] - 1
        self.van_loc = van_location
        self.van_dis_left = van_dis_left
        self.van_load = van_load
        self.cap_station = c_s
        self.cap_van = c_v
        self.cur_t = cur_t
        self.t_plan = t_p
        self.t_fore = t_f
        self.t_roll = t_roll
        self.c_mat = c_mat
        self.ei_s_arr = ei_s_arr
        # self.ei_c_arr = ei_c_arr
        # self.esd_arr = esd_arr
        self.x_s_arr = x_s_arr
        self.x_c_arr = x_c_arr
        # self.alpha = alpha

        self.to_plot = plot
        self.print_log = plot
        self.mode = mode
        self.time_limit = time_limit

        # other params
        self.destroy_num = int(self.params.drate * self.num_of_station)  # destroy的点的数量
        self.customers = [i for i in range(1, self.num_of_station + 1)]

        # route computer
        self.route_com = RouteComputer(c_van=c_v, c_station=c_s, c_mat=self.c_mat, ei_s_arr=ei_s_arr, ei_c_arr=ei_c_arr,
                                       esd_arr=esd_arr, x_s_arr=x_s_arr, x_c_arr=x_c_arr, t_cur=cur_t, t_plan=t_p,
                                       t_fore=t_f, alpha=alpha, customers=self.customers, num_of_vans=num_of_van,
                                       mode=mode)

        # metrics
        self.run_time = None
        self.bestVal = None
        self.bestVal_iter = None
        self.currentVal_iter = None

        # get initial solution
        self.init_sol = self.get_initial_sol()

        # solution
        self.best_sol = None

        # iteration time
        self.iter_time = None

    def get_initial_sol(self):
        """
        formulate an initial solution
        :return:
        """
        init_sol = Solution(van_loc=self.van_loc, van_dis_left=self.van_dis_left, van_load=self.van_load)
        customers = [val for val in self.customers if val not in self.van_loc]
        # customers = list(self.customers)
        # greedy algorithm
        for van in range(self.num_of_van):
            loc = init_sol.van_loc[van]  # 站点位置
            dis_left = init_sol.van_dis_left[van]  # 到达时间
            # load = init_sol.van_loc[van]  # 初始负载
            route, last_node = [loc], loc  # 潜在路径
            while self.route_com.is_feasible_init_route(dis_left=dis_left, route=route):
                node_ind = np.argmin(self.c_mat[last_node][i] for i in customers)
                last_node = customers[node_ind]
                route.append(last_node)
                customers.pop(node_ind)
                # customers.remove(last_node)
            route.pop(-1)
            customers.append(last_node)
            # cost, instruct = self.route_com.compute_route(r=route, t_left=dis_left, init_l=load)
            # cost, instruct = self.route_com.get_sol_cost(r=route, t_left=dis_left, init_l=load)
            init_sol.add_route(route=route)

        assert init_sol.is_feasible(self.num_of_van)
        init_sol.usages = [False for _ in range(self.num_of_van)]
        self.route_com.compute_total_cost(solution=init_sol)
        logging.info('initial solution cost:{}'.format(init_sol.total_cost))

        return init_sol

    # def init_weight(self):
    #     self.weight_destroy = np.array([1 for _ in range(len(self.destroyList))], dtype=float)  # 每个destroy算子的权重
    #     self.weight_repair = np.array([1 for _ in range(len(self.repairList))], dtype=float)  # 每个repair算子的权重
    #     p_destroy = self.weight_destroy / sum(self.weight_destroy)
    #     p_repair = self.weight_repair / sum(self.weight_repair)
    #     return p_destroy, p_repair

    def updateWeight(self, operator, used: bool):
        """
        update operator's weight
        :param operator: the chosen operator
        :param used: equals True if the operator has been used
        :return: None
        """
        if operator in self.destroyList:
            ind = self.destroyList.index(operator)
            if used is False:
                self.weight_destroy[ind] = self.weight_destroy[ind] * (1 - self.params.r)
            else:
                self.weight_destroy[ind] = self.weight_destroy[ind] * (1 - self.params.r) + \
                                           self.params.r * self.operator_score[operator] / self.operator_time[operator]
        else:
            ind = self.repairList.index(operator)
            if used is False:
                self.weight_repair[ind] = self.weight_repair[ind] * (1 - self.params.r)
            else:
                self.weight_repair[ind] = self.weight_repair[ind] * (1 - self.params.r) + \
                                          self.params.r * self.operator_score[operator] / self.operator_time[operator]

    def run(self):
        """
        process of the problem object
        :return:
        """
        logging.debug('start running')
        global_sol = copy.deepcopy(self.init_sol)  # global best  # 当前已知的全局最佳解
        current_sol = copy.deepcopy(self.init_sol)  # 当前迭代的解
        bestVal_list = []
        # currentVal = []
        bestVal_iter = []
        currentVal_iter = []

        bestVal_list.append(global_sol.total_cost)
        # currentVal.append(current_sol.total_cost)
        bestVal, currentVal = global_sol.total_cost, current_sol.total_cost
        noImprove = 0  # number of iterations that not improve  计数多少次迭代没有找到更优解

        start = time.time()
        temp = self.params.init_temp  # initial temperature  模拟退火的初始温度
        iter_num = 0  # iteration times  当前的迭代次数
        time_1 = 0
        time_2 = 0
        time_3 = 0
        print('no repositioning cost:{}'.format(self.route_com.compute_no_repositioning_cost()))  #
        while iter_num < self.params.iter_time and time.time() - start < self.time_limit:

            # logging.info(f'{iter_num}')
            # weight list of destroy operators
            p_destroy = self.weight_destroy / sum(self.weight_destroy)
            # weight list of repair operators
            p_repair = self.weight_repair / sum(self.weight_repair)

            # 内层循环，更新算子得分，循环完成后更新一次算子权重
            for i in range(self.params.fre):
                start_1 = time.time()
                # 轮盘赌选择
                Destroy = np.random.choice(self.destroyList, p=p_destroy)
                Repair = np.random.choice(self.repairList, p=p_repair)
                exist_stations, removed_sol = Destroy.destroy(
                    s=current_sol, destroy_num=self.destroy_num, computer=self.route_com)
                # logging.debug('destroyed solution:{}'.format(removed_sol))
                tmp_sol = Repair.repair(s=removed_sol, exist_stations=exist_stations, computer=self.route_com)
                end_1 = time.time()
                # print(end_1-start_1)
                time_1 += end_1 - start_1  # operator operation time
                self.operator_time = updateDict(self.operator_time, Destroy, Repair, 1)  # update using time
                logging.debug('operator time:{}'.format(self.operator_time))

                start_2 = time.time()
                tmpVal = tmp_sol.total_cost
                # simulated annealing acceptance
                acc_p = math.exp((tmpVal - current_sol.total_cost) / temp) if temp > 0.05 else 0

                # better than global best
                if tmpVal > global_sol.total_cost:
                    global_sol = tmp_sol
                    current_sol = copy.deepcopy(tmp_sol)
                    bestVal_list.append(tmpVal)
                    bestVal, currentVal = tmp_sol.total_cost, tmp_sol.total_cost
                    self.operator_score = updateDict(self.operator_score, Destroy, Repair, self.params.theta1)
                    noImprove = 0

                # better than current sol
                elif tmpVal > current_sol.total_cost:
                    current_sol = tmp_sol
                    currentVal = tmp_sol.total_cost
                    self.operator_score = updateDict(self.operator_score, Destroy, Repair, self.params.theta2)

                # accord with the accept rule
                elif acc_p > random.random():
                    current_sol = copy.deepcopy(tmp_sol)
                    currentVal = tmp_sol.total_cost
                    self.operator_score = updateDict(self.operator_score, Destroy, Repair, self.params.theta3)
                    noImprove += 1

                # deposit
                else:
                    self.operator_score = updateDict(self.operator_score, Destroy, Repair, self.params.theta4)
                    noImprove += 1

                end_2 = time.time()
                time_2 += end_2 - start_2

                '''
                if noImprove >= 10:
                    p_destroy, p_repair = self.initialWeight()
                    currentSol = copy.deepcopy(globalSol)
                    noImprove = 0
                '''

            # 每完成一次内层循环，更新一次算子权重
            start_3 = time.time()
            for operator in self.operator_time:
                if self.operator_time[operator] == 0:
                    self.updateWeight(operator=operator, used=False)  # never used
                else:
                    self.updateWeight(operator=operator, used=True)  # used
            end_3 = time.time()
            time_3 += end_3 - start_3

            temp = temp * self.params.c
            iter_num += 1
            bestVal_iter.append(bestVal)
            currentVal_iter.append(currentVal)

            if iter_num % 20 == 0 and self.print_log:
                print('iter_num:', iter_num)
                print('bestVal:', bestVal)
                print('currentVal:', currentVal)
                print('temp:', temp)
                print('noImprove:', noImprove)
                print('time_1:', time_1)
                print('time_2:', time_2)
                print('time_3:', time_3)
                print('--------------------------------------')
            if iter_num % 100 == 0 and self.print_log:
                print('repositioning cost: {}'.format(sum(current_sol.costs)))
                print('no repositioning cost: {}'.format(current_sol.unvisited_sum))
                print('current routes: {}'.format(current_sol.routes))
                print('current costs: {}'.format(current_sol.costs))
                print('current instructs: {}'.format(current_sol.instructs))

        self.iter_time = iter_num

        end = time.time()
        self.run_time = end - start

        # update solution and metrics
        self.best_sol = global_sol
        self.bestVal_iter = bestVal_iter
        self.currentVal_iter = currentVal_iter

        # 输出运行时间和各算子使用次数
        print('time span:%.2f\n' % self.run_time)
        # print('最优值：', globalSol.totalCost)
        print('bestVal:', bestVal)
        # print('currentVal:', currentVal)
        # print('最优解：', globalSol)
        for key, value in self.operator_time.items():
            print('{}:{}'.format(key.__name__, value))
        # comparison
        print(f'best solution route: {self.best_sol.routes}')
        print(f'best solution instructions: {self.best_sol.instructs}')
        print('no repositioning cost:{}'.format(self.route_com.compute_no_repositioning_cost()))
        if self.to_plot:
            self.plot()

    def plot(self):
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 3))
        X = list(range(self.iter_time))
        plt.plot(X, self.bestVal_iter, label='best_value')
        plt.plot(X, self.currentVal_iter, label='current_value')
        plt.legend()
        plt.show()

    def get_result(self) -> dict:
        sol = self.best_sol

        # objective value
        result = {'objective': sol.total_cost}

        # location, instruct and distance left
        van_loc_list, van_n_list, van_exp_inv_list, van_target_inv_list = [], [], [], []
        van_dis_left_list, dest_list = [], []
        for van in range(len(sol.routes)):
            step_loc_list, step_n_list, step_exp_inv_list, step_target_inv_list, step, cumu_step, s_ind = \
                [0 for _ in range(self.t_plan)], [0 for _ in range(self.t_plan)], [0 for _ in range(self.t_plan)], \
                [0 for _ in range(self.t_plan)], 0, sol.van_dis_left[van], 0
            van_dis_flag = False
            while step < self.t_plan:
                if step == cumu_step:
                    step_loc_list[int(step)] = sol.routes[van][s_ind]
                    step_n_list[int(step)] = sol.instructs[van][s_ind]
                    if sol.routes[van][s_ind] > 0:
                        if self.mode == 'multi':
                            step_exp_inv_list[int(step)] = self.ei_s_arr[
                                sol.routes[van][s_ind] - 1,
                                round(self.cur_t - RE_START_T),
                                round(self.cur_t - RE_START_T + step),
                                round(self.x_s_arr[sol.routes[van][s_ind] - 1]),
                                round(self.x_c_arr[sol.routes[van][s_ind] - 1])
                            ]
                            step_target_inv_list[int(step)] = \
                                round(step_exp_inv_list[int(step)]) + sol.instructs[van][s_ind]
                        elif self.mode == 'single':
                            step_exp_inv_list[int(step)] = self.ei_s_arr[
                                sol.routes[van][s_ind] - 1,
                                round(self.cur_t - RE_START_T),
                                round(self.cur_t - RE_START_T + step),
                                round(self.x_s_arr[sol.routes[van][s_ind] - 1])
                            ]
                            step_target_inv_list[int(step)] = \
                                round(step_exp_inv_list[int(step)]) + sol.instructs[van][s_ind]
                        else:
                            assert False, 'mode error'
                    else:
                        step_exp_inv_list[int(step)] = 0
                        step_target_inv_list[int(step)] = 0
                    if s_ind < len(sol.routes[van]) - 1:
                        cumu_step += self.c_mat[sol.routes[van][s_ind], sol.routes[van][s_ind + 1]]
                    else:
                        cumu_step += self.t_plan
                    if cumu_step >= self.t_plan and van_dis_flag is False:
                        van_dis_flag = True
                        van_dis_left_list.append(cumu_step - self.t_plan - self.t_roll)
                        dest_list.append(sol.routes[van][s_ind])
                    else:
                        s_ind += 1
                    step += 1
                else:
                    step_loc_list[int(step)], step_n_list[int(step)], step_exp_inv_list[int(step)], step_target_inv_list[int(step)] = \
                        None, None, None, None
                    step += 1
            van_loc_list.append(copy.deepcopy(step_loc_list))
            van_n_list.append(copy.deepcopy(step_n_list))
            van_exp_inv_list.append(copy.deepcopy(step_exp_inv_list))
            van_target_inv_list.append(copy.deepcopy(step_target_inv_list))

        assert len(van_loc_list) == len(van_n_list) == len(sol.routes)
        result['start_time'] = self.cur_t  # hour * 6
        result['routes'] = sol.routes
        result['van_dis_left'] = van_dis_left_list
        result['destination'] = dest_list
        result['exp_inv'] = van_exp_inv_list
        result['exp_target_inv'] = van_target_inv_list
        result['loc'] = van_loc_list
        result['n_r'] = van_n_list

        return result
