from algorithm.ALNS import Problem
from tests.test_case import case1


def get_relocation_routes(num_of_van: int, van_location: list, van_dis_left: list, van_load: list,
                          c_s: int, c_v: int, cur_t: int, t_p: int, t_f: int, t_roll: int, c_mat, ei_s_arr,
                          ei_c_arr, esd_arr, x_s_arr, x_c_arr, alpha: float = 10.0, plot: bool = True,
                          mode: str = 'multi', time_limit: int = 300) -> tuple:
    """
    to calculate the relocation objective value and information.

    :param num_of_van: number of relocation vans (RV)
    :param van_location: starting points of the RV
    :param van_dis_left: left time step of each RV
    :param van_load: load of each van
    :param c_s: capacity of stations
    :param c_v: capacity of relocation van
    :param cur_t: current time step of the system (in 10 min, starting from 00:00)
    :param t_p: duration of planning horizon (in 10 min)
    :param t_f: duration of forecasting horizon (in 10 min)
    :param t_roll: duration of rolling step (in 10 min)
    :param c_mat: matrix of distance between stations (in time step), station id 0 represents the depot  存储所有站点之间的距离（以时间步为单位），0 表示仓库。这个矩阵用于计算车辆在站点之间的行驶时间，进而决定每条调度路径的可行性
    :param ei_s_arr: array of Expected Inventory (self)  预计在各站点未来时间窗内的单车库存，用于判断是否需要对站点进行重新分配调度
    :param ei_c_arr: array of Expected Inventory (competitor)  预测竞争对手单车在站点的库存，用于在多方环境中对比调度策略的效果。
    :param esd_arr: array of Expected Satisfied Demand  预期满足需求量，用于预测在不同调度策略下，某个站点的需求满足情况。
    :param x_s_arr: original number of x_s at planning point  在调度开始时的站点自有单车数量，作为调度基准。
    :param x_c_arr: original number of x_c at planning point  在调度开始时的竞争对手单车数量，用于计算 multi 模式下的满足需求情况
    :param alpha: weight of relocation cost  权衡重新分配成本与需求满足情况的参数，用于控制调度的偏好（例如更关注成本或更关注需求满足）
    :param plot: whether to plot the result   指定是否生成调度方案的图表
    :param mode: 'multi' or 'single'  multi 模式考虑竞争对手库存，single 模式则不考虑
    :param time_limit: max run time for single opt (in seconds)  指定单次优化的最大运行时间，以秒为单位
    :return: dict of info
    """
    problem = Problem(num_of_van=num_of_van, van_location=van_location, van_dis_left=van_dis_left, van_load=van_load,
                      c_s=c_s, c_v=c_v, cur_t=cur_t, t_p=t_p, t_f=t_f, t_roll=t_roll, c_mat=c_mat, ei_s_arr=ei_s_arr,
                      ei_c_arr=ei_c_arr, esd_arr=esd_arr, x_s_arr=x_s_arr, x_c_arr=x_c_arr, alpha=alpha, plot=plot,
                      mode=mode, time_limit=time_limit)
    # problem.print_log = True
    problem.run()
    problem.plot()
    result = problem.get_result()
    return result, problem, problem.best_sol


if __name__ == "__main__":
    test_dict, problem, best_solution = get_relocation_routes(**case1, plot=True)
