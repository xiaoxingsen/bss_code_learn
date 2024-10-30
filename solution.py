import copy


class Solution:

    def __init__(self, van_loc: list, van_dis_left: list, van_load: list):
        self.routes = []           # 存储每辆车的路径
        self.costs = []            # 存储每辆车的路径成本
        self.instructs = []        # 存储路径上的操作指令
        self.usages = []           # 标记是否需要重新计算
        self.van_loc = van_loc     # 初始位置
        self.van_dis_left = van_dis_left  # 每辆车剩余可用距离
        self.van_load = van_load   # 每辆车的初始负载
        self.unvisited_sum = 0     # 总未访问站点的数量

    @property
    def total_cost(self):
        return sum(self.costs)+self.unvisited_sum if len(self.costs) > 0 else 0

    @property
    def num_of_route(self):
        return len(self.costs)

    # maybe useless
    def add_route(self, route):
        self.routes.append(list(route))
        # self.costs.append(cost)
        # self.instructs.append(tmp_instruct)

    # maybe useless
    def remove_route(self, route_ind):
        self.routes.pop(route_ind)
        self.costs.pop(route_ind)
        self.instructs.pop(route_ind)
        assert len(self.costs) == len(self.routes) == len(self.instructs)

    def replace_costs(self, new_costs):
        costs = list(new_costs)
        self.costs = costs

    def replace_instructs(self, new_instructs):
        instructs = copy.deepcopy(new_instructs)
        self.instructs = instructs

    def get_exist_stations(self):
        return [station for route in self.routes for station in route]
        # station_list = []
        # for route in self.routes:
        #     for station in route:
        #         station_list.append(station)
        # return station_list

    def replace_route(self, route_ind, route):
        # tmp_route = list(route)
        self.routes[route_ind] = list(route)
        # self.costs[route_ind] = cost
        # self.instructs[route_ind] = tmp_instruct
        assert len(self.costs) == len(self.routes) == len(self.instructs)

    def is_feasible(self, num_of_van) -> bool:
        """
        if num of routes equals num of vans, return True
        :param num_of_van:
        :return:
        """
        return len(self.routes) == num_of_van

