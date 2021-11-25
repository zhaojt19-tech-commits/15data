import numpy as np
from copy import deepcopy
from utils import *


class Node(object):  # Represents a node in a search tree
    def __init__(self, state, parent=None, action=None, path_cost=0, h = 0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost #路径代价
        self.h = h #到终点的曼哈顿距离
        self.f = self.h + self.path_cost #总的距离之和
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def child_node(self, problem, action):
        next_state = problem.move(self.state, action)
        next_node = Node(next_state, self, action,
                         problem.g(self.path_cost, self.state,
                                   action, next_state), problem.manhattan(next_state))
        return next_node

    def path(self):
        """
        Returns list of nodes from this node to the root node
        """
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    def __repr__(self):
        return "<Node {}(g={})>".format(self.state, self.path_cost)

    def __lt__(self, other):
        return self.path_cost < other.path_cost

    def __eq__(self, other):
        return self.state == other.state


class Problem(object):
    def __init__(self, init_state=None, goal_state=None):
        self.init_state = Node(init_state)
        self.goal_state = Node(goal_state)

    def actions(self, state):
        """
        Given the current state, return valid actions.
        :param state:
        :return: valid actions
        """
        pass

    def move(self, state, action):
        pass

    def is_goal(self, state):
        pass

    def g(self, cost, from_state, action, to_state):
        return cost + 1

    def solution(self, goal):
        """
        Returns actions from this node to the root node
        """
        if goal.state is None:
            return None
        return [node.action for node in goal.path()[1:]]

    def expand(self, node):  # Returns a list of child nodes
        return [node.child_node(self, action) for action in self.actions(node.state)]

    def manhattan(self, current_state):
        final_state = self.goal_state.state
        i = 1
        h = 0  # 到目标的总距离
        while i < 16:
            if i in current_state[0]:
                y_current = current_state[0].index(i)
                x_current = 0
            if i in current_state[1]:
                y_current = current_state[1].index(i)
                x_current = 1
            if i in current_state[2]:
                y_current = current_state[2].index(i)
                x_current = 2
            if i in current_state[3]:
                y_current = current_state[3].index(i)
                x_current = 3
            # print([x_current,y_current])
            if i in final_state[0]:
                y_final = final_state[0].index(i)
                x_final = 0
            if i in final_state[1]:
                y_final = final_state[1].index(i)
                x_final = 1
            if i in final_state[2]:
                y_final = final_state[2].index(i)
                x_final = 2
            if i in final_state[3]:
                y_final = final_state[3].index(i)
                x_final = 3

            h = h + abs(x_final - x_current) + abs(y_final - y_current)
            i = i + 1
        return h

class GridsProblem(Problem):
    def __init__(self,
                 n,
                 init_state=[[11, 9, 4, 15],
                             [1, 3, 0, 12],
                             [7, 5, 8, 6],
                             [13, 2, 10, 14]],
                 goal_state=[[1, 2, 3, 4], 
                             [5, 6, 7, 8], 
                             [9, 10, 11, 12], 
                             [13, 14, 15, 0]]):
        super().__init__(init_state, goal_state)
        self.n = n

    def is_valid(self, loc):
        if -1 < loc[0] < self.n and -1 < loc[1] < self.n:
            return True
        else:
            return False

    def actions(self, state):
        empty_row, empty_col = np.where(np.array(state) == 0)[0][0], np.where(np.array(state) == 0)[1][0]
        candidates = [[empty_row-1, empty_col], [empty_row+1, empty_col],
                      [empty_row, empty_col-1], [empty_row, empty_col+1]]
        valid_candidates = [item for item in candidates if self.is_valid(item)]
        return valid_candidates

    def move(self, state, action):
        empty_row, empty_col = np.where(np.array(state) == 0)[0][0], np.where(np.array(state) == 0)[1][0]
        new_state = deepcopy(state)
        new_state[empty_row][empty_col] = state[action[0]][action[1]]
        new_state[action[0]][action[1]] = 0
        return new_state

    def is_goal(self, state):
        return state == self.goal_state.state

    def g(self, cost, from_state, action, to_state):
        return cost + 1



def search_with_info(problem):
    print("有信息搜索。")
    closelist = set()  # 构造闭节点表（集合）
    firstnode = problem.init_state
    firstnode.h = problem.manhattan(firstnode.state)
    firstnode.f = firstnode.h + firstnode.path_cost#计算出总距离
    openlist = PriorityQueue(firstnode)  # 构造开节点表（优先级队列）

    while not openlist.empty():
        node = openlist.pop()

        print((node.state, node.path_cost, node.h, node.f))

        if problem.is_goal(node.state):
            return Node.path(node)

        closelist.add(tuple(node.state[0] + node.state[1] + node.state[2] + node.state[3]))

        for childnode in problem.expand(node):
            if (tuple(childnode.state[0] + childnode.state[1] + childnode.state[2] + childnode.state[
                3]) not in closelist) and (openlist.find(childnode) == None):
                openlist.push(childnode)
            exist = openlist.find(childnode)
            if exist != None:
                openlist.compare_and_replace(exist, childnode)
    return None


def search_without_info(problem):
    print("无信息搜索")
    openlist = Queue()  # 构造开节点表（队列）
    closelist = set()  # 构造闭节点表（集合）

    openlist.push(problem.init_state)
    while not openlist.empty():
        node = openlist.pop()

        closelist.add(tuple(node.state[0] + node.state[1] + node.state[2] + node.state[3]))

        if problem.is_goal(node.state):
            return Node.path(node)
        for childnode in problem.expand(node):
            if (tuple(childnode.state[0] + childnode.state[1] + childnode.state[2] + childnode.state[
                3]) not in closelist) and (openlist.find(childnode) == None):
                openlist.push(childnode)
                print(childnode)
    return None


if __name__ == "__main__":

    print("请分别使用有信息和无信息搜索方法求解15数码问题。")
    problem = GridsProblem(4)
    result = search_with_info(problem)
    print(result)
    result = search_without_info(problem)
    print(result)
