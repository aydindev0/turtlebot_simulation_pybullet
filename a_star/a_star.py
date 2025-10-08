"""

AStar search

author: Ashwin Bose (@atb033)

edited by: Aydin @aydindev0 & Alex @alexl0110


"""

import sys
sys.path.insert(0, '../')
import argparse
import yaml
from math import fabs

class AStar():
    def __init__(self, env):
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.get_neighbors = env.get_neighbors

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def search(self, agent_name):
        """
        low level search 
        """
        initial_state = self.agent_dict[agent_name]["start"]
        step_cost = 1
        
        closed_set = set()
        open_set = {initial_state}

        came_from = {}

        g_score = {} 
        g_score[initial_state] = 0

        f_score = {} 

        f_score[initial_state] = self.admissible_heuristic(initial_state, agent_name)

        while open_set:
            temp_dict = {open_item:f_score.setdefault(open_item, float("inf")) for open_item in open_set}
            current = min(temp_dict, key=temp_dict.get)

            if self.is_at_goal(current, agent_name):
                return self.reconstruct_path(came_from, current)

            open_set -= {current}
            closed_set |= {current}

            neighbor_list = self.get_neighbors(current)

            for neighbor in neighbor_list:
                if neighbor in closed_set:
                    continue
                
                tentative_g_score = g_score.setdefault(current, float("inf")) + step_cost

                if neighbor not in open_set:
                    open_set |= {neighbor}
                elif tentative_g_score >= g_score.setdefault(neighbor, float("inf")):
                    continue

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + self.admissible_heuristic(neighbor, agent_name)
        return False

class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __str__(self):
        return str((self.x, self.y))

class State(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location
    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location.x) + str(self.location.y))
    def is_equal_except_time(self, state):
        return self.location == state.location
    def __str__(self):
        return str((self.time, self.location.x, self.location.y))


class Environment(object):
    def __init__(self, dimension, agents, obstacles):
        self.dimension = dimension
        self.obstacles = obstacles

        self.agents = agents
        self.agent_dict = {}

        self.make_agent_dict()

        self.a_star = AStar(self)

    def get_neighbors(self, state):
        neighbors = []

        # Wait
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)

        # Up
        n = State(state.time + 1, Location(state.location.x, state.location.y + 1))
        if self.state_valid(n):
            neighbors.append(n)

        # Down
        n = State(state.time + 1, Location(state.location.x, state.location.y - 1))
        if self.state_valid(n):
            neighbors.append(n)

        # Left
        n = State(state.time + 1, Location(state.location.x - 1, state.location.y))
        if self.state_valid(n):
            neighbors.append(n)

        # Right
        n = State(state.time + 1, Location(state.location.x + 1, state.location.y))
        if self.state_valid(n):
            neighbors.append(n)

        return neighbors



    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def state_valid(self, state):
        return state.location.x >= 0 and state.location.x < self.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.dimension[1] \
            and (state.location.x, state.location.y) not in self.obstacles

    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)


    def is_at_goal(self, state, agent_name):
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Location(agent['goal'][0], agent['goal'][1]))

            self.agent_dict.update({agent['name']:{'start':start_state, 'goal':goal_state}})

    def compute_solution(self, agent):
        solution = {}
        
        local_solution = self.a_star.search(agent)
        if not local_solution:
            return False
        solution.update({agent:local_solution})
        return solution

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution.values()])
def main(inputFile, outputFile):
    with open(inputFile, 'r') as param_file:
        param = yaml.load(param_file, Loader=yaml.FullLoader)

    dimension = param["map"]["dimensions"]
    obstacles = param["map"]["obstacles"]
    agents = param['agents']

    env = Environment(dimension, agents, obstacles)

    name = agents[0]['name']
    solution = env.compute_solution(name)
    if not solution:
        print(" Solution not found")
        return

    plan = {name: [{'t':s.time,'x':s.location.x,'y':s.location.y} for s in solution[name]]}
    output = {"schedule": plan, "cost": env.compute_solution_cost(solution)}
    with open(outputFile, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)
def run(dimensions, obstacles, agents, out_file):
    env = Environment(dimensions, agents, obstacles)
    name = agents[0]['name']
    solution = env.compute_solution(name)
    if not solution:
        print("Solution not found")
        return
    plan = {name: [{'t':s.time,'x':s.location.x,'y':s.location.y} for s in solution[name]]}
    output = {"schedule": plan, "cost": env.compute_solution_cost(solution)}
    with open(out_file, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("param", help="input file containing map and obstacles")
    parser.add_argument("output", help="output file with the schedule")
    args = parser.parse_args()
    main(args.param, args.output)
