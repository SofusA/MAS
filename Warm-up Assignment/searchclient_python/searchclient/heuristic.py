from abc import ABCMeta, abstractmethod
from collections import defaultdict, deque

from state import State
import sys
import bisect

class Heuristic(metaclass=ABCMeta):
    def __init__(self, initial_state: 'State'):
        # stored data
        self.mappings = defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: defaultdict(lambda: int(sys.maxsize)))))
        self.point2goal = defaultdict(lambda: None)
        self.goal2point = {}
        self.letter2goals = defaultdict(list)

        counter = defaultdict(int) # for ids
        # Find all points in map
        for row in range(State.MAX_ROW):
            for col in range(State.MAX_COL):
                g = State.GOALS[row][col]
                if g is not None:
                    # assign unique id to each goal
                    gk = str(g)+str(counter[str(g)])
                    self.point2goal[(row,col)] = gk
                    self.goal2point[gk] = (row,col)
                    self.letter2goals[g] += [gk]
                    counter[str(g)] += 1

                if not State.WALLS[row][col]:
                    # BFS to map distance from tile
                    queue = deque()
                    queue_set = set()
                    root = (0,row,col)
                    queue.append(root)
                    queue_set.add((row,col))
                    while len(queue) > 0:
                        d,x,y = queue.popleft()
                        self.mappings[row][col][x][y] = d
                        for x_,y_ in [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]:
                            if 0 <= x_ <= State.MAX_ROW and 0 <= y_ <= State.MAX_COL and (x_,y_) not in queue_set and not State.WALLS[x_][y_]:
                                queue.append((d+1,x_,y_))
                                queue_set.add((x_,y_))

    def h(self, state: 'State') -> 'int':
        goalbox_dists = []
        for bx in state.boxes.keys():
            for by in state.boxes[bx].keys():
                b = state.boxes[bx][by]
                if b:
                    # map distance to every goal
                    for gk in self.letter2goals[str(b).lower()]:
                        gx,gy = self.goal2point[gk]
                        dist = self.mappings[bx][by][gx][gy]
                        goalbox_dists += [(dist,bx,by,gx,gy)]              

        goal_boxes_used = []
        boxes_used = []
        while goalbox_dists:
            #print(len(goalbox_dists), file=sys.stderr, flush=True)
            best = (int(sys.maxsize),-1,-1,-1,-1)
            for pairs in goalbox_dists:
                dist,bx,by,gx,gy = pairs
                if dist <= best[0]:
                    best = pairs
            goal_boxes_used += [best]
            boxes_used += [(best[1],best[2])]
            goalbox_dists = [pairs for pairs in goalbox_dists if not ((pairs[1], pairs[2]) == (best[1], best[2]) or (pairs[3], pairs[4]) == (best[3], best[4]))]
        goal_sum = sum(pairs[0]**2 for pairs in goal_boxes_used)

        agent_sum = int(sys.maxsize)
        ax,ay = state.agent_row, state.agent_col
        for bx,by in boxes_used:
            # agent distance to box not on goal
            agent_sum = min(agent_sum, self.mappings[ax][ay][bx][by])

        return goal_sum + agent_sum

    @abstractmethod
    def f(self, state: 'State') -> 'int': pass
    
    @abstractmethod
    def __repr__(self): raise NotImplementedError


class AStar(Heuristic):
    def __init__(self, initial_state: 'State'):
        super().__init__(initial_state)
    
    def f(self, state: 'State') -> 'int':
        return state.g + self.h(state)
    
    def __repr__(self):
        return 'A* evaluation'


class WAStar(Heuristic):
    def __init__(self, initial_state: 'State', w: 'int'):
        super().__init__(initial_state)
        self.w = w
    
    def f(self, state: 'State') -> 'int':
        return state.g + self.w * self.h(state)
    
    def __repr__(self):
        return 'WA* ({}) evaluation'.format(self.w)


class Greedy(Heuristic):
    def __init__(self, initial_state: 'State'):
        super().__init__(initial_state)
    
    def f(self, state: 'State') -> 'int':
        return self.h(state)
    
    def __repr__(self):
        return 'Greedy evaluation'

