from abc import ABCMeta, abstractmethod
from collections import defaultdict, deque

from state import State
import sys

class Heuristic(metaclass=ABCMeta):
    def __init__(self, initial_state: 'State'):
        # stored data
        self.mappings = {}
        self.point2goal = defaultdict(lambda: None)
        self.letter2goals = defaultdict(list)

        counter = defaultdict(int) # for ids
        # Find all goals
        for row in range(State.MAX_ROW):
            for col in range(State.MAX_COL):
                g = State.GOALS[row][col]
                if g is not None:
                    # assign unique id to each goal
                    gk = str(g)+str(counter[str(g)])
                    self.point2goal[(row,col)] = gk
                    self.letter2goals[g] += [gk]
                    counter[str(g)] += 1
                    # Make mapping for every goal
                    self.mappings[gk] = [[None for _ in range(State.MAX_COL+2)] for _ in range(State.MAX_ROW)]
                    # BFS to map distance goal to any tile 
                    queue = deque()
                    queue_set = set()
                    root = (0,row,col)
                    queue.append(root)
                    queue_set.add((row,col))
                    while len(queue) > 0:
                        d,x,y = queue.popleft()
                        self.mappings[gk][x][y] = d
                        for x_,y_ in [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]:
                            if x_ <= State.MAX_ROW and y_ <= State.MAX_COL and (x_,y_) not in queue_set and not State.WALLS[x_][y_]:
                                queue.append((d+1,x_,y_))
                                queue_set.add((x_,y_))

        #print(self.mappings.keys(), file=sys.stderr, flush=True)
        #for r in self.mappings['a0']:
        #    print(r, file=sys.stderr, flush=True)
        #print(self.point2goal.keys(), file=sys.stderr, flush=True)
        #print(self.point2goal[(2,3)], file=sys.stderr, flush=True)
        #print(self.letter2goals.keys(), file=sys.stderr, flush=True)
        #print(self.letter2goals['a'], file=sys.stderr, flush=True) 
    
    COUNT = 0

    def h(self, state: 'State') -> 'int':
        # All goals with boxes on them have h=0
        used_boxes = set()
        zero_goals = set()
        for row in range(State.MAX_ROW):
            for col in range(State.MAX_COL):
                b = state.boxes[row][col]
                if b:
                    if (row,col) in self.point2goal and self.point2goal[(row,col)] in self.letter2goals[str(b).lower()]:
                        used_boxes.add((row,col))
                        zero_goals.add(self.point2goal[(row,col)])
        
        # for each goal, distance to nearest box of correct type not already on another goal
        #minval_goal = defaultdict(lambda: int(sys.maxsize))
        minval_box = defaultdict(lambda: int(sys.maxsize))
        h_sum = 0
        for row in range(State.MAX_ROW):
            for col in range(State.MAX_COL):
                b = state.boxes[row][col]
                if b and (row,col) not in used_boxes:
                    for gk in self.letter2goals[str(b).lower()]:
                        if gk not in zero_goals:
                            h_sum += self.mappings[gk][row][col]*100
                            #minval_goal[gk] = min(minval_goal[gk], self.mappings[gk][row][col])
                            minval_box[(row,col)] = min(minval_box[(row,col)], self.mappings[gk][row][col]*10)

                    # agent distance
                    ax = state.agent_row
                    ay = state.agent_col
                    h_sum += abs(row - ax) + abs(col - ay)
                    
        h_sum += sum(minval_box.values())
        return h_sum
    
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

