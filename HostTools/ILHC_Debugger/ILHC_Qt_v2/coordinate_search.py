"""Ordered finite-graph skeleton search; controller safety remains in replay.

The reverse shortest distances include incoming direction and bend cost. They
are a lower bound for loopless routes; no prefix quota or unsafe state merging.
"""
import heapq
import itertools
import math

from navigation_planner import EPS

TURN_COST_MM = 120.0


def bend_cost(a, b, c):
    if a is None:
        return 0.0
    u = b[0]-a[0], b[1]-a[1]
    v = c[0]-b[0], c[1]-b[1]
    if abs(u[0]*v[1]-u[1]*v[0]) > EPS:
        return TURN_COST_MM
    return math.inf if u[0]*v[0]+u[1]*v[1] < 0 else 0.0


def route_cost(route):
    return sum(math.dist(a, b) for a, b in zip(route, route[1:])) + sum(
        bend_cost(a, b, c) for a, b, c in zip(route, route[1:], route[2:]))


def reverse_bounds(graph, goal, cancelled=lambda: False):
    """Exact distance-to-go on directed edge states, relaxing visited vertices."""
    incoming = {}
    for a, targets in graph.items():
        for b in targets:
            incoming.setdefault(b, []).append(a)
    bounds, frontier, counter = {}, [], itertools.count()
    for previous in incoming.get(goal, ()):
        state = previous, goal
        bounds[state] = 0.0
        heapq.heappush(frontier, (0.0, next(counter), state))
    expanded = 0
    while frontier:
        cost, _, (a, b) = heapq.heappop(frontier)
        if cost > bounds[a, b]+EPS:
            continue
        expanded += 1
        if expanded % 128 == 0 and cancelled():
            raise ValueError('关键坐标规划已取消')
        for previous in incoming.get(a, ()):
            if a == goal:
                continue  # goal is absorbing
            total = cost+math.dist(a, b)+bend_cost(previous, a, b)
            state = previous, a
            if total >= bounds.get(state, math.inf)-EPS:
                continue
            bounds[state] = total
            heapq.heappush(frontier, (total, next(counter), state))
    return bounds


class RankedRoutes:
    """A* enumeration of unique compressed loopless skeletons in cost order.

Every extension uses the exact relaxed cost-to-go. Collinear subpoints are
collapsed only if their direct edge exists. Paths retain their visited vertices
until that collapse, preventing a later edge from silently introducing a loop.
    """
    def __init__(self, graph, start, goal, cancelled=lambda: False, max_expanded=12000):
        self.graph = {tuple(a): tuple(sorted(set(map(tuple, targets)))) for a, targets in graph.items()}
        self.start, self.goal = tuple(start), tuple(goal)
        self.cancelled, self.max_expanded = cancelled, max_expanded
        self.bounds = reverse_bounds(self.graph, self.goal, cancelled)
        self.counter, self.frontier = itertools.count(), []
        self.expanded, self.yielded, self.limit_hit = 0, 0, False
        self.seen = {(self.start,)}
        lower = 0.0 if self.start == self.goal else min((
            math.dist(self.start, b)+self.bounds.get((self.start, b), math.inf)
            for b in self.graph.get(self.start, ())), default=math.inf)
        self.initial_lower_bound = lower
        if math.isfinite(lower):
            heapq.heappush(self.frontier, (lower, next(self.counter), 0.0, (self.start,)))

    @property
    def remaining_lower_bound(self):
        return self.frontier[0][0] if self.frontier else math.inf

    def next(self, upper_bound=math.inf):
        while self.frontier and self.remaining_lower_bound <= upper_bound+EPS:
            if self.cancelled():
                raise ValueError('关键坐标规划已取消')
            if self.expanded >= self.max_expanded:
                self.limit_hit = True
                return None
            _lower, _order, cost, path = heapq.heappop(self.frontier)
            self.expanded += 1
            if path[-1] == self.goal:
                self.yielded += 1
                return cost, list(path if len(path)>1 else path+(self.goal,))
            for target in self.graph.get(path[-1], ()):
                if target in path:
                    continue
                penalty = bend_cost(path[-2] if len(path)>1 else None, path[-1], target)
                if not math.isfinite(penalty):
                    continue
                trial = path+(target,)
                # Equivalent collinear routes share one skeleton, but an absent
                # direct edge must never be invented by simplification.
                if len(path)>1 and penalty == 0 and target in self.graph.get(path[-2], ()):
                    trial = path[:-1]+(target,)
                if trial in self.seen:
                    continue
                new = cost+math.dist(path[-1], target)+penalty
                lower = new+self.bounds.get((trial[-2], target), math.inf)
                if lower > upper_bound+EPS:
                    continue
                self.seen.add(trial)
                heapq.heappush(self.frontier, (lower, next(self.counter), new, trial))
        return None
