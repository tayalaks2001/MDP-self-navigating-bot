# Given input 20x20 matrix with obstacle positions and image location
# calculate order of vertices visited

class FastestPath:

    def __init__(self):
        self.min_cost = 10000 # cost to travserse the whole grid edge-by-edge
        self.path = []

    def tsp(self, distance, visited, curr, count_visited, n, cost, ans):
        if count_visited == n:
            print(ans)
            if cost < self.min_cost:
                self.min_cost = cost
                self.path = ans
                return
        for i in range(n):
            if visited[i] == False:
                if cost+distance[curr][i] > self.min_cost:
                    continue
                visited[i] = True
                new_ans = [0 for _ in range(n)]
                for k in range(count_visited):
                    new_ans[k] = ans[k]
                new_ans[count_visited] = i
                self.tsp(distance, visited, i, count_visited+1, n,
                         cost+distance[curr][i], new_ans)
                visited[i] = False


    def get_order_of_visit(self, dist, n):
        visited = [False for _ in range(n)]
        visited[0] = True
        ans = [0 for _ in range(n)]
        self.tsp(dist, visited, 0, 1, n, 0, ans)
        return self.path
