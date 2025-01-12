import numpy as np
from matplotlib import pyplot as plt
from itertools import permutations

n = 8 #2차원 평면에 존재하는 점의 개수 
x = np.random.uniform(0,100,n)
y = np.random.uniform(0,100,n)

points = np.array(list(zip(x,y)))
point0 = np.array([0,0])

def distance(x1, y1, x2, y2) :
    r = np.sqrt((x1-x2)**2 + (y1-y2)**2)
    return r

def distance_matrix(points) : #각 지점에서 다른 지점으로 갈떄의 거리를 cost라고 생각하면 matrix들에 cost가 저장되어있다.
    n = len(points)
    distance_matrix = np.zeros((n,n))
    for i in range(n) :
        for j in range(n) : #대칭행렬이라서 계산을 굳이 양쪽다 할 필요가 없을거 같은데??? -> 계산량을 줄이는 걸 생각해봐야 할 듯 
            distance_matrix[i, j] = distance(points[i][0],points[i][1], points[j][0],points[j][1])

    return distance_matrix 

def find_optimal_path(distance_matrix) :
    n = len(distance_matrix)
    all_cases = permutations(range(n)) #10개의 점들로 구현될 수 있는 모든 상황을 순열을 통해서 다 구현
    optimal_path = None #최단거리가 나오기 위한 루트
    optimal_path_sol = float('inf') #최단거리를 밑에 계산한 값과 비교해야해서 일단 큰 값으로 저장해둔다.

    for cases in all_cases : 
        path = cases
        path_distance = distance(point0[0],point0[1],points[cases[0]][0], points[cases[0]][1]) + distance(point0[0],point0[1],points[cases[-1]][0], points[cases[-1]][1])
        for i in range(len(cases)-1) : #마지막은 끊기니까 조심해야한다.
            path_distance += np.sum(distance_matrix[cases[i], cases[i+1]])

            if path_distance < optimal_path_sol :
                optimal_path_sol = path_distance
                optimal_path = path
    adjusted_path = [0] + [x + 1 for x in optimal_path] + [0]
    return adjusted_path, optimal_path_sol


matrix = distance_matrix(points)
sol = find_optimal_path(matrix)

optimal_path = sol[0] 

optimal_path_sol = sol[1]
x1 = []
y1 = []
for i in range(len(optimal_path)):
    if optimal_path[i] == 0 :
        x1.append(0)
        y1.append(0)
    else :
        x1.append(points[optimal_path[i]-1][0])
        y1.append(points[optimal_path[i]-1][1])  

print(f'최적경로의 순서 : {optimal_path}, 최적경로의 거리 : {optimal_path_sol}')

plt.scatter(point0[0], point0[1], c='red', s=100, label="Origin (0)") 
plt.scatter(x, y, c='blue', label="Points") 
plt.plot(x1,y1, label = 'Path')
plt.legend()
plt.title("Optimal Path Visualization")
plt.show()