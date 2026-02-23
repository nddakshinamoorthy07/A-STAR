<h1>ExpNo 4 : Implement A* search algorithm for a Graph</h1> 
<h3>Name: DAKSHINA MOORTHY N D</h3>
<h3>Register Number: 212224230049</h3>
<H3>Aim:</H3>
<p>To ImplementA * Search algorithm for a Graph using Python 3.</p>
<H3>Algorithm:</H3>

``````
// A* Search Algorithm
1.  Initialize the open list
2.  Initialize the closed list
    put the starting node on the open 
    list (you can leave its f at zero)

3.  while the open list is not empty
    a) find the node with the least f on 
       the open list, call it "q"

    b) pop q off the open list
  
    c) generate q's 8 successors and set their 
       parents to q
   
    d) for each successor
        i) if successor is the goal, stop search
        
        ii) else, compute both g and h for successor
          successor.g = q.g + distance between 
                              successor and q
          successor.h = distance from goal to 
          successor (This can be done using many 
          ways, we will discuss three heuristics- 
          Manhattan, Diagonal and Euclidean 
          Heuristics)
          
          successor.f = successor.g + successor.h

        iii) if a node with the same position as 
            successor is in the OPEN list which has a 
           lower f than successor, skip this successor

        iV) if a node with the same position as 
            successor  is in the CLOSED list which has
            a lower f than successor, skip this successor
            otherwise, add  the node to the open list
     end (for loop)
  
    e) push q on the closed list
    end (while loop)

``````

# PROGRAM

```

from collections import defaultdict

# Heuristic dictionary will be filled from input
H_dist = {}

# Get neighbors of a node
def get_neighbors(v, graph):
    if v in graph:
        return graph[v]
    return None

def heuristic(n):
    return H_dist[n]

def aStarAlgo(start_node, stop_node, graph):
    open_list = set([start_node])
    closed_list = set([])

    g = {}  # store distance from start
    g[start_node] = 0

    parents = {}  # store parents to reconstruct path
    parents[start_node] = start_node

    while len(open_list) > 0:

        # find node with lowest f = g + h
        n = None
        for v in open_list:
            if n is None or g[v] + heuristic(v) < g[n] + heuristic(n):
                n = v

        if n is None:
            print("Path does not exist!")
            return None

        # If goal reached, reconstruct path
        if n == stop_node:
            path = []
            while parents[n] != n:
                path.append(n)
                n = parents[n]
            path.append(start_node)
            path.reverse()
            print("Path found:", path)
            return path

        # For all neighbors of node n
        for (m, weight) in get_neighbors(n, graph):
            # g(n) + edge weight
            tentative_g = g[n] + weight

            if m not in open_list and m not in closed_list:
                open_list.add(m)
                parents[m] = n
                g[m] = tentative_g

            else:
                # If better path found
                if g[m] > tentative_g:
                    g[m] = tentative_g
                    parents[m] = n

                    if m in closed_list:
                        closed_list.remove(m)
                        open_list.add(m)

        open_list.remove(n)
        closed_list.add(n)

    print("Path does not exist!")
    return None



# -------------------------
# INPUT HANDLING
# -------------------------

graph = defaultdict(list)

n, e = map(int, input().split())
for i in range(e):
    u, v, cost = input().split()
    graph[u].append((v, int(cost)))
    graph[v].append((u, int(cost)))  # undirected graph

# read heuristics
for i in range(n):
    node, h = input().split()
    H_dist[node] = int(h)

start = input()
goal = input()

aStarAlgo(start, goal, graph)

```



<hr>
<h2>Sample Graph I</h2>
<hr>

![image](https://github.com/natsaravanan/19AI405FUNDAMENTALSOFARTIFICIALINTELLIGENCE/assets/87870499/b1377c3f-011a-4c0f-a843-516842ae056a)

<hr>
<h2>Sample Input</h2>
<hr>
10 14 <br>
A B 6 <br>
A F 3 <br>
B D 2 <br>
B C 3 <br>
C D 1 <br>
C E 5 <br>
D E 8 <br>
E I 5 <br>
E J 5 <br>
F G 1 <br>
G I 3 <br>
I J 3 <br>
F H 7 <br>
I H 2 <br>
A 10 <br>
B 8 <br>
C 5 <br>
D 7 <br>
E 3 <br>
F 6 <br>
G 5 <br>
H 3 <br>
I 1 <br>
J 0 <br>
<hr>
<h2>Sample Output</h2>
<hr>
Path found: ['A', 'F', 'G', 'I', 'J']


<hr>
<h2>Sample Graph II</h2>
<hr>

![image](https://github.com/natsaravanan/19AI405FUNDAMENTALSOFARTIFICIALINTELLIGENCE/assets/87870499/acbb09cb-ed39-48e5-a59b-2f8d61b978a3)


<hr>
<h2>Sample Input</h2>
<hr>
6 6 <br>
A B 2 <br>
B C 1 <br>
A E 3 <br>
B G 9 <br>
E D 6 <br>
D G 1 <br>
A 11 <br>
B 6 <br>
C 99 <br>
E 7 <br>
D 1 <br>
G 0 <br>
<hr>
<h2>Sample Output</h2>
<hr>
Path found: ['A', 'E', 'D', 'G']

# OUTPUT



<img width="581" height="387" alt="image" src="https://github.com/user-attachments/assets/98aec788-85b4-4a23-a04c-c561f9e04286" />


# RESULT 

Thus, A* algorithm was implemented and the path was found.
