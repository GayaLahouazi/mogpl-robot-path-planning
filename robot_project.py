import sys
import time
import random
import heapq
from collections import deque

# =============================================================================
# 1. UTILITAIRES ET PHYSIQUE DU ROBOT
# =============================================================================

DIRS = ["nord", "est", "sud", "ouest"]
MOVE = {"nord": (-1, 0), "est": (0, 1), "sud": (1, 0), "ouest": (0, -1)}


def turn_left(o):
    """Renvoie l'orientation après une rotation à gauche."""
    idx = DIRS.index(o)
    return DIRS[(idx - 1) % 4]


def turn_right(o):
    """Renvoie l'orientation après une rotation à droite."""
    idx = DIRS.index(o)
    return DIRS[(idx + 1) % 4]


def check_position(i, j, grid):
    """
    Vérifie si la position (i, j) est valide pour le centre du robot.
    """
    M = len(grid)
    N = len(grid[0])

    # Interdiction des bords stricts
    if i <= 0 or j <= 0 or i >= M or j >= N:
        return False

    # Le robot occupe (i,j) et les 3 cases autour (-1, -1)
    # On vérifie les 4 cases adjacentes
    offsets = [(-1, -1), (-1, 0), (0, -1), (0, 0)]
    for di, dj in offsets:
        r, c = i + di, j + dj
        if 0 <= r < M and 0 <= c < N:
            if grid[r][c] == 1:
                return False
    return True


def check_rail(i, j, di, dj, grid):
    """
    Vérifie que le rail emprunté est libre.
    """
    M = len(grid)
    N = len(grid[0])

    # Coordonnées cibles
    ni, nj = i + di, j + dj

    cells_to_check = []
    if di == 0:  # Horizontal
        col = min(j, nj)
        cells_to_check = [(i - 1, col), (i, col)]
    else:  # Vertical
        row = min(i, ni)
        cells_to_check = [(row, j - 1), (row, j)]

    for r, c in cells_to_check:
        if 0 <= r < M and 0 <= c < N:
            if grid[r][c] == 1:
                return False
    return True


def reconstruct_path(parent, start, end_state):
    path = []
    curr = end_state
    start_state = (start[0], start[1], start[2])

    if curr not in parent and curr != start_state:
        return []

    while curr != start_state:
        prev_state, cmd = parent[curr]
        path.append(cmd)
        curr = prev_state
    return path[::-1]


# =============================================================================
# 2. ALGORITHMES (BFS & A*)
# =============================================================================


# --- BFS ---
def solve_BFS(grid, start, orientation, goal):
    """Parcours en largeur ."""
    M, N = len(grid), len(grid[0])
    sx, sy = start
    gx, gy = goal

    # Vérification départ/arrivée
    if not check_position(sx, sy, grid) or not check_position(gx, gy, grid):
        return -1, []

    start_state = (sx, sy, orientation)
    queue = deque([start_state])
    visited = {start_state}
    parent = {}
    dist = {start_state: 0}

    while queue:
        cx, cy, co = queue.popleft()

        if (cx, cy) == (gx, gy):
            return dist[(cx, cy, co)], reconstruct_path(
                parent, (sx, sy, orientation), (cx, cy, co)
            )

        # 1. Rotations
        transitions = []
        transitions.append((turn_left(co), "G"))
        transitions.append((turn_right(co), "D"))

        for next_ori, cmd in transitions:
            next_state = (cx, cy, next_ori)
            if next_state not in visited:
                visited.add(next_state)
                dist[next_state] = dist[(cx, cy, co)] + 1
                parent[next_state] = ((cx, cy, co), cmd)
                queue.append(next_state)

        # 2. Avancées
        dx, dy = MOVE[co]
        curr_i, curr_j = cx, cy

        for n in [1, 2, 3]:
            # Vérifie le rail pour ce pas
            if not check_rail(curr_i, curr_j, dx, dy, grid):
                break

            # Avance
            curr_i += dx
            curr_j += dy

            # Vérifie la position d'atterrissage
            if not check_position(curr_i, curr_j, grid):
                break

            # Ajoute l'état
            next_state = (curr_i, curr_j, co)
            if next_state not in visited:
                visited.add(next_state)
                dist[next_state] = dist[(cx, cy, co)] + 1
                parent[next_state] = ((cx, cy, co), f"a{n}")
                queue.append(next_state)

    return -1, []


# --- A* (A-STAR) ---
def heuristic(x, y, gx, gy):
    """Heuristique admissible : Manhattan / 3."""
    return (abs(x - gx) + abs(y - gy)) / 3.0


def solve_AStar(grid, start, orientation, goal):
    """A* avec file de priorité."""
    sx, sy = start
    gx, gy = goal

    if not check_position(sx, sy, grid) or not check_position(gx, gy, grid):
        return -1, []

    start_state = (sx, sy, orientation)

    pq = []
    heapq.heappush(pq, (heuristic(sx, sy, gx, gy), start_state))
    g_score = {start_state: 0}
    parent = {}

    while pq:
        f, current = heapq.heappop(pq)
        cx, cy, co = current

        if f > g_score.get(current, float("inf")) + heuristic(cx, cy, gx, gy):
            continue

        if (cx, cy) == (gx, gy):
            return g_score[current], reconstruct_path(
                parent, (sx, sy, orientation), current
            )

        neighbors = []

        # Rotations
        neighbors.append(((cx, cy, turn_left(co)), "G"))
        neighbors.append(((cx, cy, turn_right(co)), "D"))

        # Avancées
        dx, dy = MOVE[co]
        curr_i, curr_j = cx, cy

        for n in [1, 2, 3]:
            if not check_rail(curr_i, curr_j, dx, dy, grid):
                break
            curr_i += dx
            curr_j += dy
            if not check_position(curr_i, curr_j, grid):
                break
            neighbors.append(((curr_i, curr_j, co), f"a{n}"))

        for next_state, cmd in neighbors:
            new_g = g_score[current] + 1
            if new_g < g_score.get(next_state, float("inf")):
                g_score[next_state] = new_g
                nx, ny, _ = next_state
                new_f = new_g + heuristic(nx, ny, gx, gy)
                parent[next_state] = (current, cmd)
                heapq.heappush(pq, (new_f, next_state))

    return -1, []


# =============================================================================
# 3. GESTION FICHIERS
# =============================================================================


def read_instances(filename):
    instances = []
    try:
        with open(filename, "r") as f:
            lines = f.read().split()
        iterator = iter(lines)
        while True:
            try:
                M = int(next(iterator))
                N = int(next(iterator))
                if M == 0 and N == 0:
                    break
                grid = [[int(next(iterator)) for _ in range(N)] for _ in range(M)]
                d1, d2, f1, f2 = (
                    int(next(iterator)),
                    int(next(iterator)),
                    int(next(iterator)),
                    int(next(iterator)),
                )
                ori = next(iterator)
                instances.append((grid, (d1, d2), ori, (f1, f2)))
            except StopIteration:
                break
    except FileNotFoundError:
        print("Fichier introuvable.")
    return instances


def write_results(results, filename):
    with open(filename, "w") as f:
        for cost, cmds in results:
            if cost == -1:
                f.write("-1\n")
            else:
                f.write(f"{cost} {' '.join(cmds)}\n")


def run_solver(input_file, output_file, algo):
    instances = read_instances(input_file)
    results = []
    print(f"Résolution avec {algo}...")
    for grid, start, ori, goal in instances:
        if algo == "BFS":
            res = solve_BFS(grid, start, ori, goal)
        else:
            res = solve_AStar(grid, start, ori, goal)
        results.append(res)
    write_results(results, output_file)
    print("Fait.")


# =============================================================================
# 4. PARTIE GUROBI (Question E)
# =============================================================================


def generate_grid_lp(M, N, P):
    """Génère une grille via Programme Linéaire (Gurobi)."""
    try:
        import gurobipy as gp
        from gurobipy import GRB
    except ImportError:
        print("Erreur: Gurobi n'est pas installé.")
        return None

    model = gp.Model("grid_obstacles")
    model.setParam("OutputFlag", 0)  # Silence

    # Variables binaires x[i,j] = 1 si obstacle
    x = {}
    weights = [[random.randint(0, 1000) for _ in range(N)] for _ in range(M)]

    for i in range(M):
        for j in range(N):
            x[i, j] = model.addVar(vtype=GRB.BINARY)

    # Objectif : Minimiser poids des obstacles
    model.setObjective(
        gp.quicksum(weights[i][j] * x[i, j] for i in range(M) for j in range(N)),
        GRB.MINIMIZE,
    )

    # Contrainte 1: Nombre total d'obstacles
    model.addConstr(gp.quicksum(x[i, j] for i in range(M) for j in range(N)) == P)

    # Contrainte 2 & 3: Max obstacles par ligne/colonne
    for i in range(M):
        model.addConstr(gp.quicksum(x[i, j] for j in range(N)) <= 2 * P / M)
    for j in range(N):
        model.addConstr(gp.quicksum(x[i, j] for i in range(M)) <= 2 * P / N)

    # Contrainte 4: Pas de motif "101" (Case libre entre deux obstacles)
    # Lignes
    for i in range(M):
        for j in range(1, N - 1):
            model.addConstr(x[i, j - 1] + x[i, j + 1] - x[i, j] <= 1)
    # Colonnes
    for j in range(N):
        for i in range(1, M - 1):
            model.addConstr(x[i - 1, j] + x[i + 1, j] - x[i, j] <= 1)

    model.optimize()

    if model.status != GRB.OPTIMAL:
        print("Pas de solution optimale trouvée par Gurobi.")
        return None

    # Reconstruction de la grille
    grid = [[0] * N for _ in range(M)]
    for i in range(M):
        for j in range(N):
            grid[i][j] = int(round(x[i, j].X))
    return grid


def interactive_lp_mode():
    print("\n=== MODE GUROBI ===")
    try:
        M = int(input("M (lignes) : "))
        N = int(input("N (colonnes) : "))
        P = int(input("P (obstacles) : "))
    except ValueError:
        return

    grid = generate_grid_lp(M, N, P)
    if grid is None:
        return

    print("\nGrille Générée :")
    for row in grid:
        print(" ".join(str(c) for c in row))

    try:
        print("\nDépart:")
        d1 = int(input(" Ligne : "))
        d2 = int(input(" Colonne : "))
        print("Arrivée:")
        f1 = int(input(" Ligne : "))
        f2 = int(input(" Colonne : "))
        ori = input("Orientation (nord/sud/est/ouest) : ").strip().lower()
        if ori not in DIRS:
            ori = "nord"
    except ValueError:
        return

    print("\nRésolution avec A*...")
    cost, cmds = solve_AStar(grid, (d1, d2), ori, (f1, f2))

    if cost == -1:
        print("Pas de chemin trouvé.")
    else:
        print(f"Coût : {cost}")
        print(f"Commandes : {' '.join(cmds)}")


# =============================================================================
# 5. EXPERIMENTATIONS
# =============================================================================


def generate_simple_random(M, N, P):
    grid = [[0] * N for _ in range(M)]
    if P < M * N:
        for p in random.sample(range(M * N), P):
            grid[p // N][p % N] = 1
    return grid


def get_free_cell(grid):
    cells = [
        (i, j)
        for i in range(1, len(grid) - 1)
        for j in range(1, len(grid[0]) - 1)
        if grid[i][j] == 0
    ]
    return random.choice(cells) if cells else None


def benchmark_sizes():
    print("\n--- Benchmark Taille ---")
    for n in [10, 20, 30, 40, 50]:
        t_bfs, t_astar = 0, 0
        for _ in range(10):
            grid = generate_simple_random(n, n, n)
            s, g = get_free_cell(grid), get_free_cell(grid)
            o = random.choice(DIRS)
            if not s or not g:
                continue

            t0 = time.time()
            solve_BFS(grid, s, o, g)
            t_bfs += time.time() - t0
            t0 = time.time()
            solve_AStar(grid, s, o, g)
            t_astar += time.time() - t0
        print(f"{n}x{n} | BFS: {t_bfs/10:.5f}s | A*: {t_astar/10:.5f}s")


def benchmark_obstacles():
    print("\n--- Benchmark Obstacles (20x20) ---")
    for p in [10, 20, 30, 40, 50]:
        t_bfs, t_astar = 0, 0
        for _ in range(10):
            grid = generate_simple_random(20, 20, p)
            s, g = get_free_cell(grid), get_free_cell(grid)
            o = random.choice(DIRS)
            if not s or not g:
                continue

            t0 = time.time()
            solve_BFS(grid, s, o, g)
            t_bfs += time.time() - t0
            t0 = time.time()
            solve_AStar(grid, s, o, g)
            t_astar += time.time() - t0
        print(f"Obs: {p} | BFS: {t_bfs/10:.5f}s | A*: {t_astar/10:.5f}s")


# =============================================================================
# 6. MAIN
# =============================================================================


def main():
    while True:
        print("\n=== MENU PROJET ===")
        print("1. Résoudre fichier (A*)")
        print("2. Résoudre fichier (BFS)")
        print("3. Benchmark Taille")
        print("4. Benchmark Obstacles")
        print("5. Génération Gurobi + A*")
        print("0. Quitter")

        c = input("Choix : ")
        if c == "1":
            run_solver(input("In : "), input("Out : "), "ASTAR")
        elif c == "2":
            run_solver(input("In : "), input("Out : "), "BFS")
        elif c == "3":
            benchmark_sizes()
        elif c == "4":
            benchmark_obstacles()
        elif c == "5":
            interactive_lp_mode()
        elif c == "0":
            break
        else:
            print("Invalide")


if __name__ == "__main__":
    main()
