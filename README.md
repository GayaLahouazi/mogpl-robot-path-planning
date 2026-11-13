# La balade du robot: minimum-time path planning for a warehouse robot

Course project for **MOGPL** (Modélisation, Optimisation, Graphes, Programmation Linéaire), M1 AI2D, Sorbonne Université, 2025–2026.

**Authors:** Gaya Lahouazi and Riadh Ouazib. Supervisor: Thomas Bellitto.

The full report (in French) is in [`report/Rapport_LAHOUAZI_OUAZIB.pdf`](report/Rapport_LAHOUAZI_OUAZIB.pdf).

## The problem

A circular robot (diameter 1.6 m) moves along the rails of a rectangular grid in a warehouse, between 1 m × 1 m obstacles. It always faces north, south, east or west and understands two commands, each lasting one second:

- `Avance(n)`, with n in {1, 2, 3}: move n metres in the direction it faces;
- `Tourne(gauche | droite)`: turn 90° on the spot.

Given a start point, a start orientation and a destination, find the **shortest sequence of commands**, or report that the destination cannot be reached.

## What we did

- **Modelling.** The problem becomes a shortest path in a directed graph whose vertices are the states (row, column, orientation), at most 4·M·N of them, with one arc of cost 1 per valid command.
- **Robot width.** Because the robot is wider than a cell, an obstacle next to the rail blocks it. Each move is checked metre by metre: the rail segment must be clear and the 2 × 2 block of cells around the robot's centre must be free.
- **Two solvers.** Breadth-first search (optimal because all costs are 1) and **A\*** with the admissible heuristic Manhattan distance / 3 (the robot covers at most 3 metres per second).
- **Experiments.** Running time against grid size (10 × 10 to 50 × 50) and against the number of obstacles (10 to 50 on a 20 × 20 grid), 10 random instances each.
- **Instance generation by integer linear programming** (Gurobi): place P obstacles at minimum random weight, with at most 2P/M per row, 2P/N per column, and no "obstacle, free, obstacle" pattern in a row or column.

## Results

Mean running time over 10 random instances (from the report):

| Grid | BFS | A\* |
|---|---|---|
| 10 × 10 | 0.83 ms | 0.78 ms |
| 20 × 20 | 3.31 ms | 1.23 ms |
| 30 × 30 | 6.40 ms | 5.64 ms |
| 40 × 40 | 15.17 ms | 1.19 ms |
| 50 × 50 | 30.23 ms | 10.86 ms |

A\* is faster on every size and the gap grows with the grid, but not uniformly: with only 10 instances per size the timings are noisy. When the number of obstacles varies on a 20 × 20 grid, neither algorithm wins consistently (see the report, section 5): on small, heavily constrained grids the heuristic brings little.

## Run it

Pure Python, no dependency for the solvers:

```bash
python robot_project.py
```

The menu offers: solve an instance file with A\* or BFS, the two benchmarks, and the Gurobi generator followed by A\* (this last option needs `gurobipy` and a Gurobi licence).

Example: [`examples/instance.txt`](examples/instance.txt) is a 9 × 10 instance; its optimal plan takes 12 commands ([`examples/expected_output.txt`](examples/expected_output.txt)):

```
12 D a1 D a3 a3 D a3 a1 D a1 G a2
```

### Instance file format

Each instance is a line `M N`, then M lines of N values (1 = obstacle, 0 = free), then a line `D1 D2 F1 F2 orientation` (start, destination, and `nord`, `sud`, `est` or `ouest`). The file ends with `0 0`. The output has one line per instance: the number of commands followed by the commands (`G`, `D`, `a1`, `a2`, `a3`), or `-1` if there is no path.

## Tests

```bash
pip install pytest
```

```bash
pytest
```

The tests check the example against its expected output, that BFS and A\* return the same cost on 200 random instances, and that every plan they return is physically feasible and ends on the destination.

## Files

| Path | Content |
|---|---|
| `robot_project.py` | Robot physics, BFS, A\*, file handling, benchmarks, Gurobi model |
| `examples/` | One instance and its expected output |
| `report/` | Project report (French) |
| `tests/` | Automated tests |
