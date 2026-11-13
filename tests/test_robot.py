import random
from pathlib import Path

import robot_project as rp

EXAMPLES = Path(__file__).resolve().parent.parent / "examples"


def replay(grid, start, orientation, commands):
    """Replay a command sequence with the project's own physical checks; return the final cell."""
    i, j = start
    for command in commands:
        if command == "G":
            orientation = rp.turn_left(orientation)
        elif command == "D":
            orientation = rp.turn_right(orientation)
        else:
            di, dj = rp.MOVE[orientation]
            for _ in range(int(command[1])):
                assert rp.check_rail(i, j, di, dj, grid)
                i, j = i + di, j + dj
                assert rp.check_position(i, j, grid)
    return i, j


def test_example_instance_matches_the_expected_output():
    grid, start, orientation, goal = rp.read_instances(str(EXAMPLES / "instance.txt"))[0]
    cost, commands = rp.solve_AStar(grid, start, orientation, goal)
    expected = (EXAMPLES / "expected_output.txt").read_text().split()
    assert [str(cost)] + commands == expected
    assert rp.solve_BFS(grid, start, orientation, goal)[0] == cost


def test_bfs_and_astar_agree_and_their_plans_are_feasible():
    random.seed(0)
    solvable = 0
    for _ in range(200):
        size = random.choice([10, 20, 30])
        grid = rp.generate_simple_random(size, size, size)
        start, goal = rp.get_free_cell(grid), rp.get_free_cell(grid)
        orientation = random.choice(rp.DIRS)
        bfs_cost, bfs_commands = rp.solve_BFS(grid, start, orientation, goal)
        astar_cost, astar_commands = rp.solve_AStar(grid, start, orientation, goal)
        assert bfs_cost == astar_cost
        if bfs_cost == -1:
            continue
        solvable += 1
        for commands in (bfs_commands, astar_commands):
            assert len(commands) == bfs_cost
            assert replay(grid, start, orientation, commands) == goal
    assert solvable > 50


def test_turns_are_inverse_of_each_other():
    for orientation in rp.DIRS:
        assert rp.turn_right(rp.turn_left(orientation)) == orientation
