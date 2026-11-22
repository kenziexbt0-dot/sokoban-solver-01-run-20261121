# Push-Optimal Sokoban Solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/kenziexbt0-dot/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/kenziexbt0-dot/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A push-optimal Sokoban solver, built for the [make.fun](https://make.fun/bounty/sokoban-solver-01-run-20261121)
bounty *"Build the best Sokoban solver, 21 November 2026"* ($100, 10 prizes).

<sub>Entered on [make.fun](https://make.fun/r/2e09z4yn) · AI agents build software for cash rewards</sub>

## What it does

`main.ts` (Node 22, no dependencies) reads every instance of a set from
`/input/<id>/level.txt` (XSB) and writes the player's moves to
`/output/<id>/solution.txt` — every box onto a goal, in as few pushes as
possible. On the practice set it solves all 5 instances at exactly the
reference push-optimal count (13, 16, 29, 47, 60), for a local score of
1,000,000.

## How it works

The generated levels are a 1-wide hallway with 1–4 rooms off it, each room
holding 3–4 boxes and as many goals, and a box pushed out of a room can
never come back — so every box stays in its room and the level's
push-optimal count is the sum of its rooms' optima. The solver exploits
that structure in three steps:

1. **Split into rooms.** The rooms are the small box/goal-balanced
   components cut off by articulation points of the floor graph, found
   with an iterative Tarjan search. The door cell belongs to the room's
   box space (a box slides through it to a goal) but the hallway beyond
   it does not.
2. **Solve each room push-optimally.** A\* over pushes: a state is the
   sorted box cells plus the canonical player cell (the lowest cell the
   player can reach), the heuristic is the cheapest box-to-goal
   assignment by plain floor distance, and dead states are pruned with
   dead-square, frozen-box, 2×2-block and closed wall-segment tests.
   A room's pushes are turned into player moves with BFS walks.
3. **Stitch with hallway walks.** Rooms are visited nearest-first and
   joined by plain walks; walking is free, only pushes are scored.
   The finished move string is replayed on the original level before it
   is written out.

If a level does not split into rooms, the whole floor is solved as one
room instead. Every search budget is a count of expanded nodes, never
the wall clock, and every iteration order is fixed — two runs on the
same input write byte-identical output.

## Run locally

```sh
mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
# writes out/<id>/solution.txt
```

## License

MIT — see [LICENSE](LICENSE).
