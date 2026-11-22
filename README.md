# Sokoban solver — room by room, push-optimal

`main.ts` answers a set of generated Sokoban levels (XSB text in
`/input/<id>/level.txt`, move strings out to `/output/<id>/solution.txt`).

## Why it is fast

The levels this program is built for are not one open warehouse. Each is a
single hallway row with one to four small rooms hanging off it, and a box
can never leave the room it starts in — so a level's optimal push count is
exactly the sum of its rooms' optimal push counts, and the rooms can be
solved independently instead of searching the product of all box
positions.

The solver therefore works in three passes:

1. **Split.** Remove the hallway row (the row the player starts on) and
   take the connected components of what remains. A component holding as
   many boxes as goals is a room; its door squares are the ones touching
   the hallway.
2. **Solve each room** with a push-level A\*:
   - states are box placements plus the player's reachable region, so two
     player squares that reach the same places are one state;
   - the heuristic is an exact minimum-cost assignment of boxes to goals
     over push distances precomputed per goal (reverse BFS over the push
     graph, other boxes ignored — admissible, so the first solution found
     is push-optimal);
   - squares from which no goal is reachable at all are computed once and
     any push landing a box on one is pruned;
   - a room is accepted only with an end state that leaves its door
     squares free and the player able to walk back to a door, so rooms can
     be visited one after another (a room that cannot clear its door is
     solved last).
3. **Stitch and verify.** The player walks the hallway between rooms
   (walking is free — only pushes are scored), the room solutions are
   concatenated, and the whole move string is replayed on the original
   level before it is written out.

If a level does not split cleanly, the same A\* runs on the whole board
as a fallback.

## Determinism

The grader runs the program twice and compares the outputs byte for
byte, inside a 60-second wall for the whole set. This solver never reads
a clock: search budgets are counted in expanded nodes, all iteration
orders are fixed, and there is no randomness. On the practice set it
finishes in well under a second with every instance at the optimal push
count.

## Layout

- `main.ts` — the whole solver (parsing, splitting, A\*, stitching,
  verification, I/O).
- `LICENSE` — MIT.
