// Sokoban solver built around the structure of the generated levels.
//
// A generated level is a hallway row with one to four independent rooms
// hanging off it.  Boxes never need to leave their room, and a level's
// optimal push count is the sum of its rooms' optima, so this solver:
//
//   1. splits the floor into the hallway row and the components off it,
//   2. solves each room on its own with a push-level A* whose heuristic is
//      an exact box-to-goal assignment over precomputed push distances,
//      with statically dead squares pruned away,
//   3. stitches the room solutions together with plain walking moves and
//      replays the full move string on the original level to verify it.
//
// If a level does not split cleanly, the same A* machinery runs on the
// whole board as a fallback.  All budgets are counted in expanded nodes,
// never in time, so two runs of this program produce identical output.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const IN_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Direction order is fixed everywhere: up, down, left, right.
const DR = [-1, 1, 0, 0];
const DC = [0, 0, -1, 1];
const WALK_CH = ['u', 'd', 'l', 'r'];
const PUSH_CH = ['U', 'D', 'L', 'R'];
const INF = 1e9;

function parseLevel(text) {
  const lines = text.replace(/\r\n?/g, '\n').split('\n');
  while (lines.length && lines[lines.length - 1] === '') lines.pop();
  const h = lines.length;
  let w = 0;
  for (const ln of lines) w = Math.max(w, ln.length);
  const wall = new Uint8Array(w * h).fill(1);
  const goal = new Uint8Array(w * h);
  const boxes = [];
  let player = -1;
  for (let r = 0; r < h; r++) {
    const ln = lines[r];
    for (let c = 0; c < ln.length; c++) {
      const ch = ln[c];
      const i = r * w + c;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goal[i] = 1;
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goal, boxes, player };
}

// Min pushes to bring one box from every cell to one fixed goal, ignoring
// the other boxes: BFS over the reversed push graph.  A box may sit only
// on boxCells; the player square behind a push must be a playerCell.
function pushDistances(w, h, boxCells, playerCells, goalCell) {
  const dist = new Int32Array(w * h).fill(-1);
  dist[goalCell] = 0;
  const queue = [goalCell];
  for (let q = 0; q < queue.length; q++) {
    const y = queue[q];
    const yr = (y / w) | 0, yc = y % w;
    for (let d = 0; d < 4; d++) {
      // Forward push moved a box from x into y along d, so x = y - d and
      // the player stood at x - d.
      const xr = yr - DR[d], xc = yc - DC[d];
      if (xr < 0 || xc < 0 || xr >= h || xc >= w) continue;
      const x = xr * w + xc;
      if (dist[x] !== -1 || !boxCells[x]) continue;
      const srow = xr - DR[d], scol = xc - DC[d];
      if (srow < 0 || scol < 0 || srow >= h || scol >= w) continue;
      if (!playerCells[srow * w + scol]) continue;
      dist[x] = dist[y] + 1;
      queue.push(x);
    }
  }
  return dist;
}

// Exact min-cost assignment of boxes to goals for small counts (DP over
// goal subsets); falls back to the sum of per-box minima for large ones.
function assignmentCost(cost) {
  const n = cost.length;
  if (n === 0) return 0;
  const m = cost[0].length;
  if (n > 10 || m > 12) {
    let total = 0;
    for (let i = 0; i < n; i++) {
      let best = INF;
      for (let j = 0; j < m; j++) best = Math.min(best, cost[i][j]);
      if (best >= INF) return INF;
      total += best;
    }
    return total;
  }
  const size = 1 << m;
  let dp = new Float64Array(size).fill(INF);
  dp[0] = 0;
  for (let i = 0; i < n; i++) {
    const next = new Float64Array(size).fill(INF);
    for (let mask = 0; mask < size; mask++) {
      const cur = dp[mask];
      if (cur >= INF) continue;
      for (let j = 0; j < m; j++) {
        if (mask & (1 << j)) continue;
        const c = cost[i][j];
        if (c >= INF) continue;
        const nm = mask | (1 << j);
        if (cur + c < next[nm]) next[nm] = cur + c;
      }
    }
    dp = next;
  }
  let best = INF;
  for (let mask = 0; mask < size; mask++) best = Math.min(best, dp[mask]);
  return best;
}

// Flood fill of the squares the player can walk to.  Returns the visited
// flags plus a parent array for rebuilding a walk path to any square.
function flood(w, h, playerCells, boxSet, start) {
  const seen = new Uint8Array(w * h);
  const parent = new Int32Array(w * h).fill(-1);
  const parentDir = new Int8Array(w * h).fill(-1);
  const queue = [start];
  seen[start] = 1;
  for (let q = 0; q < queue.length; q++) {
    const cur = queue[q];
    const r = (cur / w) | 0, c = cur % w;
    for (let d = 0; d < 4; d++) {
      const nr = r + DR[d], nc = c + DC[d];
      if (nr < 0 || nc < 0 || nr >= h || nc >= w) continue;
      const ni = nr * w + nc;
      if (seen[ni] || !playerCells[ni] || boxSet.has(ni)) continue;
      seen[ni] = 1;
      parent[ni] = cur;
      parentDir[ni] = d;
      queue.push(ni);
    }
  }
  return { seen, parent, parentDir };
}

function walkString(floodRes, from, to) {
  if (from === to) return '';
  const chars = [];
  let cur = to;
  while (cur !== from) {
    const d = floodRes.parentDir[cur];
    if (d < 0) return null;
    chars.push(WALK_CH[d]);
    cur = floodRes.parent[cur];
  }
  chars.reverse();
  return chars.join('');
}

// Push-level A* over one region.
//   boxCells: squares a box may occupy; playerCells: squares the player
//   may stand on; goals: this region's goal squares.
//   doorCells: squares that must hold no box in the accepted end state
//   (unless the square is itself a goal) when exitNeeded is set, and the
//   player must still be able to walk to one of them at the end.
// Returns { moves, pushes, endBoxes, endPlayer } or null.
function solveRegion(cfg) {
  const { w, h } = cfg;
  const goals = cfg.goals;
  const goalSet = new Set(goals);
  const dists = goals.map((g) => pushDistances(w, h, cfg.boxCells, cfg.playerCells, g));
  const dead = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) {
    if (!cfg.boxCells[i] || goalSet.has(i)) continue;
    let live = false;
    for (const dArr of dists) if (dArr[i] >= 0) { live = true; break; }
    if (!live) dead[i] = 1;
  }
  for (const b of cfg.startBoxes) if (dead[b]) return null;

  const heuristic = (boxes) => {
    const cost = boxes.map((b) => dists.map((dArr) => (dArr[b] < 0 ? INF : dArr[b])));
    return assignmentCost(cost);
  };

  const startH = heuristic(cfg.startBoxes);
  if (startH >= INF) return null;

  // Node store; a node is expanded at most once with its best g.
  const nodes = [];
  const best = new Map();
  const heap = []; // indices into nodes, ordered by (f, -g, seq)
  const heapLess = (a, b) => {
    const na = nodes[a], nb = nodes[b];
    if (na.f !== nb.f) return na.f < nb.f;
    if (na.g !== nb.g) return na.g > nb.g;
    return a < b;
  };
  const heapPush = (idx) => {
    heap.push(idx);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(heap[i], heap[p])) break;
      [heap[i], heap[p]] = [heap[p], heap[i]];
      i = p;
    }
  };
  const heapPop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < heap.length && heapLess(heap[l], heap[m])) m = l;
        if (r < heap.length && heapLess(heap[r], heap[m])) m = r;
        if (m === i) break;
        [heap[i], heap[m]] = [heap[m], heap[i]];
        i = m;
      }
    }
    return top;
  };

  const keyOf = (boxes, playerRep) => boxes.join(',') + '|' + playerRep;
  const addNode = (boxes, player, g, parent, segment) => {
    const f = flood(w, h, cfg.playerCells, new Set(boxes), player);
    let rep = player;
    for (let i = 0; i < f.seen.length; i++) if (f.seen[i]) { rep = i; break; }
    const key = keyOf(boxes, rep);
    if (best.has(key) && best.get(key) <= g) return -1;
    best.set(key, g);
    const idx = nodes.length;
    nodes.push({ boxes, player, g, f: g + heuristic(boxes), parent, segment, key });
    heapPush(idx);
    return idx;
  };

  addNode(cfg.startBoxes.slice(), cfg.startPlayer, 0, -1, '');
  let expansions = 0;

  while (heap.length) {
    const idx = heapPop();
    const node = nodes[idx];
    if (best.get(node.key) < node.g) continue; // stale heap entry
    // Goal test at pop time keeps the first accepted state optimal.
    let solved = true;
    for (const b of node.boxes) if (!goalSet.has(b)) { solved = false; break; }
    if (solved) {
      if (!cfg.exitNeeded || endStateClears(cfg, node)) {
        return finish(nodes, idx, node);
      }
      // Solved but the player would be trapped inside: keep searching.
    }
    if (++expansions > cfg.budget) return null;

    const boxSet = new Set(node.boxes);
    const reach = flood(w, h, cfg.playerCells, boxSet, node.player);
    for (let bi = 0; bi < node.boxes.length; bi++) {
      const b = node.boxes[bi];
      const br = (b / w) | 0, bc = b % w;
      for (let d = 0; d < 4; d++) {
        const sr = br - DR[d], sc = bc - DC[d]; // player stands here
        const tr = br + DR[d], tc = bc + DC[d]; // box lands here
        if (sr < 0 || sc < 0 || sr >= h || sc >= w) continue;
        if (tr < 0 || tc < 0 || tr >= h || tc >= w) continue;
        const stand = sr * w + sc;
        const dest = tr * w + tc;
        if (!reach.seen[stand]) continue;
        if (!cfg.boxCells[dest] || boxSet.has(dest) || dead[dest]) continue;
        const walk = walkString(reach, node.player, stand);
        if (walk === null) continue;
        const nextBoxes = node.boxes.slice();
        nextBoxes[bi] = dest;
        nextBoxes.sort((a, b2) => a - b2);
        addNode(nextBoxes, b, node.g + 1, idx, walk + PUSH_CH[d]);
      }
    }
  }
  return null;

  function endStateClears(cfg2, node) {
    for (const b of node.boxes) {
      if (cfg2.doorCells.has(b) && !goalSet.has(b)) return false;
    }
    if (!cfg2.doorCells.size) return true;
    const f = flood(w, h, cfg2.playerCells, new Set(node.boxes), node.player);
    for (const dcell of cfg2.doorCells) if (f.seen[dcell]) return true;
    return false;
  }
  function finish(allNodes, idx, node) {
    const parts = [];
    let cur = idx;
    while (cur >= 0) {
      parts.push(allNodes[cur].segment);
      cur = allNodes[cur].parent;
    }
    parts.reverse();
    return { moves: parts.join(''), pushes: node.g, endBoxes: node.boxes.slice(), endPlayer: node.player };
  }
}

// Replay a move string on a parsed level.  Returns { ok, pushes, boxes,
// player } — ok when every step is legal and every box ends on a goal.
function replay(level, moves) {
  const { w, h, wall, goal } = level;
  const boxes = new Set(level.boxes);
  let player = level.player;
  let pushes = 0;
  for (const ch of moves) {
    const d = 'udlr'.indexOf(ch.toLowerCase());
    if (d < 0) return { ok: false };
    const r = (player / w) | 0, c = player % w;
    const nr = r + DR[d], nc = c + DC[d];
    if (nr < 0 || nc < 0 || nr >= h || nc >= w) return { ok: false };
    const ni = nr * w + nc;
    if (wall[ni]) return { ok: false };
    if (boxes.has(ni)) {
      const br = nr + DR[d], bc = nc + DC[d];
      if (br < 0 || bc < 0 || br >= h || bc >= w) return { ok: false };
      const bi = br * w + bc;
      if (wall[bi] || boxes.has(bi)) return { ok: false };
      boxes.delete(ni);
      boxes.add(bi);
      pushes++;
    }
    player = ni;
  }
  for (const b of boxes) if (!goal[b]) return { ok: false, pushes, boxes, player };
  return { ok: true, pushes, boxes, player };
}

// Split the level into hallway + rooms and solve room by room.
// Returns a move string, or null when the level does not split cleanly.
function solveByRooms(level) {
  const { w, h, wall, goal, boxes, player } = level;
  const floor = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) floor[i] = wall[i] ? 0 : 1;
  const playerRow = (player / w) | 0;
  const hallway = new Uint8Array(w * h);
  for (let c = 0; c < w; c++) if (floor[playerRow * w + c]) hallway[playerRow * w + c] = 1;
  for (const b of boxes) if (hallway[b]) return null; // a box in the hallway

  // Components of the floor once the hallway row is removed.
  const compId = new Int32Array(w * h).fill(-1);
  const comps = [];
  for (let i = 0; i < w * h; i++) {
    if (!floor[i] || hallway[i] || compId[i] >= 0) continue;
    const cells = [];
    const queue = [i];
    compId[i] = comps.length;
    for (let q = 0; q < queue.length; q++) {
      const cur = queue[q];
      cells.push(cur);
      const r = (cur / w) | 0, c = cur % w;
      for (let d = 0; d < 4; d++) {
        const nr = r + DR[d], nc = c + DC[d];
        if (nr < 0 || nc < 0 || nr >= h || nc >= w) continue;
        const ni = nr * w + nc;
        if (!floor[ni] || hallway[ni] || compId[ni] >= 0) continue;
        compId[ni] = comps.length;
        queue.push(ni);
      }
    }
    comps.push(cells);
  }

  const rooms = [];
  for (const cells of comps) {
    const cellSet = new Set(cells);
    const rBoxes = boxes.filter((b) => cellSet.has(b));
    const rGoals = cells.filter((i) => goal[i]);
    if (rBoxes.length === 0 && rGoals.length === 0) continue;
    if (rBoxes.length === 0 || rBoxes.length !== rGoals.length) return null;
    const doors = new Set();
    const stances = new Set();
    for (const cell of cells) {
      const r = (cell / w) | 0, c = cell % w;
      for (let d = 0; d < 4; d++) {
        const nr = r + DR[d], nc = c + DC[d];
        if (nr < 0 || nc < 0 || nr >= h || nc >= w) continue;
        const ni = nr * w + nc;
        if (hallway[ni]) { doors.add(cell); stances.add(ni); }
      }
    }
    if (!doors.size) return null;
    rooms.push({ cells, cellSet, boxes: rBoxes, goals: rGoals, doors, stances });
  }
  if (!rooms.length) return null;
  const covered = rooms.reduce((n, r) => n + r.boxes.length, 0);
  if (covered !== boxes.length) return null;

  // Player may stand anywhere on floor; boxes stay inside their room.
  const solvedRooms = [];
  let blockers = 0;
  for (const room of rooms) {
    const boxCells = new Uint8Array(w * h);
    for (const cell of room.cells) boxCells[cell] = 1;
    const stance = Math.min(...room.stances);
    const base = {
      w, h,
      boxCells,
      playerCells: floor,
      goals: room.goals,
      startBoxes: room.boxes.slice(),
      startPlayer: stance,
      doorCells: room.doors,
      budget: 600000,
    };
    let res = solveRegion({ ...base, exitNeeded: true });
    let blocksExit = false;
    if (!res) {
      res = solveRegion({ ...base, exitNeeded: false });
      if (!res) return null;
      blocksExit = true;
      blockers++;
    }
    solvedRooms.push({ room, res, blocksExit, stance });
  }
  if (blockers > 1) return null;
  solvedRooms.sort((a, b) => (a.blocksExit ? 1 : 0) - (b.blocksExit ? 1 : 0));

  // Stitch: walk to each room's stance, then play its solution.
  let moves = '';
  let curBoxes = new Set(boxes);
  let curPlayer = player;
  for (const { res, stance } of solvedRooms) {
    const reach = flood(w, h, floor, curBoxes, curPlayer);
    const walk = walkString(reach, curPlayer, stance);
    if (walk === null) return null;
    moves += walk + res.moves;
    // Advance the stitched state by replaying this segment on the level.
    const sim = replayPartial(level, curBoxes, curPlayer, walk + res.moves);
    if (!sim) return null;
    curBoxes = sim.boxes;
    curPlayer = sim.player;
  }
  const check = replay(level, moves);
  return check.ok ? moves : null;
}

// Replay moves from an arbitrary state; null on an illegal step.
function replayPartial(level, startBoxes, startPlayer, moves) {
  const { w, h, wall } = level;
  const boxes = new Set(startBoxes);
  let player = startPlayer;
  for (const ch of moves) {
    const d = 'udlr'.indexOf(ch.toLowerCase());
    if (d < 0) return null;
    const r = (player / w) | 0, c = player % w;
    const nr = r + DR[d], nc = c + DC[d];
    if (nr < 0 || nc < 0 || nr >= h || nc >= w) return null;
    const ni = nr * w + nc;
    if (wall[ni]) return null;
    if (boxes.has(ni)) {
      const br = nr + DR[d], bc = nc + DC[d];
      if (br < 0 || bc < 0 || br >= h || bc >= w) return null;
      const bi = br * w + bc;
      if (wall[bi] || boxes.has(bi)) return null;
      boxes.delete(ni);
      boxes.add(bi);
    }
    player = ni;
  }
  return { boxes, player };
}

function solveWhole(level) {
  const { w, h, wall, goal, boxes, player } = level;
  const floor = new Uint8Array(w * h);
  const goals = [];
  for (let i = 0; i < w * h; i++) {
    floor[i] = wall[i] ? 0 : 1;
    if (goal[i]) goals.push(i);
  }
  const res = solveRegion({
    w, h,
    boxCells: floor,
    playerCells: floor,
    goals,
    startBoxes: boxes.slice(),
    startPlayer: player,
    doorCells: new Set(),
    exitNeeded: false,
    budget: 2500000,
  });
  return res ? res.moves : '';
}

function solveLevel(text) {
  const level = parseLevel(text);
  if (level.player < 0) return '';
  if (level.boxes.length === 0) return '';
  const byRooms = solveByRooms(level);
  if (byRooms !== null) return byRooms;
  return solveWhole(level);
}

for (const id of readdirSync(IN_DIR).sort()) {
  const text = readFileSync(join(IN_DIR, id, 'level.txt'), 'utf8');
  const moves = solveLevel(text);
  mkdirSync(join(OUT_DIR, id), { recursive: true });
  writeFileSync(join(OUT_DIR, id, 'solution.txt'), `${moves}\n`);
}
