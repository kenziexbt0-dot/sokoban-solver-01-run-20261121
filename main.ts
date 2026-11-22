// Push-optimal Sokoban solver, written for the make.fun bounty
// "Build the best Sokoban solver, 21 November 2026" (sokoban-solver-01-run-20261121).
//
// Idea: the generated levels are a 1-wide hallway with 1-4 rooms off it,
// each room holding 3-4 boxes and as many goals, and a box pushed out of
// a room can never come back - so every box stays in its room and the
// level's push-optimal count is the sum of its rooms' optima. The solver
// therefore (1) finds the rooms as the small box/goal-balanced components
// cut off by articulation points of the floor graph, (2) solves each room
// push-optimally with A* over pushes (admissible min-assignment heuristic
// plus dead-square / corner / 2x2 / wall-segment deadlock pruning), and
// (3) stitches the rooms together with plain hallway walks. Only the push
// count is scored, so walking is free and the room order is arbitrary.
//
// Determinism: every iteration order is fixed, every search budget is a
// count of expanded nodes (never the wall clock), so two runs of this
// program on the same input write byte-identical output.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Four moves: up, down, left, right. Index 0..3.
const DR = [-1, 1, 0, 0];
const DC = [0, 0, -1, 1];
const LETTER = ['u', 'd', 'l', 'r'];

// ---------------------------------------------------------------------------
// Small deterministic binary heap (min-heap on [f, seq]) holding node ids.
// ---------------------------------------------------------------------------
class Heap {
  keys: number[] = []; // f values, parallel to ids
  seqs: number[] = [];
  ids: number[] = [];
  get size(): number {
    return this.ids.length;
  }
  push(f: number, seq: number, id: number): void {
    const { keys, seqs, ids } = this;
    keys.push(f);
    seqs.push(seq);
    ids.push(id);
    let i = ids.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (keys[p] < keys[i] || (keys[p] === keys[i] && seqs[p] <= seqs[i])) break;
      this.swap(i, p);
      i = p;
    }
  }
  pop(): number {
    const { keys, seqs, ids } = this;
    const top = ids[0];
    const lastKey = keys.pop() as number;
    const lastSeq = seqs.pop() as number;
    const lastId = ids.pop() as number;
    if (ids.length > 0) {
      keys[0] = lastKey;
      seqs[0] = lastSeq;
      ids[0] = lastId;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1;
        const r = l + 1;
        let m = i;
        if (l < ids.length && (keys[l] < keys[m] || (keys[l] === keys[m] && seqs[l] < seqs[m]))) m = l;
        if (r < ids.length && (keys[r] < keys[m] || (keys[r] === keys[m] && seqs[r] < seqs[m]))) m = r;
        if (m === i) break;
        this.swap(i, m);
        i = m;
      }
    }
    return top;
  }
  private swap(i: number, j: number): void {
    const { keys, seqs, ids } = this;
    [keys[i], keys[j]] = [keys[j], keys[i]];
    [seqs[i], seqs[j]] = [seqs[j], seqs[i]];
    [ids[i], ids[j]] = [ids[j], ids[i]];
  }
}

// ---------------------------------------------------------------------------
// Level parsing
// ---------------------------------------------------------------------------
interface Level {
  w: number;
  h: number;
  wall: Uint8Array; // 1 = wall, 0 = floor
  goals: number[]; // cell ids
  goalSet: Uint8Array;
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.replace(/\r/g, '').split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goalSet = new Uint8Array(w * h);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  for (let r = 0; r < h; r++) {
    for (let c = 0; c < w; c++) {
      const ch = rows[r][c] ?? '#';
      const id = r * w + c;
      if (ch === '#') continue;
      wall[id] = 0;
      if (ch === '.' || ch === '*' || ch === '+') {
        goalSet[id] = 1;
        goals.push(id);
      }
      if (ch === '$' || ch === '*') boxes.push(id);
      if (ch === '@' || ch === '+') player = id;
    }
  }
  return { w, h, wall, goals, goalSet, boxes, player };
}

function neighbours(id: number, w: number, h: number): number[] {
  const r = Math.floor(id / w);
  const c = id % w;
  const out: number[] = [];
  for (let d = 0; d < 4; d++) {
    const nr = r + DR[d];
    const nc = c + DC[d];
    if (nr >= 0 && nr < h && nc >= 0 && nc < w) out.push(nr * w + nc);
  }
  return out;
}

// BFS walk for the player on `passable` cells (Uint8Array 1 = can stand).
// Returns the move string from `from` to `to`, or null when unreachable.
function walkPath(
  from: number,
  to: number,
  passable: Uint8Array,
  w: number,
  h: number,
): string | null {
  if (from === to) return '';
  const prev = new Int32Array(w * h).fill(-2);
  const prevDir = new Int8Array(w * h).fill(-1);
  const queue: number[] = [from];
  prev[from] = -1;
  let head = 0;
  while (head < queue.length) {
    const cur = queue[head++];
    for (let d = 0; d < 4; d++) {
      const nr = Math.floor(cur / w) + DR[d];
      const nc = (cur % w) + DC[d];
      if (nr < 0 || nr >= h || nc < 0 || nc >= w) continue;
      const nid = nr * w + nc;
      if (!passable[nid] || prev[nid] !== -2) continue;
      prev[nid] = cur;
      prevDir[nid] = d;
      if (nid === to) {
        const letters: string[] = [];
        let p = nid;
        while (p !== from) {
          letters.push(LETTER[prevDir[p]]);
          p = prev[p];
        }
        letters.reverse();
        return letters.join('');
      }
      queue.push(nid);
    }
  }
  return null;
}

// ---------------------------------------------------------------------------
// Room detection: articulation points of the floor graph.
// Removing a door cell splits the floor into components; a room is a small
// component (3-4 boxes, as many goals) on the far side of a door.
// ---------------------------------------------------------------------------
function articulationPoints(level: Level): Set<number> {
  const { w, h, wall } = level;
  const n = w * h;
  const disc = new Int32Array(n).fill(-1);
  const low = new Int32Array(n);
  const isArt = new Uint8Array(n);
  let time = 0;
  // Iterative DFS so deep floors cannot overflow the call stack.
  // Frame: [cell, next neighbour index, parent, childCount].
  for (let start = 0; start < n; start++) {
    if (wall[start] || disc[start] !== -1) continue;
    disc[start] = low[start] = time++;
    const stack: { v: number; ni: number; parent: number; kids: number }[] = [
      { v: start, ni: 0, parent: -1, kids: 0 },
    ];
    while (stack.length > 0) {
      const frame = stack[stack.length - 1];
      const v = frame.v;
      const nbs = neighbours(v, w, h).filter((x) => !wall[x]);
      if (frame.ni < nbs.length) {
        const to = nbs[frame.ni++];
        if (to === frame.parent) continue;
        if (disc[to] !== -1) {
          low[v] = Math.min(low[v], disc[to]);
        } else {
          disc[to] = low[to] = time++;
          frame.kids++;
          stack.push({ v: to, ni: 0, parent: v, kids: 0 });
        }
      } else {
        stack.pop();
        const p = frame.parent;
        if (p !== -1) {
          low[p] = Math.min(low[p], low[v]);
          // v is an articulation point of its parent when the subtree
          // below v cannot reach above p (checked at the parent frame).
          const parentFrame = stack[stack.length - 1];
          if (parentFrame && parentFrame.parent !== -1 && low[v] >= disc[p]) {
            isArt[p] = 1;
          }
        } else if (frame.kids > 1) {
          isArt[v] = 1;
        }
      }
    }
  }
  const out = new Set<number>();
  for (let i = 0; i < n; i++) if (isArt[i]) out.add(i);
  return out;
}

// Components of the floor graph with `removed` taken out.
function componentsWithout(level: Level, removed: number): number[][] {
  const { w, h, wall } = level;
  const seen = new Uint8Array(w * h);
  seen[removed] = 1;
  const comps: number[][] = [];
  for (let s = 0; s < w * h; s++) {
    if (wall[s] || seen[s]) continue;
    const comp: number[] = [];
    const queue: number[] = [s];
    seen[s] = 1;
    let head = 0;
    while (head < queue.length) {
      const cur = queue[head++];
      comp.push(cur);
      for (const nb of neighbours(cur, w, h)) {
        if (!wall[nb] && !seen[nb]) {
          seen[nb] = 1;
          queue.push(nb);
        }
      }
    }
    comps.push(comp);
  }
  return comps;
}

interface Room {
  cells: number[]; // room floor cells (boxes live here)
  cellSet: Uint8Array;
  door: number; // articulation cell the player enters through
  boxes: number[];
  goals: number[];
}

function findRooms(level: Level): Room[] {
  const arts = articulationPoints(level);
  const candidates: Room[] = [];
  for (const door of [...arts].sort((a, b) => a - b)) {
    for (const comp of componentsWithout(level, door)) {
      if (comp.includes(level.player)) continue;
      if (comp.length > 48) continue;
      const boxes = comp.filter((c) => level.boxes.includes(c));
      const goals = comp.filter((c) => level.goalSet[c] === 1);
      if (boxes.length < 3 || boxes.length > 4) continue;
      if (boxes.length !== goals.length) continue;
      const cellSet = new Uint8Array(level.w * level.h);
      for (const c of comp) cellSet[c] = 1;
      candidates.push({ cells: comp, cellSet, door, boxes, goals });
    }
  }
  // Keep only minimal candidates: drop any candidate that strictly
  // contains another candidate's cells (a hallway-side component that
  // happens to hold a whole room plus a stub of hallway).
  const keep: Room[] = [];
  for (const cand of candidates) {
    let contained = false;
    for (const other of candidates) {
      if (other === cand) continue;
      if (other.cells.length < cand.cells.length && other.cells.every((c) => cand.cellSet[c] === 1)) {
        contained = true;
        break;
      }
    }
    if (!contained) keep.push(cand);
  }
  // Drop overlaps (should not happen after the minimal filter, but be safe).
  keep.sort((a, b) => a.cells.length - b.cells.length || a.door - b.door);
  const chosen: Room[] = [];
  const used = new Uint8Array(level.w * level.h);
  for (const room of keep) {
    if (room.cells.some((c) => used[c])) continue;
    for (const c of room.cells) used[c] = 1;
    chosen.push(room);
  }
  return chosen;
}

// ---------------------------------------------------------------------------
// Push-optimal A* over one room.
// State: sorted box cells + canonical player cell (the lowest cell the
// player can reach). Moves: one push. Cost: pushes. Heuristic: the cheapest
// assignment of boxes to goals by plain floor distance (each push moves one
// box one cell, so this never overestimates). Deadlock pruning: dead
// squares, corners, 2x2 blocks and wall segments without a goal.
// ---------------------------------------------------------------------------
interface Budget {
  expansions: number;
  limit: number;
}

interface PushStep {
  from: number; // box cell before the push (player ends here)
  to: number; // box cell after the push
  dir: number; // 0..3
}

interface RoomSolution {
  pushes: PushStep[];
  playerStart: number;
}

function solveRoom(
  level: Level,
  boxCells: Uint8Array, // cells a box may stand on
  playerCells: Uint8Array, // cells the player may stand on
  playerStart: number,
  boxes: number[],
  goals: number[],
  budget: Budget,
): RoomSolution | null {
  const { w, h } = level;
  const n = w * h;
  const nb = boxes.length;

  // Plain floor distances from each goal over the box cells.
  const distFromGoal: Int32Array[] = goals.map((g) => {
    const dist = new Int32Array(n).fill(-1);
    const queue: number[] = [g];
    dist[g] = 0;
    let head = 0;
    while (head < queue.length) {
      const cur = queue[head++];
      for (const nbCell of neighbours(cur, w, h)) {
        if (!boxCells[nbCell] || dist[nbCell] !== -1) continue;
        dist[nbCell] = dist[cur] + 1;
        queue.push(nbCell);
      }
    }
    return dist;
  });
  // A box cell is a dead square when no goal is reachable from it.
  const live = new Uint8Array(n);
  for (let c = 0; c < n; c++) {
    if (!boxCells[c]) continue;
    for (let g = 0; g < goals.length; g++) {
      if (distFromGoal[g][c] !== -1) {
        live[c] = 1;
        break;
      }
    }
  }
  const goalSet = new Uint8Array(n);
  for (const g of goals) goalSet[g] = 1;

  // Cheapest box->goal assignment by brute-force permutation (<= 4 boxes).
  const permCache = new Map<string, number>();
  function heuristic(sortedBoxes: number[]): number {
    const key = sortedBoxes.join(',');
    const cached = permCache.get(key);
    if (cached !== undefined) return cached;
    let best = Infinity;
    const used = new Array(goals.length).fill(false);
    const order = [...sortedBoxes].sort((a, b) => a - b);
    function rec(i: number, cost: number): void {
      if (cost >= best) return;
      if (i === order.length) {
        best = cost;
        return;
      }
      for (let g = 0; g < goals.length; g++) {
        if (used[g]) continue;
        const d = distFromGoal[g][order[i]];
        if (d === -1) continue;
        used[g] = true;
        rec(i + 1, cost + d);
        used[g] = false;
      }
    }
    rec(0, 0);
    permCache.set(key, best);
    return best;
  }

  // Deadlock test for a box placement (boxes sorted). True = dead.
  // A cell is a true wall only when neither a box nor the player can use
  // it: the room's door cell blocks boxes but the player can stand on it
  // and push from it, so it is not a wall for these tests. Other boxes are
  // ignored throughout (they can move), which keeps every test sound.
  function isDead(sortedBoxes: number[]): boolean {
    const boxSet = new Uint8Array(n);
    for (const b of sortedBoxes) boxSet[b] = 1;
    const trueWall = (c: number): boolean => boxCells[c] === 0 && playerCells[c] === 0;
    const canPush = (from: number, d: number): boolean => {
      const r = Math.floor(from / w);
      const c = from % w;
      const nr = r + DR[d];
      const nc = c + DC[d];
      const br = r - DR[d];
      const bc = c - DC[d];
      if (nr < 0 || nr >= h || nc < 0 || nc >= w) return false;
      if (br < 0 || br >= h || bc < 0 || bc >= w) return false;
      return boxCells[nr * w + nc] === 1 && playerCells[br * w + bc] === 1;
    };
    for (const b of sortedBoxes) {
      if (goalSet[b]) continue;
      if (!live[b]) return true;
      // Frozen: no push is possible even ignoring the other boxes.
      if (!canPush(b, 0) && !canPush(b, 1) && !canPush(b, 2) && !canPush(b, 3)) return true;
      const r = Math.floor(b / w);
      const c = b % w;
      // 2x2 block of true walls/boxes (a door cell does not block).
      for (const [dr, dc] of [[0, 0], [-1, 0], [0, -1], [-1, -1]] as const) {
        const r0 = r + dr;
        const c0 = c + dc;
        if (r0 < 0 || c0 < 0 || r0 + 1 >= h || c0 + 1 >= w) continue;
        const cells = [r0 * w + c0, r0 * w + c0 + 1, (r0 + 1) * w + c0, (r0 + 1) * w + c0 + 1];
        if (cells.every((x) => trueWall(x) || boxSet[x] === 1) && cells.some((x) => boxSet[x] === 1)) {
          return true;
        }
      }
      // Wall segment without a goal: a box pressed against a true wall can
      // only slide along it, and if its whole run is closed off at both
      // ends by walls and holds no goal, it can never reach one.
      const wallRunDead = (horizontal: boolean, wallDir: -1 | 1): boolean => {
        if (horizontal) {
          const wr = r + wallDir;
          const wallAt = (cc: number): boolean => wr < 0 || wr >= h || trueWall(wr * w + cc);
          if (!wallAt(c)) return false;
          let lo = c;
          while (lo - 1 >= 0 && boxCells[r * w + lo - 1] && wallAt(lo - 1)) lo--;
          let hi = c;
          while (hi + 1 < w && boxCells[r * w + hi + 1] && wallAt(hi + 1)) hi++;
          const closed = (lo - 1 < 0 || !boxCells[r * w + lo - 1]) && (hi + 1 >= w || !boxCells[r * w + hi + 1]);
          if (!closed) return false;
          for (let cc = lo; cc <= hi; cc++) if (goalSet[r * w + cc]) return false;
          return true;
        }
        const wc = c + wallDir;
        const wallAt = (rr: number): boolean => wc < 0 || wc >= w || trueWall(rr * w + wc);
        if (!wallAt(r)) return false;
        let lo = r;
        while (lo - 1 >= 0 && boxCells[(lo - 1) * w + c] && wallAt(lo - 1)) lo--;
        let hi = r;
        while (hi + 1 < h && boxCells[(hi + 1) * w + c] && wallAt(hi + 1)) hi++;
        const closed = (lo - 1 < 0 || !boxCells[(lo - 1) * w + c]) && (hi + 1 >= h || !boxCells[(hi + 1) * w + c]);
        if (!closed) return false;
        for (let rr = lo; rr <= hi; rr++) if (goalSet[rr * w + c]) return false;
        return true;
      };
      if (wallRunDead(true, -1) || wallRunDead(true, 1) || wallRunDead(false, -1) || wallRunDead(false, 1)) {
        return true;
      }
    }
    return false;
  }

  // Player flood from `start` over playerCells avoiding `boxSet`.
  function flood(start: number, boxSet: Uint8Array): Uint8Array {
    const seen = new Uint8Array(n);
    const queue: number[] = [start];
    seen[start] = 1;
    let head = 0;
    while (head < queue.length) {
      const cur = queue[head++];
      for (const nbCell of neighbours(cur, w, h)) {
        if (!playerCells[nbCell] || boxSet[nbCell] || seen[nbCell]) continue;
        seen[nbCell] = 1;
        queue.push(nbCell);
      }
    }
    return seen;
  }
  function canonicalPlayer(reach: Uint8Array): number {
    for (let i = 0; i < n; i++) if (reach[i]) return i;
    return -1;
  }

  const startBoxes = [...boxes].sort((a, b) => a - b);
  if (startBoxes.every((b) => goalSet[b] === 1)) {
    return { pushes: [], playerStart };
  }
  if (isDead(startBoxes)) return null;
  if (!Number.isFinite(heuristic(startBoxes))) return null;

  interface Node {
    boxes: number[];
    player: number; // actual player cell after the push that made this node
    parent: number; // node id, -1 for the root
    push: PushStep | null;
    g: number;
  }
  const nodes: Node[] = [];
  const bestG = new Map<string, number>();
  const heap = new Heap();
  let seq = 0;
  const rootFlood = flood(playerStart, (() => {
    const s = new Uint8Array(n);
    for (const b of startBoxes) s[b] = 1;
    return s;
  })());
  const rootKey = `${startBoxes.join(',')}|${canonicalPlayer(rootFlood)}`;
  nodes.push({ boxes: startBoxes, player: playerStart, parent: -1, push: null, g: 0 });
  bestG.set(rootKey, 0);
  heap.push(heuristic(startBoxes), seq++, 0);

  while (heap.size > 0) {
    if (budget.expansions >= budget.limit) return null;
    const id = heap.pop();
    const node = nodes[id];
    budget.expansions++;
    const nodeKey = `${node.boxes.join(',')}|${canonicalPlayer(flood(node.player, boxSetOf(node.boxes, n)))}`;
    if ((bestG.get(nodeKey) ?? Infinity) < node.g) continue;
    if (node.boxes.every((b) => goalSet[b] === 1)) {
      const pushes: PushStep[] = [];
      let cur: Node | undefined = node;
      while (cur && cur.push) {
        pushes.push(cur.push);
        cur = nodes[cur.parent];
      }
      pushes.reverse();
      return { pushes, playerStart };
    }
    const boxSet = boxSetOf(node.boxes, n);
    const reach = flood(node.player, boxSet);
    for (let i = 0; i < nb; i++) {
      const b = node.boxes[i];
      const r = Math.floor(b / w);
      const c = b % w;
      for (let d = 0; d < 4; d++) {
        const nr = r + DR[d];
        const nc = c + DC[d];
        if (nr < 0 || nr >= h || nc < 0 || nc >= w) continue;
        const to = nr * w + nc;
        const br = r - DR[d];
        const bc = c - DC[d];
        if (br < 0 || br >= h || bc < 0 || bc >= w) continue;
        const behind = br * w + bc;
        if (!boxCells[to] || boxSet[to]) continue;
        if (!playerCells[behind] || boxSet[behind] || !reach[behind]) continue;
        const nextBoxes = [...node.boxes];
        nextBoxes[i] = to;
        nextBoxes.sort((a, b2) => a - b2);
        if (isDead(nextBoxes)) continue;
        const hh = heuristic(nextBoxes);
        if (!Number.isFinite(hh)) continue;
        const nextFlood = flood(b, boxSetOf(nextBoxes, n));
        const key = `${nextBoxes.join(',')}|${canonicalPlayer(nextFlood)}`;
        const g2 = node.g + 1;
        if ((bestG.get(key) ?? Infinity) <= g2) continue;
        bestG.set(key, g2);
        const nid = nodes.length;
        nodes.push({ boxes: nextBoxes, player: b, parent: id, push: { from: b, to, dir: d }, g: g2 });
        heap.push(g2 + hh, seq++, nid);
      }
    }
  }
  return null;
}

function boxSetOf(boxes: number[], n: number): Uint8Array {
  const s = new Uint8Array(n);
  for (const b of boxes) s[b] = 1;
  return s;
}

// ---------------------------------------------------------------------------
// Whole-level solver: split into rooms, solve each, stitch with walks.
// ---------------------------------------------------------------------------
function solveLevel(text: string, budget: Budget): string {
  const level = parseLevel(text);
  const n = level.w * level.h;
  const rooms = findRooms(level);

  // Every box must belong to exactly one room; otherwise fall back to
  // solving the whole level as a single room.
  const covered = new Uint8Array(n);
  let allCovered = rooms.length > 0;
  for (const room of rooms) {
    for (const b of room.boxes) covered[b] = 1;
  }
  if (allCovered) {
    for (const b of level.boxes) if (!covered[b]) allCovered = false;
  }

  // Current box placement, updated as rooms are solved.
  const curBoxes = [...level.boxes];
  let player = level.player;
  let moves = '';

  // Walk the player over floor cells that hold no box right now.
  const walk = (to: number): string | null => {
    const passable = new Uint8Array(n);
    for (let i = 0; i < n; i++) passable[i] = level.wall[i] === 0 ? 1 : 0;
    for (const b of curBoxes) passable[b] = 0;
    passable[player] = 1;
    return walkPath(player, to, passable, level.w, level.h);
  };

  // Turn a room's push list into player moves, starting from `player`.
  const playPushes = (
    room: Room,
    sol: RoomSolution,
    playerCells: Uint8Array,
  ): string | null => {
    const boxSet = new Uint8Array(n);
    for (const b of room.boxes) boxSet[b] = 1;
    let out = '';
    let cur = player;
    for (const push of sol.pushes) {
      const behind = push.from - DR[push.dir] * level.w - DC[push.dir];
      const passable = new Uint8Array(n);
      for (let i = 0; i < n; i++) passable[i] = playerCells[i];
      for (let i = 0; i < n; i++) if (boxSet[i]) passable[i] = 0;
      passable[cur] = 1;
      const leg = walkPath(cur, behind, passable, level.w, level.h);
      if (leg === null) return null;
      out += leg + LETTER[push.dir];
      boxSet[push.from] = 0;
      boxSet[push.to] = 1;
      cur = push.from;
    }
    return out;
  };

  if (allCovered) {
    // Solve rooms nearest-first from the player's current position.
    const remaining = [...rooms];
    while (remaining.length > 0) {
      // Pick the reachable room whose door is nearest by walking distance.
      let bestIdx = -1;
      let bestWalk: string | null = null;
      for (let i = 0; i < remaining.length; i++) {
        const w = walk(remaining[i].door);
        if (w !== null && (bestWalk === null || w.length < bestWalk.length)) {
          bestIdx = i;
          bestWalk = w;
        }
      }
      if (bestIdx === -1) return '';
      const room = remaining.splice(bestIdx, 1)[0];
      moves += bestWalk as string;
      player = room.door;
      // Boxes may stand on (and pass through) the door cell on their way to
      // a goal in the room; they just cannot leave through it into the
      // hallway, so the room's box space is its cells plus the door.
      const boxCells = new Uint8Array(n);
      for (const c of room.cells) boxCells[c] = 1;
      boxCells[room.door] = 1;
      const sol = solveRoom(level, boxCells, roomCellPlayerSpace(level, room), player, room.boxes, room.goals, budget);
      if (sol === null) return '';
      const played = playPushes(room, sol, roomCellPlayerSpace(level, room));
      if (played === null) return '';
      moves += played;
      // Update the world: the room's boxes are now on its goals, and the
      // player stands where the last push left them.
      for (const b of room.boxes) {
        const idx = curBoxes.indexOf(b);
        if (idx !== -1) curBoxes.splice(idx, 1);
      }
      for (const g of room.goals) curBoxes.push(g);
      player = lastPlayerAfterPushes(room, sol);
    }
  } else {
    // Fallback: one room covering the whole floor.
    const boxCells = new Uint8Array(n);
    const playerCells = new Uint8Array(n);
    for (let i = 0; i < n; i++) {
      if (!level.wall[i]) {
        boxCells[i] = 1;
        playerCells[i] = 1;
      }
    }
    const sol = solveRoom(level, boxCells, playerCells, level.player, level.boxes, level.goals, budget);
    if (sol === null) return '';
    const fakeRoom: Room = {
      cells: [],
      cellSet: boxCells,
      door: level.player,
      boxes: [...level.boxes],
      goals: [...level.goals],
    };
    const played = playPushes(fakeRoom, sol, playerCells);
    if (played === null) return '';
    moves += played;
    player = lastPlayerAfterPushes(fakeRoom, sol);
  }

  // Verify by replaying the whole move string on the original level.
  if (!replayOk(level, moves)) return '';
  return moves;
}

function roomCellPlayerSpace(level: Level, room: Room): Uint8Array {
  const s = new Uint8Array(level.w * level.h);
  for (const c of room.cells) s[c] = 1;
  s[room.door] = 1;
  return s;
}

// Player cell after the last push of a room solution: the box's old cell.
function lastPlayerAfterPushes(room: Room, sol: RoomSolution): number {
  if (sol.pushes.length === 0) return room.door;
  return sol.pushes[sol.pushes.length - 1].from;
}

// Replay `moves` on the level; true when every box ends on a goal.
function replayOk(level: Level, moves: string): boolean {
  const boxes = new Set<number>(level.boxes);
  let player = level.player;
  for (const ch of moves) {
    const d = LETTER.indexOf(ch);
    if (d === -1) return false;
    const r = Math.floor(player / level.w);
    const c = player % level.w;
    const nr = r + DR[d];
    const nc = c + DC[d];
    if (nr < 0 || nr >= level.h || nc < 0 || nc >= level.w) return false;
    const next = nr * level.w + nc;
    if (level.wall[next]) return false;
    if (boxes.has(next)) {
      const br = nr + DR[d];
      const bc = nc + DC[d];
      if (br < 0 || br >= level.h || bc < 0 || bc >= level.w) return false;
      const beyond = br * level.w + bc;
      if (level.wall[beyond] || boxes.has(beyond)) return false;
      boxes.delete(next);
      boxes.add(beyond);
    }
    player = next;
  }
  for (const b of boxes) if (!level.goalSet[b]) return false;
  return true;
}

// ---------------------------------------------------------------------------
// Driver: one run answers every instance in /input, writing /output.
// ---------------------------------------------------------------------------
const budget: Budget = { expansions: 0, limit: 3_000_000 };
for (const id of readdirSync(INPUT_DIR).sort()) {
  const moves = solveLevel(readFileSync(join(INPUT_DIR, id, 'level.txt'), 'utf8'), budget);
  mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
  writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), `${moves}\n`);
}
