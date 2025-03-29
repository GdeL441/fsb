export function shortestPath(COLS, ROWS, dots) {
  const start = dots.find(dot => dot.color == "start")
  const red = dots.filter(dot => dot.color == "red")
  const green = dots.filter(dot => dot.color == "green")

  if (!start || green.length == 0) {
    console.log("Make sure there is a start position and atleast one green dot")
    return null
  }


  let grid = Array.from({ length: ROWS }).map((_, i) =>
    Array.from({ length: COLS }).map((_, j) => ({ x: j + 1, y: i + 1, color: "empty" }))
  );
  for (const dot of dots) {
    grid[dot.y - 1][dot.x - 1] = dot
  }

  let nodes = [start, ...green]
  let dists = new Map();
  // Compute shortest paths between all nodes using BFS
  for (let i = 0; i < nodes.length; i++) {
    for (let j = i + 1; j < nodes.length; j++) {
      let path = bfs(ROWS, COLS, grid, nodes[i], nodes[j]);
      if (path) {
        let dist = path.length - 1;
        // save the found shortest path in the dists map on both the start end end position
        if (!dists.has(nodes[i])) dists.set(nodes[i], new Map());
        if (!dists.has(nodes[j])) dists.set(nodes[j], new Map());

        let reversed = [...path].reverse()
        if (path[0].x != nodes[i].x || path[0].y != nodes[i].y) {
          dists.get(nodes[i]).set(nodes[j], { dist, path: reversed });
          dists.get(nodes[j]).set(nodes[i], { dist, path: path });
        } else {
          dists.get(nodes[i]).set(nodes[j], { dist, path: path });
          dists.get(nodes[j]).set(nodes[i], { dist, path: reversed });
        }

      }
    }
  }


  let bestCost = 1000000
  let bestPath = null

  const backtrack = (position, cost, visitedNodes, path) => {
    if (visitedNodes.size == nodes.length) {
      // All nodes visited, if the cost is lower than the previous, save the found path
      if (cost < bestCost) {
        bestCost = cost
        bestPath = path
      }
      return
    }

    // Now visit all unvisited nodes from the current position
    for (const node of nodes) {

      if (node == position || visitedNodes.has(node)) {
        // Skip if already visited
        continue
      }


      // Look up the distance and path between the current position and the node
      let pathBetween = dists.get(position)?.get(node)
      if (pathBetween) {
        // Now mark this node as visited
        visited.add(node);
        backtrack(node, cost + pathBetween.dist, visited, [...path, ...pathBetween.path.slice(1)]);
        // After a backtracking step ended, remove it from visited so 
        // it can be visisted from a different path in the next iterations
        visited.delete(node);

      }
    }
  };

  let visited = new Set()
  // Add start to visited, since it is also included in nodes
  visited.add(start)
  // Start the backtracking
  backtrack(start, 0, visited, [start])

  const directions = pathToDirections(bestPath)
  return bestPath ? { cost: bestCost, path: bestPath, directions } : null
}

// Calculate the shortest path between two nodes, later used to calculate the overal shortest path
function bfs(ROWS, COLS, grid, start, goal) {
  let queue = [[start, [start]]]; // (current position, path taken to reach goal)
  let visited = new Set([`${start[0]},${start[1]}`]);

  while (queue.length > 0) {
    let [pos, path] = queue.shift();
    let { x, y } = pos;

    if (x === goal.x && y === goal.y) return path; // Shortest path found

    for (let [dx, dy] of [[-1, 0], [1, 0], [0, -1], [0, 1]]) { // Up, Down, Left, Right
      let nx = x + dx, ny = y + dy;
      let key = `${nx},${ny}`;
      let i = nx - 1, j = ny - 1

      if (i >= 0 && j >= 0 && j < ROWS && i < COLS && grid[j][i].color !== 'red' && !visited.has(key)) {
        queue.push([grid[j][i], [...path, grid[j][i]]]);
        visited.add(key);
      }
    }
  }
  return null; // No path found
}


function pathToDirections(path, heading = "N") {
  const directions = []
  for (let i = 0; i < path.length - 1; i++) {
    const from = path[i], to = path[i + 1]

    if (from.y == to.y - 1) {
      // Go up 1
      if (heading == "N") directions.push("FORWARD")
      else if (heading == "E") directions.push(["LEFT", "FORWARD"])
      else if (heading == "S") directions.push(["LEFT", "LEFT", "FORWARD"])
      else if (heading == "W") directions.push(["RIGHT", "FORWARD"])

      heading = "N"
    } else if (from.y == to.y + 1) {
      // Go down 1
      if (heading == "S") directions.push("FORWARD")
      else if (heading == "N") directions.push(["LEFT", "LEFT", "FORWARD"])
      else if (heading == "E") directions.push(["RIGHT", "FORWARD"])
      else if (heading == "W") directions.push(["LEFT", "FORWARD"])
      heading = "S"
    } else if (from.x == to.x - 1) {
      // Go right 1
      if (heading == "E") directions.push("FORWARD")
      else if (heading == "N") directions.push(["RIGHT", "FORWARD"])
      else if (heading == "S") directions.push(["LEFT", "FORWARD"])
      else if (heading == "W") directions.push(["LEFT", "LEFT", "FORWARD"])
      heading = "E"
    } else if (from.x == to.x + 1) {
      // Go left 1
      if (heading == "W") directions.push("FORWARD")
      else if (heading == "N") directions.push(["LEFT", "FORWARD"])
      else if (heading == "E") directions.push(["LEFT", "LEFT", "FORWARD"])
      else if (heading == "S") directions.push(["RIGHT", "FORWARD"])
      heading = "W"
    }
  };

  return directions.flat()
}
