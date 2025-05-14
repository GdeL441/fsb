export function shortestPath(COLS, ROWS, dots) {
  const start = dots.find(dot => dot.color == "start")
  const red = dots.filter(dot => dot.color == "red")
  const green = dots.filter(dot => dot.color == "green")

  if (!start || green.length == 0) {
    console.log("Make sure there is a start position and atleast one green dot")
    return null
  }

  // Initialize grid
  let grid = Array.from({ length: ROWS }).map((_, i) =>
    Array.from({ length: COLS }).map((_, j) => ({ x: j + 1, y: i + 1, color: "empty" }))
  );
  for (const dot of dots) {
    grid[dot.y - 1][dot.x - 1] = dot
  }

  let nodes = [start, ...green]
  let dists = new Map();
  // Compute shortest paths between all nodes using BFS with prioritization
  for (let i = 0; i < nodes.length; i++) {
    for (let j = i + 1; j < nodes.length; j++) {
      let path = prioritizedBfs(ROWS, COLS, grid, nodes[i], nodes[j]);
      if (path) {
        // Calculate time cost instead of just distance
        let timeCost = calculatePathTime(path);
        
        if (!dists.has(nodes[i])) dists.set(nodes[i], new Map());
        if (!dists.has(nodes[j])) dists.set(nodes[j], new Map());

        let reversed = [...path].reverse()
        if (path[0].x != nodes[i].x || path[0].y != nodes[i].y) {
          dists.get(nodes[i]).set(nodes[j], { dist: timeCost, path: reversed });
          dists.get(nodes[j]).set(nodes[i], { dist: timeCost, path: path });
        } else {
          dists.get(nodes[i]).set(nodes[j], { dist: timeCost, path: path });
          dists.get(nodes[j]).set(nodes[i], { dist: timeCost, path: reversed });
        }
      }
    }
  }

  let bestCost = Infinity;
  let bestPath = null;
  let bestStartDirection = "N";

  // Try all four possible starting directions
  const startDirections = ["N", "E", "S", "W"];
  
  for (const startDirection of startDirections) {
    let visited = new Set();
    visited.add(start);
    
    function backtrack(position, cost, visitedNodes, path, currentDirection) {
      if (visitedNodes.size == nodes.length) {
        let pathHome = dists.get(position)?.get(start)
        if (!pathHome) return

        const newCost = cost + pathHome.dist
        if (newCost < bestCost) {
          bestCost = newCost
          bestPath = [...path, ...pathHome.path.slice(1)]
          bestStartDirection = startDirection
        }
        return
      }

      // Get all unvisited nodes
      const nodesToVisit = nodes.filter(node => node !== position && !visitedNodes.has(node));
      
      // No sorting based on direction - we want to explore all possibilities
      for (const node of nodesToVisit) {
        let pathBetween = dists.get(position)?.get(node)
        if (pathBetween) {
          // Calculate the actual initial direction when moving from start
          let actualFirstDirection = null;
          if (position === start && pathBetween.path.length > 1) {
            actualFirstDirection = getDirection(pathBetween.path[0], pathBetween.path[1]);
            
            // Skip paths that don't match our desired initial direction
            if (actualFirstDirection !== startDirection) continue;
          }
          
          visitedNodes.add(node);
          backtrack(
            node, 
            cost + pathBetween.dist, 
            visitedNodes, 
            [...path, ...pathBetween.path.slice(1)], 
            actualFirstDirection || currentDirection
          );
          visitedNodes.delete(node);
        }
      }
    }

    // Start backtracking with the current startDirection
    backtrack(start, 0, visited, [start], startDirection);
  }

  if (bestPath) {
    const directions = pathToDirections(bestPath, bestStartDirection)
    return { cost: bestCost, path: bestPath, directions, startDirection: bestStartDirection }
  }
}

function calculatePathTime(path) {
  if (path.length < 2) return 0;
  
  let time = 0;
  let currentDirection = getDirection(path[0], path[1]);
  
  for (let i = 1; i < path.length - 1; i++) {
    const nextDirection = getDirection(path[i], path[i+1]);
    if (nextDirection === currentDirection) {
      time += 1367; // Straight movement
    } else {
      time += 950; // Turn
      currentDirection = nextDirection;
    }
  }
  
  // Add time for the last segment
  time += 1367;
  
  return time;
}

function getDirection(from, to) {
  if (from.y > to.y) return 'N';
  if (from.y < to.y) return 'S';
  if (from.x > to.x) return 'W';
  return 'E';
}

function prioritizedBfs(ROWS, COLS, grid, start, goal) {
  let queue = [[start, [start]]];
  let visited = new Set([`${start.x},${start.y}`]);
  let foundPaths = [];

  while (queue.length > 0) {
    let [pos, path] = queue.shift();
    let { x, y } = pos;

    if (x === goal.x && y === goal.y) {
      foundPaths.push(path);
      continue; // Keep looking for other paths to find the most optimal one
    }

    // Get neighbors in a way that prioritizes straight movement
    const neighbors = [];
    const lastDirection = path.length > 1 ? getDirection(path[path.length-2], path[path.length-1]) : null;
    
    // Add straight movement first if continuing in same direction
    if (lastDirection) {
      const [dx, dy] = getDirectionVector(lastDirection);
      const nx = x + dx, ny = y + dy;
      addNeighborIfValid(nx, ny, neighbors, ROWS, COLS, grid, visited);
    }
    
    // Then add other directions
    for (let direction of ['N', 'S', 'E', 'W']) {
      if (!lastDirection || direction !== lastDirection) {
        const [dx, dy] = getDirectionVector(direction);
        const nx = x + dx, ny = y + dy;
        addNeighborIfValid(nx, ny, neighbors, ROWS, COLS, grid, visited);
      }
    }

    for (const neighbor of neighbors) {
      const key = `${neighbor.x},${neighbor.y}`;
      visited.add(key);
      queue.push([neighbor, [...path, neighbor]]);
    }
  }

  // Return the path with the lowest time cost
  if (foundPaths.length > 0) {
    foundPaths.sort((a, b) => calculatePathTime(a) - calculatePathTime(b));
    return foundPaths[0];
  }
  return null;
}

function getDirectionVector(direction) {
  switch (direction) {
    case 'N': return [0, -1];
    case 'S': return [0, 1];
    case 'E': return [1, 0];
    case 'W': return [-1, 0];
    default: return [0, 0];
  }
}

function addNeighborIfValid(nx, ny, neighbors, ROWS, COLS, grid, visited) {
  const i = nx - 1, j = ny - 1;
  const key = `${nx},${ny}`;
  if (i >= 0 && j >= 0 && j < ROWS && i < COLS && grid[j][i].color !== 'red' && !visited.has(key)) {
    neighbors.push(grid[j][i]);
  }
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