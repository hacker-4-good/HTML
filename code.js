// Performs Dijkstra's algorithm; returns *all* nodes in the order
// in which they were visited. Also makes nodes point back to their
// previous node, effectively allowing us to compute the shortest path
// by backtracking from the finish node.
export function dijkstra(grid, startNode, finishNode) {
    const visitedNodesInOrder = [];
    startNode.distance = 0;
    const unvisitedNodes = getAllNodes(grid);
    while (!!unvisitedNodes.length) {
      sortNodesByDistance(unvisitedNodes);
      const closestNode = unvisitedNodes.shift();
      // If we encounter a wall, we skip it.
      if (closestNode.isWall) continue;
      // If the closest node is at a distance of infinity,
      // we must be trapped and should therefore stop.
      if (closestNode.distance === Infinity) return visitedNodesInOrder;
      closestNode.isVisited = true;
      visitedNodesInOrder.push(closestNode);
      if (closestNode === finishNode) return visitedNodesInOrder;
      updateUnvisitedNeighbors(closestNode, grid);
    }
  }
  
  function sortNodesByDistance(unvisitedNodes) {
    unvisitedNodes.sort((nodeA, nodeB) => nodeA.distance - nodeB.distance);
  }
  
  function updateUnvisitedNeighbors(node, grid) {
    const unvisitedNeighbors = getUnvisitedNeighbors(node, grid);
    for (const neighbor of unvisitedNeighbors) {
      neighbor.distance = node.distance + 1;
      neighbor.previousNode = node;
    }
  }
  
  function getUnvisitedNeighbors(node, grid) {
    const neighbors = [];
    const {col, row} = node;
    if (row > 0) neighbors.push(grid[row - 1][col]);
    if (row < grid.length - 1) neighbors.push(grid[row + 1][col]);
    if (col > 0) neighbors.push(grid[row][col - 1]);
    if (col < grid[0].length - 1) neighbors.push(grid[row][col + 1]);
    return neighbors.filter(neighbor => !neighbor.isVisited);
  }
  
  function getAllNodes(grid) {
    const nodes = [];
    for (const row of grid) {
      for (const node of row) {
        nodes.push(node);
      }
    }
    return nodes;
  }
  
  // Backtracks from the finishNode to find the shortest path.
  // Only works when called *after* the dijkstra method above.
  export function getNodesInShortestPathOrder(finishNode) {
    const nodesInShortestPathOrder = [];
    let currentNode = finishNode;
    while (currentNode !== null) {
      nodesInShortestPathOrder.unshift(currentNode);
      currentNode = currentNode.previousNode;
    }
    return nodesInShortestPathOrder;
  }






   
/**
 * A* Search Algorithm
 * 
 * @author Darren Tynan
 * @date Jan 2019
 */
class Astar
{
    /**
     * Constructor
     * 
     * @param {*} grid - pointer to grid 
     * @param {*} rows a actual number of rows (y)
     * @param {*} cols - actual number of columns (x)
     */
    constructor(grid, rows, cols)
    {
        // Grid
        this.grid = grid;

        // Adjustments for number of rows and columns,
        // as array's are zero indexed.
        this.cols = cols - 1;
        this.rows = rows - 1;

        // Node info
        this.sourceNode = null;
        this.targetNode = null;
        this.currentNode = null;

        // astar
        this.neighbors = [];
        this.path = new Queue;

        this.openSet = [];
        this.closeSet = [];
        this.isFound = false;
    }

    /**
     * Initialization
     * 
     * @param {*} sourceNode 
     * @param {*} targetNode 
     */
    init(sourceNode, targetNode)
    {
        this.sourceNode = sourceNode;
        this.targetNode = targetNode;

        // Initial frontier to search from.
        this.openSet.push(this.sourceNode);
    }

    /**
     * The most intensive part of the process is the
     * sub-routine too search the openSet for the node
     * with the lowest f cost.
     * An optimized alternative would be to use a heap.  
     */
    findPath()
    {
        while (this.openSet.length > 0)
        {
            // Find the node with the lowest f on the open set
            let lowF = 0;
            for (let i = 0; i < this.openSet.length; i++)
            {
                if (this.openSet[i].f < this.openSet[lowF].f || 
                    this.openSet[i].f == this.openSet[lowF].f && 
                    this.openSet[i].h < this.openSet[lowF].h)
                {
                    lowF = i;
                }
            }

            // Set lowF as current node
            this.currentNode = this.openSet[lowF];
         
            // Remove lowF from the open set
            this.openSet.splice( this.openSet.indexOf(this.currentNode), 1);

            // Add to close set
            this.closeSet.push(this.currentNode);

            // Are we done?
            if (this.currentNode == this.targetNode)
            {
                this.isFound = true;

                // Stop the sketch.js draw() loop.
                noLoop();

                console.log("DONE");
                while (this.currentNode != this.sourceNode)
                {
                    this.path.enqueue(this.currentNode);
                    this.currentNode = this.currentNode.parent;
                }

                // iterate over path and set id to 'path'.
                let pf;
                let size = this.path.size();
                for (let p = 0; p < size; p++)
                {
                    pf = this.path.dequeue();
                    pf.id = "path";
                    // Yellow
                    pf.drawSet("#FFF700");
                }

                return;
            }

            // Clear neighbors array for next iteration.
            this.neighbors = [];

            // Find neighbors of current node.
            this.findNeighbors(this.grid, this.currentNode);

            // Iterate over neighbors.
            for (var i = 0; i < this.neighbors.length; i++)
            {
                if (this.closeSet.includes(this.neighbors[i]) || this.neighbors[i].id == "wall")
                {
                    continue;
                }

                // if (!this.closeSet.includes(this.neighbors[i]) || this.neighbors[i].id != "wall")
                else
                {
                    // Set parent node for path creation.
                    this.neighbors[i].parent = this.currentNode;

                    // gfx debug
                    if (this.neighbors[i].id != "source")
                    {
                        this.neighbors[i].id = "frontier";
                    }

                    // Calculate costs.
                    this.neighbors[i].g += this.currentNode.g;
                    // this.neighbors[i].g = this.currentNode.g + this.heuristic(this.neighbors[i], this.currentNode);
                    this.neighbors[i].h = this.heuristic_Diagonal(this.targetNode, this.neighbors[i]);
                    this.neighbors[i].f = this.neighbors[i].g + this.neighbors[i].h;
    
                    if (!this.openSet.includes(this.neighbors[i]))
                    {
                        this.openSet.push(this.neighbors[i]);
                    }
                }
            }
        }
    }

    /**
     * Calculate the heuristic cost between Source and Target node.
     * Based on Manhattan ie North, East, South, West.
     * With a cost of 10.
     *  
     * @param {*} s 
     * @param {*} t 
     */
    heuristic_Manhattan(s, t)
    {
        let dx = abs(s.x - t.x)
        let dy = abs(s.y - t.y);
        let d = 10;
        return d * (dx + dy);
    }

    /**
     * Calculate the heuristic cost between Source and Target node.
     * Based on 8-way direction.
     * With cost of 10 and 14 for diagonal.
     * 
     * @param {*} s 
     * @param {*} t 
     */
    heuristic_Diagonal(s, t)
    {
        let dx = abs(s.x - t.x)
        let dy = abs(s.y - t.y);
        let d = 10;
        let d2 = 10;
        return d * (dx + dy) + (d2 - 2 * d) * min(dx, dy);
    }

    /**
     * Find neighbors of current node.
     * But, only if, it's not been previously visited.
     * 
     * @param {*} grid 
     * @param {*} node 
     */
    findNeighbors(grid, node)
    {
        // North
        if (node.y > 0)
        {
            grid[node.y - 1][node.x].g = 10;
            this.neighbors.push(grid[node.y - 1][node.x]);
        }
        // NW
        if (node.y > 0 && node.x > 0)
        {
            grid[node.y - 1][node.x - 1].g = 14;
            this.neighbors.push(grid[node.y - 1][node.x - 1]);
        }
        // West
        if (node.x > 0)
        {
            grid[node.y][node.x - 1].g = 10;
            this.neighbors.push(grid[node.y][node.x - 1]);
        }
        // SW
        if (node.y < this.rows && node.x > 0)
        {
            grid[node.y + 1][node.x - 1].g = 14;
            this.neighbors.push(grid[node.y + 1][node.x - 1]);
        }
        // South
        if (node.y < this.rows)
        {
            grid[node.y + 1][node.x].g = 10;
            this.neighbors.push(grid[node.y + 1][node.x]);
        }
        // SE
        if (node.y < this.rows && node.x < this.cols)
        {
            grid[node.y + 1][node.x + 1].g = 14;
            this.neighbors.push(grid[node.y + 1][node.x + 1]);
        }
        // East
        if (node.x < this.cols)
        {
            grid[node.y][node.x + 1].g = 10;
            this.neighbors.push(grid[node.y][node.x + 1]);
        }
        // NE
        if (node.y > 0 && node.x < this.cols)
        {
            grid[node.y - 1][node.x + 1].g = 14;
            this.neighbors.push(grid[node.y - 1][node.x + 1]);
        }
    }
}