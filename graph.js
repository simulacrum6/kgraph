/*
 * MAJOR TODO: Consider making Graph Objects reactive, using Observer Pattern? 
 *      + more robust
 *      - performance decrease
 */

// TODO: Remove?
function Vertex (id, label = id) {
	this.id = id;
    this.label = label;
}

function Edge (from, to, weight = 1) {
    this.from = from;
    this.to = to;
    this.weight = weight;
}

function GraphFactory () {
	this.createGraph = function (buildInstruction) {
        var graph = new Graph(buildInstruction.vertexList); 
            graph.addEdge = buildInstruction.addEdgeFunction;
            graph.show = buildInstruction.showFunction;

            buildInstruction.edges = buildInstruction.edges || [];
            buildInstruction.edges.forEach(function addEdges(edge) {
                graph.addEdge(edge.from, edge.to, edge.weight);
            });

        return graph;
    }
    this.createUndirectedGraph = function createUndirectedGraph (vertexList, edges = []) {
        var buildInstructions = new GraphBuildInstruction(vertexList, edges, addUndirectedEdge, showUndirected);

        return this.createGraph(buildInstructions);
    }
    this.createUndirectedWeightedGraph = function createUndirectedWeightedGraph (vertexList, edges = []) {
        var buildInstructions = new GraphBuildInstruction(vertexList, edges, addUndirectedEdge, showUndirectedWeighted);

        return this.createGraph(buildInstructions);
    }
    this.createDirectedGraph = function createDirectedGraph (vertexList, edges = []) {
       var buildInstructions = new GraphBuildInstruction(vertexList, edges, addDirectedEdge, showDirected);

       return this.createGraph(buildInstructions);
    }
    this.createDirectedWeightedGraph = function createDirectedWeightedGraph (vertexList, edges = []) {
       var buildInstructions = new GraphBuildInstruction(vertexList, edges, addDirectedEdge, showDirectedWeighted);

       return this.createGraph(buildInstructions);
    }

    function addDirectedEdge (from, to, weight = 1) {
        this.edgeList[from].push(to);
        this.edgeWeights[from].push(weight);
		this.edgeCount++;
    }
 
    function addUndirectedEdge (vertex1, vertex2, weight = 1) {
        this.edgeList[vertex1].push(vertex2);
        this.edgeWeights[vertex1].push(weight);
		this.edgeList[vertex2].push(vertex1);
        this.edgeWeights[vertex2].push(weight);
        this.edgeCount++;
    }
        
    function showDirected () {
        for (var i = 0; i < this.vertices; i++)
        console.log(i + " -> " + this.edgeList[i] + " [" + this.edgeList[i].length + "]" );
    }
    function showDirectedWeighted () {
        for (var i = 0; i < this.vertices; i++)
        console.log(i + " -" + this.edgeWeights[i] + "-> " + this.edgeList[i] + " [" + this.edgeList[i].length + "]" );
    }
    function showUndirected () {
        for (var i = 0; i < this.vertices; i++)
        console.log(i + " <-> " + this.edgeList[i] + " [" + this.edgeList[i].length + "]" );
    }
    function showUndirectedWeighted () {
        for (var i = 0; i < this.vertices; i++)
        console.log(i + " <-" + this.edgeWeights[i] + "-> " + this.edgeList[i] + " [" + this.edgeList[i].length + "]" );
    }

    // REFACTOR: Make properties private
    function Graph (vertexList) {	
        this.vertexList = vertexList;
        this.vertices = vertexList.length;
        
        this.edgeCount = 0;
        this.edgeList = [];
        this.edgeWeights = [];

        // Initialise edgeList & edgeWeights
        for (var i = 0; i < this.vertices; i++){
            this.edgeList[i] = [];
            this.edgeWeights[i] = [];
        }
            
        this.addEdge = null;
        this.show = null;
        
        this.getVertex = function getVertex (vertex) {
            return this.vertexList[vertex];
        }
        this.getNeighbours = function getNeighbours (vertex) {
            return this.edgeList[vertex];
        }
        this.getEdgeWeight = function getEdgeWeight (from, to) {
            if (from == to) return 0;

            var index = this.edgeList[from].indexOf(to);
            return this.edgeWeights[from][index];
        }
    }

    function GraphBuildInstruction (vertexList, edges, addEdgeFunction, showFunction) {
        this.vertexList = vertexList;
        this.addEdgeFunction = addEdgeFunction;
        this.showFunction = showFunction;
        this.edges = edges;        
    }
}

/*
 * MAJOR TODO: Aggregate searches in CommandObjects, make GraphSearcher ActiveObject.
 *      Interface Search: search, nextCandidate, targetCheck, exploreCandidate, retrievePathTo
 *      Interface GraphSearcher: run, add
 *      
 * MAJOR TODO: Create general Queue interface and Factory? -> Overkill, create Queue and overwrite interface
 */ 
// REFACTOR: Clean up code Duplication
// TODO: Allow vertices as search inputs, not only vertexids
// TODO: Rework SearchNode, 
function GraphSearcher (graph) {
	this.graph = graph;

    // TODO: Implement recursive and Iterative version?
    this.depthFirstSearch = function depthFirstSearch (start, target) {
        var startTime = window.performance.now();
        var startNode = new SimpleSearchNode(start, null);
        
        var frontier = [startNode];
        var explored = {};

        while (frontier.length > 0) {
            var candidate = frontier.pop();

            if (candidate.id === target) {
                console.log("DFS Elapsed Time:" + (window.performance.now() - startTime));
                return retrievePathTo(candidate);
            }
            
            explored[candidate.id] = true;
            var neighbours = graph.getNeighbours(candidate.id);

            for (var i in neighbours) {
                if (!explored[neighbours[i]])
                    frontier.push(new SimpleSearchNode(neighbours[i], candidate));
            }
        }
        return [];
    }
	this.dfs = this.depthFirstSearch;
	
    // REFACTOR: queue : Array -> frontier : Queue.
	this.breadthFirstSearch = function breadthFirstSearch (start, target) {		
		var explored = {};
		var queue = [start];
		
		while (queue.length > 0) {
			var vertex = queue.shift();
            var neighbours = this.graph.getNeighbours(vertex);
			
			if (neighbours !== undefined) 				
				neighbours.forEach(visitNeighbour, this);
			
			function visitNeighbour (neighbour) {
				if (!explored[neighbour]) {
					explored[neighbour] = true;
					queue.push(neighbour);
				}
			}
		}
	}
	this.bfs = this.breadthFirstSearch;

	this.aStarSearch = function aStarSearch (start, target, heuristicFunction, costFunction) {    
        var startTime = window.performance.now();    
        
        var heuristic = function (vertex) {
			return heuristicFunction(graph.getVertex(vertex), graph.getVertex(target));
        }
        var costs = function (vertex, anotherVertex) {
            return costFunction(graph.getVertex(vertex), graph.getVertex(anotherVertex));
        }
        
        var frontier = new PriorityQueue();
        var explored = {};
        var startNode = new SearchNode(start, null);
        
        frontier.add(startNode, startNode.estimatedCosts);

        while (frontier.size() > 0) {        
            var candidate = frontier.poll();

            if (isTarget(candidate)){
                console.log("A* Elapsed Time:" + (window.performance.now() - startTime));
                return retrievePathTo(candidate);
            }
                                
            
            expandFrontier(candidate);
        }
        console.log("No Path between nodes found. Returning closest path.");
        return retrievePathTo(candidate);

        function expandFrontier (searchNode) {
            explored[searchNode.id] = true;
            
            var neighbours = graph.getNeighbours(searchNode.id);
            
            neighbours.forEach(function (neighbour) {
                if (!explored[neighbour]) {
                    var neighbourNode = new SearchNode(neighbour, searchNode);
                    frontier.add(neighbourNode, neighbourNode.estimatedCosts);
                }
            });
            
        }

        function retrievePathTo (searchNode) {
            if (searchNode.parent == null) 
                return [searchNode.id];
            else
                return [searchNode.id].concat(retrievePathTo(searchNode.parent));
        }

        function isTarget(searchNode) {
            return searchNode.id == target;
        }

        function SearchNode (vertex, parent) {
            this.id = vertex;
            this.parent = parent;
            this.costs = (parent !== null) ? costs(parent.id, vertex) + parent.costs : 0;
            this.estimatedCosts = heuristic(vertex) + this.costs;
        }
	}

    function retrievePathTo (searchNode) {
        if (searchNode.parent == null) 
            return [searchNode.id];
        else
            return [searchNode.id].concat(retrievePathTo(searchNode.parent));
    }

    function SimpleSearchNode (vertex, parentNode) {
        this.id = vertex;
        this.parent = parentNode;
    }
}   

// TODO: Move to queue.js file
function PriorityQueue () {
    var dataStore = [];

    this.size = function size () {
        return dataStore.length;
    }

    this.poll = function poll () {
        if(this.size() === 0) 
            return null;

        return dataStore.pop().data;
    }

    this.peek = function peek () {
        if(this.size() === 0)
            return null;

        return dataStore.peek().data;    
    }

    this.add = function add (data, priority) {        
        dataStore.splice(findInsertionIndex(priority), 0, new PriorityNode(data, priority));      
    }

    function findInsertionIndex(value) {
        if (dataStore.length === 0)
            return 0;
        
        var minIndex = 0;
        var maxIndex = dataStore.length;
        
        do {
            var currentIndex = Math.floor((minIndex + maxIndex) / 2);
            var currentValue = dataStore[currentIndex].priority;

            if (currentValue > value)
                minIndex = currentIndex + 1;
            else
                maxIndex = currentIndex;
        
        } while (minIndex < maxIndex);

        return minIndex;
    }

    function PriorityNode (data, priority) {
        this.data = data;
        this.priority = priority;
    }
}

function GraphSearch (queueConstructorFunction) {
    this.search = function search (start, target, graph) {
        // TODO: Abstract to any collection?
        var frontier = new Queue (start);
        var explored = {};

        while (frontier.size() > 0) {
            // TODO: Abstract to getNext()?
            var candidate = frontier.poll();

            if (isTarget(candidate))
                return candidate.retrievePath();
            
            expandFrontier(candidate);
        }
        return candidate.retrievePath();
    }

    function isTarget (searchNode) {
        return searchNode.id === target;
    }

    function expandFrontier (searchNode) {
        explored[searchNode] = true;

        var neighbours = graph.getNeighbours(searchNode.id);
        for (var i in neighbours) {
            if (!explored[neighbours[i]]) {
                var neighbourNode = new SearchNode(neighbours[i], searchNode);
                // TODO: Abstract to addToFrontier()? 
                frontier.add(neighbourNode, neighbourNode.estimatedCosts);
            }
        }
    }

    // TODO: just pass a queue object in Constructor?
    function Queue (initialValue = null) {
        queueConstructorFunction.call(this);

        if (initialValue !== null)
            this.add(initialValue);
    }

    function SearchNode (vertexId, parentNode = null) {
        this.id = vertexId;
        this.parentNode = parentNode;

        this.retrievePath = function retrievePath () {
            if (this.parentNode == null)
                return this.id;

            return [this.id].concat(this.parentNode.retrievePath.call(this.parentNode));
        }
    }
}


/*
Recursive Depth First Search
this.depthFirstSearch = function depthFirstSearch (start, target, alreadyExplored = {}) {
    var explored = alreadyExplored;
    explored[start] = true;
    var neighbours = this.graph.getNeighbours(start);

    for (var i in neighbours) {
        var neighbour = neighbours[i];

        if (neighbour == target)
        
        if (!explored[neighbour])
            this.dfs(neighbour, target, explored);
    }
}*/

/* Max First
function findInsertionIndex(value, priorityQueue) {
    if (priorityQueue.dataStore.length == 0) {
        console.log("Queue empty. Returning [0]")
        return 0;
    }
    
    var minIndex = 0;
    var maxIndex = priorityQueue.dataStore.length;
    var currentIndex = 0;
    var currentValue = 0;

    do {
        currentIndex = Math.floor((minIndex + maxIndex) / 2);
        currentValue = priorityQueue.dataStore[currentIndex].priority;

        if (currentValue < value) {
            minIndex = currentIndex + 1;
        }
        else {
            maxIndex = currentIndex;
        }
    } while (minIndex < maxIndex);

    return minIndex;
}
*/

// TODO: make selection in Constructor for Min/Max PriorityQueue
    // TODO: Implement in this way?
    // var comparison = isMaxQueue ? currentValue < value : currentValue > value;