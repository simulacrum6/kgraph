(function (kgraph, $, undefined) {
    var Vertex = (function () {
        class Vertex {
            constructor(id, label = id) {
                this.id = id;
                this.label = label;
            }
            toString() {
                return '{id: ' + this.id + ', label: ' + this.label + '}';
            }
        }

        return Vertex
    })()

    var Edge = (function () {
        class Edge {
            constructor(from, to, weight = 1) {
                this.from = from;
                this.to = to;
                this.weight = weight;
            }
            toString() {
                return '{from: ' + this.from + ', to: ' + this.to + ', weight: ' + this.weight.toFixed(2) + '}';
            }
        }

        return Edge
    })()

    var GraphFactory = (function () {
        class GraphFactory {
            constructor() { }

            createGraph(buildInstruction) {
                var graph = new Graph(buildInstruction.nodes);
                graph.addEdge = buildInstruction.addEdgeFunction;
                graph.show = buildInstruction.showFunction;
                buildInstruction.edges = buildInstruction.edges || [];
                buildInstruction.edges.forEach(function (edge) {
                    graph.addEdge(edge.from, edge.to, edge.weight);
                });
                return graph;
            }
            createUndirectedGraph(nodes, edges = []) {
                var buildInstructions = new GraphBuildInstruction(nodes, edges, addUndirectedEdge, showUndirected);
                return this.createGraph(buildInstructions);
            }
            createUndirectedWeightedGraph(nodes, edges = []) {
                var buildInstructions = new GraphBuildInstruction(nodes, edges, addUndirectedEdge, showUndirectedWeighted);
                return this.createGraph(buildInstructions);
            }
            createDirectedGraph(nodes, edges = []) {
                var buildInstructions = new GraphBuildInstruction(nodes, edges, addDirectedEdge, showDirected);
                return this.createGraph(buildInstructions);
            }
            createDirectedWeightedGraph(nodes, edges = []) {
                var buildInstructions = new GraphBuildInstruction(nodes, edges, addDirectedEdge, showDirectedWeighted);
                return this.createGraph(buildInstructions);
            }
        }

        class GraphBuildInstruction {
            constructor(nodes, edges, addEdgeFunction, showFunction) {
                this.nodes = nodes;
                this.addEdgeFunction = addEdgeFunction;
                this.showFunction = showFunction;
                this.edges = edges;
            }
        }

        class Graph {
            constructor(nodes) {
                this.nodes = nodes;
                this.edges = []
                this.nodeCount = nodes.length
                this.edgeCount = 0;
                this.edgeList = [];
                this.edgeWeights = [];
                this.adjacencyList = [];
                for (var i = 0; i < this.nodeCount; i++) {
                    this.edgeList[i] = [];
                    this.edgeWeights[i] = [];
                    this.adjacencyList[i] = new Array(this.nodeCount).fill(Infinity);
                    this.adjacencyList[i][i] = 0;
                }
                this.addEdge = function () { };
                this.show = function () { };
            }

            addNode(node) {
                this.nodes.push(node);
                this.nodeCount++;
                this.edgeList.push([]);
                this.edgeWeights.push([]);
                this.adjacencyList.forEach(row => row.push(0));
                this.adjacencyList.push(new Array(this.nodeCount).fill(Infinity));
                this.adjacencyList[this.nodeCount - 1][this.nodeCount - 1] = 0;
            }
            contains(id) {
                if (typeof id === 'number')
                    return this.nodes[id] ? true : false;
                if (id instanceof Array)
                    return this.containsAll(id);
                return false;
            }
            containsAll(ids) {
                return ids.every(id => this.contains(id));
            }
            getVertex(id) {
                return this.nodes[id];
            }
            getNeighbours(id) {
                return this.edgeList[id];
            }
            degree(id) {
                return this.getNeighbours(id).length;
            }
            averageDegree() {
                // TODO: Fix for directed graphs.
                return 2 * this.edgeCount / this.nodeCount;
            }
            isIsolated(id) {
                return this.degree(id) === 0;
            }
            getEdgeWeight(from, to) {
                if (from == to)
                    return 0;
                var index = this.edgeList[from].indexOf(to);
                return this.edgeWeights[from][index];
            }
            getAdjacencyList() {
                return this.adjacencyList;
            }
        }

        // Graph addEdge & show functions
        function addDirectedEdge(from, to, weight = 1) {
            this.edgeList[from].push(to);
            this.edgeWeights[from].push(weight);
            this.adjacencyList[from][to] = weight;
            this.edgeCount++;
            this.edges.push(new Edge(from, to, weight))
        }
        function addUndirectedEdge(from, to, weight = 1) {
            this.edgeList[from].push(to);
            this.edgeList[to].push(from);
            this.edgeWeights[from].push(weight);
            this.edgeWeights[to].push(weight);
            this.adjacencyList[from][to] = weight;
            this.adjacencyList[to][from] = weight;
            this.edgeCount++;
            this.edges.push(new Edge(from, to, weight))
            this.edges.push(new Edge(to, from, weight))
        }

        function showDirected() {
            for (var i = 0; i < this.nodeCount; i++)
                console.log(i + " -> " + this.edgeList[i] + " [" + this.edgeList[i].length + "]");
        }
        function showDirectedWeighted() {
            for (var i = 0; i < this.nodeCount; i++)
                console.log(i + " -" + this.edgeWeights[i].map(x => x.toFixed(2)) + "-> " + this.edgeList[i] + " [" + this.edgeList[i].length + "]");
        }
        function showUndirected() {
            for (var i = 0; i < this.nodeCount; i++)
                console.log(i + " <-> " + this.edgeList[i] + " [" + this.edgeList[i].length + "]");
        }
        function showUndirectedWeighted() {
            for (var i = 0; i < this.nodeCount; i++)
                console.log(i + " <-" + this.edgeWeights[i].map(x => x.toFixed(2)) + "-> " + this.edgeList[i] + " [" + this.edgeList[i].length + "]");
        }

        return GraphFactory
    })()


    var GraphSearcher = (function () {
        class GraphSearcher {
            constructor(graph) {
                this.graph = graph;
            }
            distance(start, target) {
                var path = this.bfs(start, target)
                if (path.length > 0)
                    return path.length
                return Infinity
            }
            pathLengths() {
                var lengths = []
                for (let i = 0; i < this.graph.edgeCount; i++) {
                    for (let j = 0; j < this.graph.edgeCount; j++) {
                        var path = this.bfs(i, j)
                        lengths.push(path.length)
                    }
                }
                return lengths
            }
            diameter() {
                var lengths = this.pathLengths()
                return lengths.reduce((a, b) => Math.max(a, b), 0)
            }
            radius() {
                // TODO
            }
            depthFirstSearch(start, target) {
                var startTime = window.performance.now();
                var startNode = new SimpleSearchNode(start, null);
                var frontier = [startNode];
                var explored = {};
                while (frontier.length > 0) {
                    var candidate = frontier.pop();
                    if (candidate.id == target) {
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
            breadthFirstSearch(start, target) {
                if (start === target)
                    return [start]

                var explored = {};
                var queue = [new SimpleSearchNode(start, null)];
                while (queue.length > 0) {
                    var candidate = queue.shift();
                    var neighbours = this.graph.getNeighbours(candidate.id);
                    if (neighbours !== undefined) {
                        if (neighbours.includes(target)) {
                            return retrievePathTo(new SimpleSearchNode(target, candidate));
                        }
                        neighbours.forEach(visitNeighbour);
                    }
                }
                return [];
                function visitNeighbour(neighbour) {
                    if (!explored[neighbour]) {
                        explored[neighbour] = true;
                        queue.push(new SimpleSearchNode(neighbour, candidate));
                    }
                }
            }
            getPath(start, target, directed) {
                directed = directed === undefined ? true : directed;
                var path = this.bfs(start, target);
                if (path.length === 0 && directed === false) {
                    path = this.bfs(target, start);
                }
                return path;
            }
            aStarSearch(start, target, heuristicFunction, costFunction) {
                var startTime = window.performance.now();
                var heuristic = function (vertex) {
                    return heuristicFunction(graph.getVertex(vertex), graph.getVertex(target));
                };
                var costs = function (vertex, anotherVertex) {
                    return costFunction(graph.getVertex(vertex), graph.getVertex(anotherVertex));
                };
                var frontier = new PriorityQueue();
                var explored = {};
                var startNode = new SearchNode(start, null);
                frontier.add(startNode, startNode.estimatedCosts);
                while (frontier.size() > 0) {
                    var candidate = frontier.poll();
                    if (isTarget(candidate)) {
                        console.log("A* Elapsed Time:" + (window.performance.now() - startTime));
                        return retrievePathTo(candidate);
                    }
                    expandFrontier(candidate);
                }
                console.log("No Path between nodes found. Returning closest path.");

                return retrievePathTo(candidate);

                function expandFrontier(searchNode) {
                    explored[searchNode.id] = true;
                    var neighbours = graph.getNeighbours(searchNode.id);
                    neighbours.forEach(function (neighbour) {
                        if (!explored[neighbour]) {
                            var neighbourNode = new SearchNode(neighbour, searchNode);
                            frontier.add(neighbourNode, neighbourNode.estimatedCosts);
                        }
                    });
                }
                function retrievePathTo(searchNode) {
                    if (searchNode.parent == null)
                        return [searchNode.id];
                    else
                        return [searchNode.id].concat(retrievePathTo(searchNode.parent));
                }
                function isTarget(searchNode) {
                    return searchNode.id == target;
                }
                function SearchNode(vertex, parent) {
                    this.id = vertex;
                    this.parent = parent;
                    this.costs = (parent !== null) ? costs(parent.id, vertex) + parent.costs : 0;
                    this.estimatedCosts = heuristic(vertex) + this.costs;
                }
            }
            getPathNeighbours() {
                /**
                 * Returns a matrix M of closest neighbours on a path from 
                 * Node a to Node b.
                 * 
                 * M[a][b] returns the closest neighbour of a on the path to b.
                 * The returned neighbour is the node index in the source graph.
                 */
                var pathNeighbour = [];
                var graph = this.graph;

                for (var n = 0; n < graph.nodeCount; n++) {
                    pathNeighbour[n] = [];
                }

                for (var i = 0; i < graph.nodeCount; i++) {
                    var explored = [];
                    var layer = [i];
                    var nextLayer = [];

                    explored[i] = true;

                    while (layer.length > 0) {
                        nextLayer = getNextLayer(layer);
                        nextLayer.forEach(vertex => {
                            pathNeighbour[vertex][i] = getClosestTo(vertex, layer);
                        });
                        layer = nextLayer;
                    }
                }

                return pathNeighbour;

                function getClosestTo(vertex, candidates) {
                    var neighbours = graph.getNeighbours(vertex);
                    var min = Infinity;
                    var closest = -1;

                    candidates.forEach(candidate => {
                        if (neighbours.includes(candidate)) {
                            var weight = graph.getEdgeWeight(vertex, candidate)
                            if (weight < min) {
                                min = weight;
                                closest = candidate;
                            }
                        }
                    })

                    return closest;
                }

                function getNextLayer(layer) {
                    var nextLayer = [];
                    layer.forEach(function (vertex) {
                        graph.getNeighbours(vertex).forEach(function (neighbour) {
                            if (!explored[neighbour]) {
                                nextLayer.push(neighbour);
                                explored[neighbour] = true;
                            }
                        })
                    })
                    return nextLayer;
                }
            }
            getDistanceMatrix() {
                /**
                 * Returns a matrix M of distances between two Nodes a and b,
                 * based on the Floyd-Warshall Algorithm.
                 * 
                 * M[a][b] returns the distance between a and b. The distance
                 * is the length of the shortest path between a and b.
                 */

                var graph = this.graph;

                var distances = new Array(graph.nodeCount);
                for (var i = 0; i < graph.nodeCount; i++) {
                    var neighbours = graph.getNeighbours(i);
                    distances[i] = new Array(graph.nodeCount).fill(Infinity);
                    distances[i][i] = 0;
                    // TODO: Add adjacency matrix to graph.
                    neighbours.forEach(neighbour => {
                        distances[i][neighbour] = graph.getEdgeWeight(i, neighbour);
                    })
                }

                for (var n = 0; n < graph.nodeCount; n++) {
                    for (var i = 0; i < graph.nodeCount; i++) {
                        for (var j = 0; j < graph.nodeCount; j++) {
                            var directPath = distances[i][j];
                            var indirectPath = distances[i][n] + distances[n][j];
                            if (directPath > indirectPath)
                                distances[i][j] = indirectPath;
                        }
                    }
                }

                return distances;
            }
        }

        GraphSearcher.prototype.dfs = GraphSearcher.prototype.depthFirstSearch;
        GraphSearcher.prototype.bfs = GraphSearcher.prototype.breadthFirstSearch;

        function retrievePathTo(searchNode) {
            if (searchNode.parent === null)
                return [searchNode.id];
            else
                return [searchNode.id].concat(retrievePathTo(searchNode.parent));
        }

        function SimpleSearchNode(vertex, parentNode) {
            this.id = vertex;
            this.parent = parentNode;
        }

        function PriorityQueue() {
            var dataStore = [];

            this.size = function size() {
                return dataStore.length;
            }

            this.poll = function poll() {
                if (this.size() === 0)
                    return null;

                return dataStore.pop().data;
            }

            this.peek = function peek() {
                if (this.size() === 0)
                    return null;

                return dataStore.peek().data;
            }

            this.add = function add(data, priority) {
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

            function PriorityNode(data, priority) {
                this.data = data;
                this.priority = priority;
            }
        }

        return GraphSearcher
    })()

    function isSubGraph(graph, anotherGraph) {
    }
    function isDisjoint(graph, anotherGraph) {
    }

    kgraph.Vertex = Vertex
    kgraph.Edge = Edge
    kgraph.GraphFactory = new GraphFactory()
    kgraph.GraphSearcher = GraphSearcher

    return kgraph;
})(window.kgraph = window.kgraph || {}, jQuery)