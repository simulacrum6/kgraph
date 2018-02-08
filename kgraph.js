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

    const GraphTypes = {
        SIMPLE: 1,
        DIRECTED_SIMPLE: 2,
        WEIGHTED: 3,
        DIRECTED_WEIGHTED: 4
    };

    var GraphFactory = (function () {
        class GraphFactory {
            constructor() { }

            createUndirectedGraph(nodes = [], edges = []) {
                return this._createGraph(nodes, edges, GraphTypes.SIMPLE);
            }
            createUndirectedWeightedGraph(nodes = [], edges = []) {
                return this._createGraph(nodes, edges, GraphTypes.WEIGHTED);
            }
            createDirectedGraph(nodes = [], edges = []) {
                return this._createGraph(nodes, edges, GraphTypes.DIRECTED_SIMPLE);
            }
            createDirectedWeightedGraph(nodes = [], edges = []) {
                return this._createGraph(nodes, edges, GraphTypes.DIRECTED_WEIGHTED);
            }
            _createGraph(nodes, edges, graphType) {
                switch (graphType) {
                    case GraphTypes.SIMPLE:
                        return new SimpleGraph(nodes, edges);
                    case GraphTypes.DIRECTED_SIMPLE:
                        return new DirectedSimpleGraph(nodes, edges);
                    case GraphTypes.WEIGHTED:
                        return new WeightedGraph(nodes, edges);
                    case GraphTypes.DIRECTED_WEIGHTED:
                        return new DirectedWeightedGraph(nodes, edges);
                }
            }
        }

        class Graph {
            constructor(nodes) {
                this.nodes = [];
                this.edges = [];
                this.edgeCount = 0;
                this.neighbours = [];
                this.edgeWeights = [];
                this.adjacencyMatrix = [];
                nodes.forEach(node => this.addNode(node));
            }

            addNode(object) {
                this.nodes.push(object);
                this.neighbours.push([]);
                this.edgeWeights.push([]);
                this.adjacencyMatrix.forEach(row => row.push(Infinity));
                this.adjacencyMatrix.push(new Array(this.size()).fill(Infinity));
                this.adjacencyMatrix[this.size() - 1][this.size() - 1] = 0;
            }
            size() {
                return this.nodes.length;
            }
            averageDegree() {
                var degreeSum = 0;
    
                for (var node = 0; node < this.size(); node++)
                    degreeSum += this.degree(node);

                return degreeSum / this.size();
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
            getNode(id) {
                return this.nodes[id];
            }
            getNeighbours(id) {
                return this.neighbours[id];
            }
            neighbourOf(node, neighbour) {
                if (node === neighbour)
                    return false;
                return this.adjacencyMatrix[node][neighbour] < Infinity;
            }
            isIsolated(id) {
                return this.degree(id) === 0;
            }
            getEdgeWeight(from, to) {
                return this.adjacencyMatrix[from][to];
            }
            getAdjacencyList() {
                return this.adjacencyMatrix;
            }
            getNodes() {
                return this.nodes;
            }
            getEdgeList() {
                return this.edges;
            }
            _addEdge(from, to, weight, undirected) {
                this.neighbours[from].push(to);
                this.edgeWeights[from].push(weight);
                this.adjacencyMatrix[from][to] = weight;
                this.edgeCount++;
                this.edges.push(new Edge(from, to, weight))
                if (undirected) {
                    this.edgeCount--;
                    this._addEdge(to, from, weight, false);
                }
            }
            _centralisation(normalised, directed) {
                /**
                 * Returns the centralisation of the graph.
                 * 
                 * Centralisation is a network measure, describing how unevenly 
                 * degrees are distributed between high degree and low degree 
                 * nodes. A high centralisation (close to 1), indicates that the 
                 * difference between all nodes and the ones with the highest
                 * centrality is high.
                 * 
                 */
                var centralisation;
                var degreeSum = 0;
                var maxDegree = 0;
                // FIXME: Not true for weighted graphs.
                var maxCentralisation = (this.size() - 1) * (this.size() - 2);
                if (directed) {
                    maxCentralisation *= 2;
                }

                // TODO: write function for max/min degree
                for (var node = 0; node < this.size(); node++) {
                    var degree = this.degree(node)
                    if (degree > maxDegree)
                        maxDegree = degree;
                    degreeSum += degree;
                }

                centralisation = (maxDegree * this.size()) - degreeSum;

                if (normalised)
                    centralisation /= maxCentralisation;

                return centralisation;
            }
            _degree(id, directed) {
                var neighbours = this.getNeighbours(id);
                var degree = neighbours.length;

                if (directed) {
                    neighbours.forEach(neighbour => {
                        if (this.neighbourOf(neighbour, node))
                            degree++;
                    })
                }
                return degree;
            }
            _show(directed, weighted) {
                for (var node = 0; node < this.size(); node++) {
                    for (var neighbour = 0; neighbour < this.size(); neighbour++) {
                        var weight = this.getEdgeWeight(node, neighbour);
                        if (0 < weight && weight < Infinity)
                            console.log(showString(node, weight.toFixed(2), neighbour, directed, weighted));
                    }
                }

                function showString(node, weight, neighbour, directed, weighted) {
                    var leftArrow = '<'
                    if (!weighted)
                        weight = '';
                    if (directed)
                        leftArrow = '';
                    return `${node} ${leftArrow}-${weight}-> ${neighbour}`
                }
            }
        }

        class SimpleGraph extends Graph {
            constructor(nodes = [], edges = []) {
                super(nodes);
                edges.forEach(edge => {
                    this.addEdge(edge.from, edge.to);
                });
            }

            addEdge(from, to) {
                this._addEdge(from, to, 1, true);
            }
            centralisation(normalised = true) {
                return this._centralisation(normalised, false);
            }
            degree(node) {
                return this._degree(node, false);
            }
            show() {
                this._show(false, false);
            }
        }

        class DirectedSimpleGraph extends Graph {
            constructor(nodes = [], edges = []) {
                super(nodes);
                edges.forEach(edge => {
                    this.addEdge(edge.from, edge.to);
                })
            }

            addEdge(from, to) {
                this._addEdge(from, to, 1, false);
            }
            centralisation(normalised = true) {
                return this._centralisation(normalised, false);
            }
            degree(node) {
                return this._degree(node, true);
            }
            show() {
                return this._show(true, false);
            }
        }

        class WeightedGraph extends Graph {
            constructor(nodes = [], edges = []) {
                super(nodes);
                edges.forEach(edge => {
                    this.addEdge(edge.from, edge.to, edge.weight);
                })
            }

            addEdge(from, to, weight) {
                this._addEdge(from, to, weight, true);
            }
            centralisation(normalised = true) {
                return this._centralisation(normalised, false);
            }
            degree(node) {
                return this._degree(node, false);
            }
            show() {
                return this._show(false, true);
            }
        }

        class DirectedWeightedGraph extends Graph {
            constructor(nodes = [], edges = []) {
                super(nodes);
                edges.forEach(edge => {
                    this.addEdge(edge.from, edge.to, edge.weight);
                })
            }

            addEdge(from, to, weight) {
                this._addEdge(from, to, weight, false);
            }
            centralisation(normalised = true) {
                return this._centralisation(normalised, true);
            }
            degree(node) {
                return this._degree(node, true);
            }
            show() {
                return this._show(true, true);
            }
        }

        return GraphFactory
    })()


    var GraphSearcher = (function () {
        class GraphSearcher {
            constructor(graph) {
                this.graph = graph;
                this.distanceMatrix;
                this.pathNeighbour;
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
                    return heuristicFunction(graph.getNode(vertex), graph.getNode(target));
                };
                var costs = function (vertex, anotherVertex) {
                    return costFunction(graph.getNode(vertex), graph.getNode(anotherVertex));
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
                var graph = this.graph;
                var pathDistance = this.getDistanceMatrix();

                var pathNeighbour = new Array(graph.size());
                for (var n = 0; n < graph.size(); n++) {
                    pathNeighbour[n] = new Array(graph.size()).fill(null);
                }

                for (var start = 0; start < graph.size(); start++) {
                    for (var target = 0; target < graph.size(); target++) {
                        if (pathDistance[start][target] != Infinity && start !== target) {
                            var neighbours = graph.getNeighbours(start);

                            var closest = null;
                            var min = Infinity;

                            neighbours.forEach(neighbour => {
                                if (pathDistance[neighbour][target] < min) {
                                    min = pathDistance[neighbour][target];
                                    closest = neighbour;
                                }
                            })

                            pathNeighbour[start][target] = closest;
                        }
                    }
                }
                this.pathNeighbour = pathNeighbour;
                return pathNeighbour;
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

                var distances = graph.getAdjacencyList();

                for (var n = 0; n < graph.size(); n++) {
                    for (var i = 0; i < graph.size(); i++) {
                        for (var j = 0; j < graph.size(); j++) {
                            var directPath = distances[i][j];
                            var indirectPath = distances[i][n] + distances[n][j];
                            if (directPath > indirectPath)
                                distances[i][j] = indirectPath;
                        }
                    }
                }
                this.distanceMatrix = distances;
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