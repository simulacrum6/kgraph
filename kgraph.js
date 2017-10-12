(function (kgraph, $, undefined) {
    kgraph.Vertex = function Vertex(id, label = id) {
        this.id = id;
        this.label = label;
    }
    kgraph.Vertex.prototype.toString = function () {
        return '{id: ' + this.id + ', label: ' + this.label + '}'
    }

    kgraph.Edge = function (from, to, weight = 1) {
        this.from = from;
        this.to = to;
        this.weight = weight;
    }
    kgraph.Edge.prototype.toString = function () {
        return '{from: ' + this.from + ', to: ' + this.to + ', weight: ' + this.weight.toFixed(2) + '}'
    }
    
    kgraph.GraphFactory = function GraphFactory() {
        this.createGraph = function (buildInstruction) {
            var graph = new Graph(buildInstruction.vertexList);
            graph.addEdge = buildInstruction.addEdgeFunction;
            graph.show = buildInstruction.showFunction;
            
            buildInstruction.edges = buildInstruction.edges || [];
            buildInstruction.edges.forEach(function (edge) {
                graph.addEdge(edge.from, edge.to, edge.weight);
            });
            return graph;
        }
        this.createUndirectedGraph = function createUndirectedGraph(vertexList, edges = []) {
            var buildInstructions = new GraphBuildInstruction(vertexList, edges, addUndirectedEdge, showUndirected);
            return this.createGraph(buildInstructions);
        }
        this.createUndirectedWeightedGraph = function createUndirectedWeightedGraph(vertexList, edges = []) {
            var buildInstructions = new GraphBuildInstruction(vertexList, edges, addUndirectedEdge, showUndirectedWeighted);

            return this.createGraph(buildInstructions);
        }
        this.createDirectedGraph = function createDirectedGraph(vertexList, edges = []) {
            var buildInstructions = new GraphBuildInstruction(vertexList, edges, addDirectedEdge, showDirected);

            return this.createGraph(buildInstructions);
        }
        this.createDirectedWeightedGraph = function createDirectedWeightedGraph(vertexList, edges = []) {
            var buildInstructions = new GraphBuildInstruction(vertexList, edges, addDirectedEdge, showDirectedWeighted);
            return this.createGraph(buildInstructions);
        }

        function addDirectedEdge(from, to, weight = 1) {
            this.edgeList[from].push(to);
            this.edgeWeights[from].push(weight);
            this.edgeCount++;
        }
        function addUndirectedEdge(vertex1, vertex2, weight = 1) {
            this.edgeList[vertex1].push(vertex2);
            this.edgeWeights[vertex1].push(weight);
            this.edgeList[vertex2].push(vertex1);
            this.edgeWeights[vertex2].push(weight);
            this.edgeCount++;
        }
        function showDirected() {
            for (var i = 0; i < this.vertices; i++)
                console.log(i + " -> " + this.edgeList[i] + " [" + this.edgeList[i].length + "]");
        }
        function showDirectedWeighted() {
            for (var i = 0; i < this.vertices; i++)
                console.log(i + " -" + this.edgeWeights[i].map(x => x.toFixed(2)) + "-> " + this.edgeList[i] + " [" + this.edgeList[i].length + "]");
        }
        function showUndirected() {
            for (var i = 0; i < this.vertices; i++)
                console.log(i + " <-> " + this.edgeList[i] + " [" + this.edgeList[i].length + "]");
        }
        function showUndirectedWeighted() {
            for (var i = 0; i < this.vertices; i++)
                console.log(i + " <-" + this.edgeWeights[i].map(x => x.toFixed(2)) + "-> " + this.edgeList[i] + " [" + this.edgeList[i].length + "]");
        }

        function Graph(vertexList) {
            this.vertexList = vertexList;
            this.vertices = vertexList.length;
            this.edgeCount = 0;
            this.edgeList = [];
            this.edgeWeights = [];

            for (var i = 0; i < this.vertices; i++) {
                this.edgeList[i] = [];
                this.edgeWeights[i] = [];
            }

            this.addEdge = function(){};
            this.show = function(){};
            this.contains = function contains(vertex) {
                if (typeof vertex === 'number')
                    return this.vertexList[vertex] ? true : false
                if (vertex instanceof Array)
                    return this.containsAll(vertex)
                return false
            }
            this.containsAll = function containsAll(vertices) {
                return vertices.every(vertex => this.contains(vertex))
            }
            this.getVertex = function getVertex(vertex) {
                return this.vertexList[vertex];
            }
            this.getNeighbours = function getNeighbours(vertex) {
                return this.edgeList[vertex];
            }
            this.getEdgeWeight = function getEdgeWeight(from, to) {
                if (from == to) return 0;
                var index = this.edgeList[from].indexOf(to);
                return this.edgeWeights[from][index];
            }
        }

        function GraphBuildInstruction(vertexList, edges, addEdgeFunction, showFunction) {
            this.vertexList = vertexList;
            this.addEdgeFunction = addEdgeFunction;
            this.showFunction = showFunction;
            this.edges = edges;
        }
    }
    kgraph.GraphSearcher = function GraphSearcher(graph) {
        this.graph = graph;

        this.depthFirstSearch = function depthFirstSearch(start, target) {
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
        this.dfs = this.depthFirstSearch;

        this.breadthFirstSearch = function breadthFirstSearch(start, target) {
            var startTime = window.performance.now();
            var explored = {};
            var queue = [new SimpleSearchNode(start, null)];

            while (queue.length > 0) {
                var candidate = queue.shift();
                var neighbours = this.graph.getNeighbours(candidate.id);

                if (neighbours !== undefined) {
                    if (neighbours.includes(target)) {
                        console.log("BFS Elapsed Time:" + (window.performance.now() - startTime));
                        return retrievePathTo(new SimpleSearchNode(target, candidate));
                    }
                    neighbours.forEach(visitNeighbour);
                }

            }

            return []

            function visitNeighbour(neighbour) {
                if (!explored[neighbour]) {
                    explored[neighbour] = true;
                    queue.push(new SimpleSearchNode(neighbour, candidate));
                }
            }
        }
        this.bfs = this.breadthFirstSearch;

        this.getPath = function getPath(start, target, directed) {
            directed = directed === undefined ? true : directed;
            var path = this.bfs(start, target)
            if (path.length === 0 && directed === false) {
                path = this.bfs(target, start)
            }
            return path;
        }

        this.aStarSearch = function aStarSearch(start, target, heuristicFunction, costFunction) {
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
    }
    return kgraph;
})(window.kgraph = window.kgraph || {}, jQuery)