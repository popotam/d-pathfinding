#!/usr/bin/rdmd

import core.time;
import std.c.stdlib: exit;
import std.container;
import std.json;
import std.file;
import std.math;
import std.stdio;
import std.string;

enum DIRECTIONS = [XYZ(1, 0, 0), XYZ(-1, 0, 0),
                   XYZ(0, 1, 0), XYZ(0, -1, 0)];
enum NOT_PASSABLE = 0;
enum NON_PASSABLE_VERTICES = [XYZ(8, 0, 0), XYZ(8, 1, 0), XYZ(9, 0, 0)];

alias Vertex[XYZ] Graph;


immutable struct XYZ {

    long x, y, z;

    string toString() {
        return format("XYZ(%s, %s, %s)", x, y, z);
    }

}


class Vertex {

    XYZ xyz;
    Connection[] connections;

    @property long x() { return xyz.x; }
    @property long y() { return xyz.y; }
    @property long z() { return xyz.z; }

    this(XYZ xyz) {
        this.xyz = xyz;
    }

    string toString() {
        return format("V[%s,%s,%s]", x, y, z);
    }

}


struct Connection {

    long cost;
    Vertex destination;

    this(long cost, Vertex destination) {
        this.cost = cost;
        this.destination = destination;
    }

    string toString() {
        return format("C(%s, %s)", cost, destination);
    }

}


struct PathCosts {

    long g, h;
    Vertex parent;

    @property long f() {
        return g + h;
    }

}


struct PathVertex {

    long f, g;
    Vertex vertex;

    @property long h() {
        return f - g;
    }

    string toString() {
        return format("P%s", vertex);
    }

    int opCmp(PathVertex other) {
        if (this.f == other.f) {
            return 0;
        } else if (this.f > other.f) {
            return 1;
        } else {
            return -1;
        }
    }
}


float calculationTime(TickDuration start_time) {
    return (TickDuration.currSystemTick() - start_time).msecs / 1000.0;
}


long distanceHeuristic(Vertex v1, Vertex v2) {
    return (abs(v1.xyz.x - v2.xyz.x) +
            abs(v1.xyz.y - v2.xyz.y) +
            abs(v1.xyz.z - v2.xyz.z));
}


Vertex[] findPath(Vertex src, Vertex dst) {
    auto start_time = TickDuration.currSystemTick();
    Vertex[] path;
    PathCosts[Vertex] costs;
    bool[Vertex] opened, closed;
    long g, h;

    if (src == dst) {
        return path;
    }

    h = distanceHeuristic(src, dst);
    costs[src] = PathCosts(0, h, null);
    auto vertex = src;
    auto path_vert = PathVertex(h, 0, vertex);
    auto queue = new RedBlackTree!(PathVertex, "a.f < b.f", true)(path_vert);

    while (!queue.empty()) {
        path_vert = queue.front();
        queue.removeFront();
        vertex = path_vert.vertex;
        if (vertex in closed) {
            continue;
        }
        if (vertex == dst) {
            break;
        }
        // move field to closed
        opened.remove(vertex);
        closed[vertex] = true;
        // check every neighbouring fields
        foreach (ref connection; vertex.connections) {
            auto cost = connection.cost;
            auto neighbour = connection.destination;
            if (cost == NOT_PASSABLE || neighbour in closed) {
                continue;
            }
            cost += path_vert.g;
            if (neighbour in opened) {
                auto old_costs = costs[neighbour];
                if (cost < old_costs.g) {
                    // update cost
                    costs[neighbour] = PathCosts(
                            cost, old_costs.h, vertex);
                    queue.insert(PathVertex(
                            cost + old_costs.h, cost, neighbour));
                }
            } else {
                // add field to opened list
                h = distanceHeuristic(vertex, neighbour);
                costs[neighbour] = PathCosts(cost, h, vertex);
                queue.insert(PathVertex(cost + h, cost, neighbour));
                opened[neighbour] = true;
            }
        }
    }
    auto path_cost = costs[dst];
    while (path_cost.parent) {
        path ~= path_cost.parent;
        path_cost = costs[path_cost.parent];
    }
    writefln("astar %.3f %s->%s lenght=%s closed=%s total_cost=%s heur=%s",
             calculationTime(start_time), src.xyz, dst.xyz, path.length,
             closed.length, costs[dst].g, costs[src].h);
    return path;
}


Graph createSimpleGraph(uint size_x, uint size_y) {
    auto start_time = TickDuration.currSystemTick();
    Graph graph;
    // create some vertices
    writeln("Create some vertices");
    foreach (x; 0..size_x) {
        foreach (y; 0..size_y) {
            auto xyz = XYZ(x, y, 0);
            graph[xyz] = new Vertex(xyz);
        }
    }
    // create connections
    writeln("Create connections");
    foreach (key, ref vertex; graph) {
        foreach (direction; DIRECTIONS) {
            auto xyz = XYZ(vertex.x + direction.x,
                           vertex.y + direction.y, 0);
            if (xyz in graph) {
                vertex.connections ~= Connection(1, graph[xyz]);
            }
        }
    }
    // create some non-passable vertices
    writeln("Create some non-passable vertices");
    foreach (xyz; NON_PASSABLE_VERTICES) {
        foreach (connection; graph[xyz].connections) {
            connection.cost = 0;
            foreach (other_conn; connection.destination.connections) {
                if (other_conn.destination.xyz == xyz) {
                    other_conn.cost = 0;
                }
            }
        }
    }
    writefln("Graph was set up in %.3f s", calculationTime(start_time));
    return graph;
}


Graph createGraphFromJSON(string json) {
    Graph graph;

    Vertex getOrCreateVertex(JSONValue json_node) {
        Vertex vertex;
        auto xyz = XYZ(json_node.array[0].integer,
                       json_node.array[1].integer,
                       json_node.array[2].integer);
        if (xyz in graph) {
            vertex = graph[xyz];
        } else {
            vertex = new Vertex(xyz);
            graph[xyz] = vertex;
        }
        return vertex;
    }

    class GraphFromJSONException : Exception {
        this(string msg) {
            super(msg);
        }
    }

    auto root_node = parseJSON(json);
    // validate
    if (root_node.type !is JSON_TYPE.OBJECT ||
        !("graph" in root_node.object)) {
        throw new GraphFromJSONException("Wrong root node formatting");
    }
    auto graph_node = root_node.object["graph"];
    if (graph_node.type !is JSON_TYPE.ARRAY) {
        throw new GraphFromJSONException("Wrong graph node formatting");
    }

    // process graph
    foreach (vertex_node; graph_node.array) {
        // validate
        if (vertex_node.type !is JSON_TYPE.ARRAY ||
            vertex_node.array.length != 4 ||
            vertex_node.array[0].type !is JSON_TYPE.INTEGER ||
            vertex_node.array[1].type !is JSON_TYPE.INTEGER ||
            vertex_node.array[2].type !is JSON_TYPE.INTEGER ||
            vertex_node.array[3].type !is JSON_TYPE.ARRAY) {
            throw new GraphFromJSONException("Wrong vertex formatting");
        }
        // process
        auto vertex = getOrCreateVertex(vertex_node);
        foreach (connection_node; vertex_node.array[3].array) {
            // validate
            if (connection_node.type !is JSON_TYPE.ARRAY) {
                throw new GraphFromJSONException("Wrong connection node type");
            }
            foreach (node; connection_node.array) {
                if (node.type !is JSON_TYPE.INTEGER) {
                    throw new GraphFromJSONException(format(
                            "Wrong connection node element type %s",
                            node.type));
                }
            }
            // process
            auto destination = getOrCreateVertex(connection_node);
            long cost = connection_node.array[3].integer;
            vertex.connections ~= Connection(cost, destination);
        }
    }
    return graph;
}


Vertex[] getSampleFromJSON(Graph graph, string json) {
    Vertex[] sample = [];

    class SampleFromJSONException : Exception {
        this(string msg) {
            super(msg);
        }
    }

    Vertex getVertex(JSONValue json_node) {
        Vertex vertex;
        auto xyz = XYZ(json_node.array[0].integer,
                       json_node.array[1].integer,
                       json_node.array[2].integer);
        if (!(xyz in graph)) {
            throw new SampleFromJSONException("Vertex not present in graph");
        }
        return graph[xyz];
    }

    auto root_node = parseJSON(json);
    // validate
    if (root_node.type !is JSON_TYPE.OBJECT ||
        !("sample" in root_node.object)) {
        throw new SampleFromJSONException("Wrong root node formatting");
    }
    auto sample_node = root_node.object["sample"];
    if (sample_node.type !is JSON_TYPE.ARRAY) {
        throw new SampleFromJSONException("Wrong sample node formatting");
    }

    // process sample
    foreach (vertex_node; sample_node.array) {
        // validate
        if (vertex_node.type !is JSON_TYPE.ARRAY ||
            vertex_node.array.length != 3 ||
            vertex_node.array[0].type !is JSON_TYPE.INTEGER ||
            vertex_node.array[1].type !is JSON_TYPE.INTEGER ||
            vertex_node.array[2].type !is JSON_TYPE.INTEGER) {
            throw new SampleFromJSONException("Wrong sample formatting");
        }
        // process
        sample ~= getVertex(vertex_node);
    }
    return sample;
}


void measureFindPath(Vertex[] sample) {
    auto start_time = TickDuration.currSystemTick();
    foreach (src; sample) {
        foreach (dst; sample) {
            findPath(src, dst);
        }
    }
    writefln("All sample paths found in %.3f", calculationTime(start_time));
}


unittest {
    writeln("START UNITTESTS");
    enum SizeX = 100;
    enum SizeY = 10;
    auto sample_graph = createSimpleGraph(SizeX, SizeY);
    auto src = sample_graph[XYZ(1, 1, 0)];
    auto dst = sample_graph[XYZ(SizeX - 2, SizeY - 2, 0)];
    auto sample_path = findPath(src, dst);
    enum TEST_JSON_GRAPH = "{
        \"sample\": [
            [0,0,0],
            [1,0,0],
            [0,1,0],
            [1,1,0]
        ],
        \"graph\": [
            [0,0,0, [
                [1,0,0, 1],
                [0,1,0, 1]
            ]],
            [1,0,0, [
                [0,0,0, 1],
                [1,1,0, 1]
            ]],
            [0,1,0, [
                [0,0,0, 1],
                [1,1,0, 1]
            ]],
            [1,1,0, [
                [1,0,0, 1],
                [0,1,0, 1]
            ]]
        ]
    }";
    auto json_graph = createGraphFromJSON(TEST_JSON_GRAPH);
    auto json_sample = getSampleFromJSON(json_graph, TEST_JSON_GRAPH);
    auto json_path = findPath(json_graph[XYZ(0, 0, 0)],
                              json_graph[XYZ(1, 1, 0)]);
    auto json_path2 = findPath(json_sample[0],
                              json_sample[json_sample.length - 1]);
    writeln("END UNITTESTS");
}


void main(string[] args) {
    if (args.length < 2) {
        writefln("Usage: %s <graph.json>", args[0]);
        exit(1);
    }
    auto json = readText(args[1]);
    auto graph = createGraphFromJSON(json);
    auto sample = getSampleFromJSON(graph, json);
    measureFindPath(sample);
}
