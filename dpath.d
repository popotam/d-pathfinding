#!/usr/bin/rdmd

import std.algorithm;
import std.array;
import std.c.time;
import std.math;
import std.stdio;
import std.string;
import std.typetuple;


struct Connection {

    uint cost;
    Vertex* destination;

    this(uint cost, ref Vertex destination) {
        this.cost = cost;
        this.destination = &destination;
    }

    string toString() {
        return format("C(%s, %s)", cost, *destination);
    }

}


immutable struct XYZ {

    int x, y, z;

    string toString() {
        return format("XYZ(%s, %s, %s)", x, y, z);
    }

}


struct Vertex {

    XYZ xyz;
    Connection[] connections;

    @property uint x() { return xyz.x; }
    @property uint y() { return xyz.y; }
    @property uint z() { return xyz.z; }
    
    string toString() {
        return format("V[%s,%s,%s]->%s", x, y, z, connections.length);
    }

}


struct PathCosts {

    uint g, h;
    Vertex* parent = null;
    
    @property uint f() {
        return g + h;
    }

}

struct PathVertex {

    uint f, g;
    Vertex* vertex;
    
    @property uint h() {
        return f - g;
    }

    string toString() {
        return format("P%s", *vertex);
    }

}

enum DIRECTIONS = [XYZ(1, 0, 0), XYZ(-1, 0, 0),
                   XYZ(0, 1, 0), XYZ(0, -1, 0)];
enum NOT_PASSABLE = 0;
enum NON_PASSABLE_VERTICES = [XYZ(8, 0, 0), XYZ(8, 1, 0), XYZ(9, 0, 0)];

alias Vertex[XYZ] Graph;


uint dist_heuristic(ref Vertex v1, ref Vertex v2) {
    return abs(v1.x - v2.x) + abs(v1.y - v2.y) + abs(v1.z - v2.z);
}


Vertex[] find_nearest(ref Vertex src, ref Vertex dst) {
    uint start_time = time(null);
    Vertex[] path;
    PathCosts[XYZ] costs;
    PathVertex[] opened_ordered;
    bool[Vertex] opened, closed;
    uint g, h;
    
    if (src == dst) {
        return path;
    }

    h = dist_heuristic(src, dst);
    costs[src.xyz] = PathCosts(0, h, null);
    opened_ordered ~= PathVertex(h, 0, &src);
    
    while (opened_ordered.length) {
        auto path_vert = opened_ordered[0];
        auto vertex = *path_vert.vertex;
        opened_ordered = opened_ordered[1..opened_ordered.length];//.popFront();
        writeln("---new---");
        writeln("VERTEX: ", vertex);
        writeln("OO:", opened_ordered);
        if (vertex in closed) {
            writeln("continue");
            continue;
        }
        if (vertex == dst) {
            writeln("break");
            break;
        }
        // move field to closed
        opened.remove(vertex);
        closed[vertex] = true;
        writeln("OPENED: ", opened);
        writeln("CLOSED:", closed);
        // check every neighbouring fields
        foreach (ref connection; vertex.connections) {
            auto cost = connection.cost;
            auto neighbour = *connection.destination;
            if (cost == NOT_PASSABLE || neighbour in closed) {
                continue;
            }
            cost += path_vert.g;
            if (neighbour in opened) {
                auto old_costs = costs[neighbour.xyz];
                if (cost < old_costs.g) {
                    // update cost
                    costs[neighbour.xyz] = PathCosts(
                            cost, old_costs.h, &vertex);
                    opened_ordered ~=
                            PathVertex(cost + old_costs.h, cost, &neighbour);
                    sort!("a.f < b.f", //(a, b) { return a.f < b.f; }
                          SwapStrategy.stable)(opened_ordered);
                }
            } else {
                // add field to opened list
                h = dist_heuristic(vertex, neighbour);
                costs[neighbour.xyz] = PathCosts(cost, h, &vertex);
                opened_ordered ~=
                        PathVertex(cost + h, cost, &neighbour);
                sort!("a.f < b.f", //(a, b) { return a.f < b.f; }
                      SwapStrategy.stable)(opened_ordered);
                opened[neighbour] = true;
            }
        }
        writeln("---foreach---");
        writeln("OPENED: ", opened);
        writeln("CLOSED:", closed);
    }
    auto xyz = dst.xyz;
    auto path_cost = costs[xyz];
    while (path_cost.parent) {
        path ~= *path_cost.parent;
        path_cost = costs[(*path_cost.parent).xyz];
    }
    auto calculation_time = time(null) - start_time;
    writefln("astar %.3f %s->%s lenght=%i closed=%i total_cost=%i heur=%i",
             calculation_time, src.xyz, dst.xyz, path.length,
             closed.length, costs[dst.xyz].g, costs[src.xyz].h);
    return path;
}


void main() {
    Graph graph;
    // create some vertices
    foreach (x; 0..10) {
        foreach (y; 0..10) {
            auto xyz = XYZ(x, y, 0);
            graph[xyz] = Vertex(xyz);
        }
    }
    // create connections
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

    foreach (vertex; find_nearest(graph[XYZ(1, 1, 0)], graph[XYZ(8, 8, 0)])) {
        writeln(vertex);
    }
    
    // print some
    //foreach (xyz, ref vertex; graph) {
    //    writefln("%s: %s", xyz, vertex);
    //}
}
