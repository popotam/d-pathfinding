#!/usr/bin/rdmd

import std.algorithm;
import std.array;
import std.c.time;
import std.math;
import std.stdio;
import std.string;
import std.typetuple;


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

}

enum DIRECTIONS = [XYZ(1, 0, 0), XYZ(-1, 0, 0),
                   XYZ(0, 1, 0), XYZ(0, -1, 0)];
enum NOT_PASSABLE = 0;
enum NON_PASSABLE_VERTICES = [XYZ(8, 0, 0), XYZ(8, 1, 0), XYZ(9, 0, 0)];

alias Vertex[XYZ] Graph;


long dist_heuristic(Vertex v1, Vertex v2) {
    return (abs(v1.xyz.x - v2.xyz.x) +
            abs(v1.xyz.y - v2.xyz.y) +
            abs(v1.xyz.z - v2.xyz.z));
}


Vertex[] find_nearest(Vertex src, Vertex dst) {
    long start_time = time(null);
    Vertex[] path;
    PathCosts[XYZ] costs;
    PathVertex[] opened_ordered;
    bool[Vertex] opened, closed;
    long g, h;

    if (src == dst) {
        return path;
    }

    h = dist_heuristic(src, dst);
    costs[src.xyz] = PathCosts(0, h, null);
    auto vertex = src;
    auto path_vert = PathVertex(h, 0, vertex);
    opened_ordered ~= path_vert;
    while (opened_ordered.length) {
        path_vert = opened_ordered[0];
        opened_ordered.popFront();
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
                auto old_costs = costs[neighbour.xyz];
                if (cost < old_costs.g) {
                    // update cost
                    costs[neighbour.xyz] = PathCosts(
                            cost, old_costs.h, vertex);
                    opened_ordered ~=
                            PathVertex(cost + old_costs.h, cost, neighbour);
                    sort!("a.f < b.f", SwapStrategy.stable)(opened_ordered);
                }
            } else {
                // add field to opened list
                h = dist_heuristic(vertex, neighbour);
                costs[neighbour.xyz] = PathCosts(cost, h, vertex);
                opened_ordered ~= PathVertex(cost + h, cost, neighbour);
                sort!("a.f < b.f", SwapStrategy.stable)(opened_ordered);
                opened[neighbour] = true;
            }
        }
    }
    auto xyz = dst.xyz;
    auto path_cost = costs[xyz];
    while (path_cost.parent) {
        path ~= path_cost.parent;
        path_cost = costs[path_cost.parent.xyz];
    }
    auto calculation_time = time(null) - start_time;
    writefln("astar %s %s->%s lenght=%s closed=%s total_cost=%s heur=%s",
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
            graph[xyz] = new Vertex(xyz);
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
    // do some search
    foreach (vertex; find_nearest(graph[XYZ(1, 1, 0)], graph[XYZ(8, 8, 0)])) {
        writeln(vertex);
    }
}