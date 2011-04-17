#!/usr/bin/rdmd

import core.time;
import std.math;
import std.range;
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


auto trisectInsert(alias pred = "a < b", T)(ref T[] array, T element) {
    auto r = assumeSorted!(pred)(array).trisect(element);
    auto index = r[0].length;
    array = (array[0 .. index] ~ element ~ array[index .. array.length]);
    return index;
}


long dist_heuristic(Vertex v1, Vertex v2) {
    return (abs(v1.xyz.x - v2.xyz.x) +
            abs(v1.xyz.y - v2.xyz.y) +
            abs(v1.xyz.z - v2.xyz.z));
}


Vertex[] find_nearest(Vertex src, Vertex dst) {
    auto start_time = TickDuration.currSystemTick();
    Vertex[] path;
    PathCosts[Vertex] costs;
    PathVertex[] queue;
    bool[Vertex] opened, closed;
    long g, h;

    if (src == dst) {
        return path;
    }

    h = dist_heuristic(src, dst);
    costs[src] = PathCosts(0, h, null);
    auto vertex = src;
    auto path_vert = PathVertex(h, 0, vertex);
    queue ~= path_vert;
    while (queue.length) {
        path_vert = queue[0];
        queue.popFront();
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
                    auto npv = PathVertex(cost + old_costs.h, cost, neighbour);
                    trisectInsert(queue, npv);
                }
            } else {
                // add field to opened list
                h = dist_heuristic(vertex, neighbour);
                costs[neighbour] = PathCosts(cost, h, vertex);
                auto npv = PathVertex(cost + h, cost, neighbour);
                trisectInsert(queue, npv);
                opened[neighbour] = true;
            }
        }
    }
    auto path_cost = costs[dst];
    while (path_cost.parent) {
        path ~= path_cost.parent;
        path_cost = costs[path_cost.parent];
    }
    auto calculation_time = (TickDuration.currSystemTick() -
                             start_time).msecs / 1000.0;
    writefln("astar %.3f %s->%s lenght=%s closed=%s total_cost=%s heur=%s",
             calculation_time, src.xyz, dst.xyz, path.length,
             closed.length, costs[dst].g, costs[src].h);
    return path;
}


void main() {
    auto start_time = TickDuration.currSystemTick();

    Graph graph;
    // create some vertices
    writeln("Create some vertices");
    foreach (x; 0..1000) {
        foreach (y; 0..100) {
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

    writefln("Graph was set up in %.3f s",
             (TickDuration.currSystemTick() - start_time).msecs / 1000.0);

    // do some search
    writeln("Let's do some search!");
    auto path = find_nearest(graph[XYZ(1, 1, 0)], graph[XYZ(998, 98, 0)]);
}
