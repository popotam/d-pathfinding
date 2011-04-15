#!/usr/bin/rdmd

import std.stdio;
import std.string;
import std.conv;


struct Connection {

    Vertex destination;
    uint cost;

    this(Vertex destination, uint cost) {
        this.destination = destination;
        this.cost = cost;
    }

    string toString() {
        return format("C(%s, %s)", destination, cost);
    }

}


struct Vertex {

    uint index;
    Connection[] connections;

    string toString() {
        return format("V#%s", index);
    }

}


immutable struct XYZ {

    int x, y, z;

    string toString() {
        return format("XYZ(%s, %s, %s)", x, y, z);
    }

}


enum DIRECTIONS = [XYZ(1, 0, 0), XYZ(-1, 0, 0),
                   XYZ(0, 1, 0), XYZ(0, -1, 0)];


class GraphList {

    Vertex[] vertices;

    this(uint num_vertices) {
        this.vertices = new Vertex[num_vertices];
        foreach (uint index; 0 .. num_vertices) {
            this.vertices[index].index = index;
        }
    }
}


alias Vertex[XYZ] Graph;


void main() {
    auto graph = new GraphList(5);
    foreach (vertex; graph.vertices) {
        writeln(vertex.index);
    }
    Vertex v = Vertex(1);
    auto c = new Connection(v, 42);
    c.cost = 1;
    writefln("Vertex %s", v.connections);
    writefln("Connection %s %s", c.destination, c.cost);
    writeln(DIRECTIONS);
}
