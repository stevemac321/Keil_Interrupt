#include "graph.h"
// graph operations
void create_graph(struct Graph *g, const int sz) {
    g->numVertices = 0;
    g->graph_capacity=sz;

    // Assign function pointers
    g->add_vertex = add_vertex_impl;
    g->add_edge = add_edge_impl;
}
// Internal function to add a vertex
static void add_vertex_impl(struct Graph* g, int ts, int ev, const int gp_regs[8], const int hw_regs[4]) {
    // Ensure we don't exceed the array size
    if (g->numVertices >= 32) {
        // Handle full graph (e.g., return or log error)
        return;
    }

    // Initialize vertex fields directly
    g->vertices[g->numVertices].timestamp = ts;
    g->vertices[g->numVertices].event = ev;

    // Copy the passed values into the general-purpose registers
    for (int i = 0; i < 8; i++) {
        g->vertices[g->numVertices].gp_registers[i] = gp_regs[i];
    }

    // Copy the passed values into the hardware-specific registers
    for (int i = 0; i < 4; i++) {
        g->vertices[g->numVertices].hw_registers[i] = hw_regs[i];
    }

    // Initialize the adjacency list to 0 (no connections initially)
    g->vertices[g->numVertices].adjList = 0;

    // Increment the number of vertices after initialization
    g->numVertices++;
}

// Internal function to add an edge between two vertices
static void add_edge_impl(struct Graph* g, size_t v, size_t w) {
    if (v >= g->numVertices || w >= g->numVertices) return;


    // Set the edge bit in the adjacency list
   // vertex->adjList[w / 32] |= (1 << (w % 32));
}
//
void bfs(struct Graph* g, int start_vertex) {
    // Initialize all vertices
    for (int i = 0; i < g->numVertices; i++) {
        setcolor(&(g->vertices[i].event), WHITE);  // Mark all vertices as unvisited (white)
        g->vertices[i].distance = -1;  // Initialize distances to -1 (undefined)
        g->vertices[i].parent = -1;  // No parent yet
    }

    // Set up the start vertex
    setcolor(&(g->vertices[start_vertex].event), GRAY);  // Mark the start vertex as discovered
    g->vertices[start_vertex].distance = 0;  // Distance from the start vertex to itself is 0
    g->vertices[start_vertex].parent = -1;  // Start vertex has no parent

    // Initialize a queue
    int queue[ARRAYSZ];  // Simple array-based queue (you can adjust this size if needed)
    int front = 0, rear = 0;  // Queue indices
    queue[rear++] = start_vertex;  // Enqueue the start vertex

    // BFS loop
    while (front != rear) {  // While queue is not empty
        int u = queue[front++];  // Dequeue vertex `u`

        // Explore all adjacent vertices of `u`
        for (int v = 0; v < g->numVertices; v++) {
            if (g->vertices[u].adjList & (1 << v)) {  // Check if there is an edge (u, v)
                if (getcolor(g->vertices[v].event) == WHITE) {  // If vertex `v` is unvisited
                    setcolor(&(g->vertices[v].event), GRAY);  // Mark vertex `v` as discovered
                    g->vertices[v].distance = g->vertices[u].distance + 1;  // Set distance
                    g->vertices[v].parent = u;  // Set parent
                    queue[rear++] = v;  // Enqueue vertex `v`
                }
            }
        }

        setcolor(&(g->vertices[u].event), BLACK);  // Mark vertex `u` as fully explored
    }
}
void inspect(struct Graph* pg) {
    // Local variables for inspection in the debugger
    size_t numVertices = pg->numVertices;
    size_t graphCapacity = pg->graph_capacity;
    
    // Loop through the vertices and extract relevant fields
    for (size_t i = 0; i < numVertices; i++) {
        // Extract vertex data into local variables for debugging
        int timestamp = pg->vertices[i].timestamp;
        int event = pg->vertices[i].event;
    }
}

// queue operations

void init_queue(struct Queue* q) {
    q->head = 0;
    q->tail = 0;
    q->count = 0;
}

void enqueue(struct Queue* q, int x) {
    if (q->count == QUEUE_SIZE) {
        // Queue is full
        //printf("Queue is full\n");
        return;
    }
    q->que[q->tail] = x;
    q->tail = (q->tail + 1) % QUEUE_SIZE;
    q->count++;
}

int dequeue(struct Queue* q) {
    if (q->count == 0) {
        // Queue is empty
        //printf("Queue is empty\n");
        return -1;  // Return some invalid value (or handle this differently)
    }
    int x = q->que[q->head];
    q->head = (q->head + 1) % QUEUE_SIZE;
    q->count--;
    return x;
}
