#ifndef __GRAPH_H
#define __GRAPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>

#define TIMESTAMP_WINDOW 100  // Adjust this value as needed
#define ARRAYSZ 32
#define QUEUE_SIZE 1000000

enum EventField {
    // Regular events (lower bits)
    TIMER_EVENT            = 0x00000001,  // TIM2 interrupt or any other timer
    GPIO_PIN_CHANGE        = 0x00000002,  // Change in GPIO pin state
    INTERRUPT_EVENT        = 0x00000004,  // General interrupt event
    ENTER_CRITICAL_SECTION = 0x00000008,  // Enter critical section
    EXIT_CRITICAL_SECTION  = 0x00000010,  // Exit critical section
    POWER_STATE_CHANGE     = 0x00000020,  // Power state transition
    ERROR_EVENT            = 0x00000040,  // Error or fault condition
    WATCHDOG_RESET         = 0x00000080,  // Watchdog reset event
    SENSOR_READ_EVENT      = 0x00000100,  // Sensor read event
    COMMUNICATION_EVENT    = 0x00000200,  // Communication event
    MAIN_LOOP_EVENT        = 0x00000400,  // Event added by the main loop
    INTERRUPT_HANDLER_EVENT = 0x00000800, // Event added by the interrupt handler

    // Colors using the two most significant bits (MSBs) in the 32-bit field
    WHITE                  = 0x00000000,  // No color (default)
    GRAY                   = 0x40000000,  // 01 in the MSBs (used in graph traversal)
    BLACK                  = 0x80000000   // 10 in the MSBs (used in graph traversal)
};

// Define the Vertex structure
struct Vertex {
    int timestamp;          // Placeholder for the timestamp
    int event;              // Placeholder for event description or enum
		int distance;
		int parent;
    int gp_registers[8];    // General-purpose registers
    int hw_registers[4];    // Hardware-specific registers
    uint32_t adjList;       // 32-bit bitvector representing adjacency list
};

// Function to get the color from the event field (the 2 MSBs)
inline int getstate(int event) {
    // Mask out the two MSBs (color bits) and return the lower 30 bits as the state
    return event & 0x3FFFFFFF;
}
inline void setstate(int *event, int state) {
    // Mask out the color bits from the state
    state &= 0x3FFFFFFF;  // Ensure the new state doesn't affect the color bits

    // Clear the existing state bits (lower 30 bits) in the event
    *event &= 0xC0000000;  // Preserve the color bits (MSBs)

    // Set the new state value (lower 30 bits)
    *event |= state;
}

static inline int getcolor(int event) {
    // Extract the 2 MSBs using a mask and right-shift to the lowest bits
    return (event & 0xC0000000) >> 30;
}

// Function to set the color in the event field (the 2 MSBs)
static inline void setcolor(int *event, int color) {
    // Clear the current color bits (2 MSBs)
    *event &= 0x3FFFFFFF;  // Clear the 2 MSBs

    // Set the new color by shifting it into the 2 MSB positions
    *event |= (color << 30);
}

// Define the Graph structure with a fixed-size array vertices
struct Graph { 
		struct Vertex vertices[ARRAYSZ];  // Fixed-size array of 32 vertices
    size_t numVertices;          // Number of vertices currently added
		size_t graph_capacity;
	 // Function pointers for graph operations
    void (*add_vertex)(struct Graph*, int, int, const int[8], const int[4]);
    void (*add_edge)(struct Graph*, size_t, size_t);
	 
};

// graph operations
void create_graph(struct Graph *g, const int sz);
static void add_vertex_impl(struct Graph* g, int ts, int ev, const int gp_regs[8], const int hw_regs[4]);
static void add_edge_impl(struct Graph* g, size_t v, size_t w);
void save_to_vertex();
void inspect(struct Graph*);
void bfs(struct Graph* g, int start_vertex);


 struct Queue {
    int que[QUEUE_SIZE];  // Replace 'T' with the desired type (e.g., int)
    int head;
    int tail;
    int count;
} ;
 
void init_queue(struct Queue* q);
void enqueue(struct Queue* q, int x);
int dequeue(struct Queue* q);

static inline int is_empty(const struct Queue* q) {
    return q->count == 0;
}

static inline int is_full(const struct Queue* q) {
    return q->count == QUEUE_SIZE;
}

#ifdef __cplusplus
}
#endif

#endif /* __GRAPH_H */