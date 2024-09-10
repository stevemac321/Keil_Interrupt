# uVisionEmbeddedStateGraphTimer

**uVisionEmbeddedStateGraphTimer** is a C-based embedded systems project designed to run on an STM32F401xx microcontroller using the Keil uVision IDE. The program constructs a dynamic graph based on system state polling from both the main execution loop and a timer interrupt. It leverages atomic operations to ensure thread-safe updates to shared resources. Each vertex in the graph holds a bitvector representing the edges (adjacency list), and edges are created based on command flags and a timestamp window. The project also includes a Breadth-First Search (BFS) algorithm for graph traversal.

## Features

- **State Polling**: Polls system states from both the `main()` function and a timer interrupt.
- **Graph Management**: 
  - Each vertex represents a point in time and holds system state information.
  - A bitvector in each vertex represents its adjacency list (the edges to other vertices).
  - Edges are dynamically created based on command flags and timestamp windows.
- **Timer Interrupt Integration**: Polls the system at regular intervals using a hardware timer interrupt.
- **Atomic Operations**: Ensures safe concurrent access to shared resources (such as vertices) using atomic operations.
- **Breadth-First Search (BFS)**: A BFS implementation is included for traversing the graph, starting from any given vertex.
- **Dynamic Adjacency**: Edges between vertices are formed based on event flags and command inputs within a certain timestamp window, allowing for dynamic and evolving graph structures.

## System Requirements

- **Keil uVision IDE**: The project is designed for the Keil uVision IDE.
- **STM32F401xx Microcontroller**: Targets the STM32F401xx series, but the project can be modified for other STM32 families.
- **C Compiler**: A C compiler compatible with Keil uVision.
- **ST-Link Utility**: Used for flashing the binary to the STM32 microcontroller.
- **Atomic Operations**: Ensure the atomic library or appropriate atomic operations are available on your target platform.

## Installation

1. **Clone the Repository**:

   ```bash
   git clone https://github.com/yourusername/uVisionEmbeddedStateGraphTimer.git
   cd uVisionEmbeddedStateGraphTimer
