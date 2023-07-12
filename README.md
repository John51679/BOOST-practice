# BOOST-practice
This project was created as part of "Algorithm implementation technologies" subject in Computer Engineering &amp; Informatics Department (CEID) of University of Patras. It involves the creation two searching algorithms using the BOOST library

In this project, the task was to create two algorithms, Dijkstra-SP and A*. The Dijkstra-SP algorithm works like the simple Dijkstra algorithm for some graph G(V, E) except that it terminates as soon as the shortest path from a vertex `start` ∈ V to a vertex `goal` ∈ V is found (versus discovering a shortest path from the vertex `start` to all vertices in the graph).

Based on the implementation part, it is now clear that the goal was not only to experiment with the boost library and get familiar with this form of programming (generalized programming) in `C++`, but also to observe the huge difference in the execution time of these programs compared to the LEDA library. 

In this repository there's the file `Source.cpp` as well as the `Makefile` file for direct compilation of the code.
