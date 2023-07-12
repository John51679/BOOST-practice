#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <queue>
#include <vector>
#include <chrono>

using namespace boost;
using namespace std;
using namespace std::chrono;

/*for G_mat*/
#define rows 60
#define cols 1000
/*for random graph*/
#define NODES 10000
/*for cost generation range*/
#define COST_GEN_RANGE 10000

struct NodeProperty {
	double heuristic;
	int distance;
	int node_state;
	int pred;
	int gray_distance;
	int gray_pred;
};

struct EdgeProperty {
	int cost;
};

typedef adjacency_list<vecS, vecS, undirectedS, NodeProperty, EdgeProperty> Graph;
typedef graph_traits<Graph>::vertex_descriptor vertex_d;
typedef graph_traits<Graph>::edge_descriptor edge_d;
typedef graph_traits<Graph>::vertex_iterator vertex_t;
typedef graph_traits<Graph>::edge_iterator edge_t;
typedef property_map<Graph, int NodeProperty::*>::type node_property_map;
typedef property_map<Graph, int EdgeProperty::*>::type edge_property_map;


int my_Dijkstra_SP(Graph& graph, vertex_d& start, vertex_d& goal, edge_property_map& costs, node_property_map& dist, node_property_map& pred);
int my_Astar(Graph& graph, vertex_d& start, vertex_d& goal, edge_property_map& cost, property_map<Graph, double NodeProperty::*>::type& heuristic, node_property_map& dist, node_property_map& pred);
void init(Graph& graph, edge_property_map& epm, node_property_map& dist);
void init_mat(Graph& graph, edge_property_map& epm, node_property_map& dist);
void h_rand(Graph& graph, vertex_d goal, property_map<Graph, double NodeProperty::*>::type& heuristic, edge_property_map& cost, node_property_map& dist, node_property_map& pred);
void h_matrix(Graph& graph, int xt, int yt, property_map<Graph,double NodeProperty::*>::type& heuristic);

int main() {
	int total_nodes;
	srand(time(0));
	auto start = high_resolution_clock::now();
	auto stop = high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(stop - start);

	/*Graph G_mat(rows * cols);
	property_map<Graph, double NodeProperty::*>::type heuristic_map_mat = get(&NodeProperty::heuristic, G_mat);
	node_property_map pred_map_mat = get(&NodeProperty::pred, G_mat);
	edge_property_map cost_map_mat = get(&EdgeProperty::cost, G_mat);
	node_property_map dist_map_mat = get(&NodeProperty::distance, G_mat);
	init_mat(G_mat, cost_map_mat, dist_map_mat);
	int x_cord_g = rand() % rows;
	int y_cord_g = cols - 1;
	h_matrix(G_mat, x_cord_g, y_cord_g, heuristic_map_mat);
	vertex_d v_d_g_mat = vertex(x_cord_g * cols + y_cord_g, G_mat);
	int x_cord_s = rand() % rows;
	int y_cord_s = 0;
	vertex_d v_d_s_mat = vertex(x_cord_s * cols + y_cord_s, G_mat);

	start = high_resolution_clock::now();
	total_nodes = my_Dijkstra_SP(G_mat, v_d_s_mat, v_d_g_mat, cost_map_mat, dist_map_mat, pred_map_mat);
	stop = high_resolution_clock::now();
	duration = duration_cast<microseconds>(stop - start);

	cout << "Dijkstra-SP time -> " << (double)duration.count() / 1000000 << " seconds" << endl;
	cout << "Dijkstra-SP has seen " << total_nodes << " nodes" << endl;

	start = high_resolution_clock::now();
	total_nodes = my_Astar(G_mat, v_d_s_mat, v_d_g_mat, cost_map_mat, heuristic_map_mat, dist_map_mat, pred_map_mat);
	stop = high_resolution_clock::now();
	duration = duration_cast<microseconds>(stop - start);

	cout << "A* time -> " << (double)duration.count() / 1000000 << " seconds" << endl;
	cout << "A* has seen " << total_nodes << " nodes" << endl;	*/
	

	Graph G(NODES);
	int counter = 0;

	vertex_t vi_from, vi_end;
	for (tie(vi_from, vi_end) = vertices(G); vi_from != vi_end; vi_from++) {
		counter = 0;
		while (counter != 2) {
			vertex_d vi_to = rand() % NODES;
			while (vi_to == *vi_from) vi_to = rand() % NODES;
			if (!edge(*vi_from, vi_to, G).second) {
				add_edge(*vi_from, vi_to, G);
				counter++;
			}
		}
	}
	cout << "Number of Nodes = " << num_vertices(G) << endl;
	cout << "Number of edges = " << num_edges(G) << endl;

	property_map<Graph, double NodeProperty::*>::type heuristic_map = get(&NodeProperty::heuristic, G);
	node_property_map pred_map = get(&NodeProperty::pred, G);
	edge_property_map cost_map = get(&EdgeProperty::cost, G);
	node_property_map dist_map = get(&NodeProperty::distance, G);

	init(G, cost_map, dist_map);
	vertex_d t = rand() % num_vertices(G);
	h_rand(G, t, heuristic_map, cost_map, dist_map, pred_map);
	vertex_d v_d_s = rand() % num_vertices(G);
	while (v_d_s == t) rand() % num_vertices(G);
	vertex_d v_d_g = t;

	start = high_resolution_clock::now();
	total_nodes = my_Dijkstra_SP(G, v_d_s, v_d_g, cost_map, dist_map, pred_map);
	stop = high_resolution_clock::now();
	duration = duration_cast<microseconds>(stop - start);
	cout << "Dikstra-SP time -> " << (double)duration.count() / 1000000 << " seconds" << endl;
	cout << "Dikstra-SP has seen " << total_nodes << " nodes" << endl;	

	start = high_resolution_clock::now();
	total_nodes = my_Astar(G, v_d_s, v_d_g, cost_map, heuristic_map, dist_map, pred_map);
	stop = high_resolution_clock::now();
	duration = duration_cast<microseconds>(stop - start);
	cout << "A* time -> " << (double)duration.count() / 1000000 << " seconds" << endl;
	cout << "A* has seen " << total_nodes << " nodes" << endl;	

	return 0;
}

void init(Graph& graph, edge_property_map& epm, node_property_map& dist){
	edge_t ei, ei_end;
	vertex_t vi, vi_end;
	srand(time(0));
	for (tie(ei, ei_end) = edges(graph); ei != ei_end; ei++) {
		epm[*ei] = rand()%COST_GEN_RANGE + 1;
	}
	for (tie(vi, vi_end) = vertices(graph); vi != vi_end; vi++) {
		dist[*vi] = INT_MAX;
	}
}

void init_mat(Graph& graph, edge_property_map& epm, node_property_map& dist) {
	int r, c;
	vertex_t vi, vi_end;
	for (r = 0; r < rows; r++) {
		for (c = 0; c < cols; c++) {
			if (c != cols - 1 && r != rows - 1) {
				add_edge(r * cols + c, r * cols + c + 1, graph);
				add_edge(r * cols + c, (r + 1) * cols + c, graph);
			}
			else if (c == cols - 1 && r != rows - 1) add_edge(r * cols + c, (r + 1) * cols + c, graph);
			else if (c != cols - 1 && r == rows - 1) add_edge(r * cols + c, r * cols + c + 1, graph);
		}
	}
	edge_t ei, ei_end;
	srand(time(0));
	int i = 0;
	for (tie(ei, ei_end) = edges(graph); ei != ei_end; ei++,i++) {
		epm[*ei] = rand() % COST_GEN_RANGE + 1;
	}
	
	for (tie(vi, vi_end) = vertices(graph); vi != vi_end; vi++) {
		dist[*vi] = INT_MAX;
	}
}

void h_matrix(Graph& graph, int xt, int yt, property_map<Graph, double NodeProperty::*>::type& heuristic){
	// grammh pou 8elw * ARI8MOS STHLWN + sthlh pou 8elw
	vertex_t vi, vi_end;
	int r, c;
	for (r = 0; r < rows; r++) {
		for (c = 0; c < cols; c++) {
			vertex_d it = r * cols + c;
			heuristic[it] = sqrt((c - xt) * (c - xt) + (r - yt) * (r - yt));
		}
	}
}

void h_rand(Graph& graph, vertex_d goal, property_map<Graph, double NodeProperty::*>::type& heuristic, edge_property_map& cost, node_property_map& dist, node_property_map& pred) {
	srand(time(0));
	heuristic[goal] = 0;
	vertex_d L1 = rand() % num_vertices(graph);
	while (L1 == goal) L1 = rand() % num_vertices(graph);
	vertex_t vi, vi_end, vi2, vi2_end;
	int* dist_L1_i = (int*)malloc(num_vertices(graph) * sizeof(int));
	//int* distanceL2 = (int*)malloc(num_vertices(graph) * sizeof(int));
	int tmp = 0;
	int i = 0;
	for (tie(vi, vi_end) = vertices(graph); vi != vi_end; vi++, i++) {
		vertex_d j = *vi;
		for (tie(vi2, vi2_end) = vertices(graph); vi2 != vi2_end; vi2++) {
			dist[*vi2] = INT_MAX;
		}
		dist_L1_i[i] = my_Dijkstra_SP(graph, L1, j, cost, dist, pred);
		if (dist_L1_i[i] > dist_L1_i[tmp]) tmp = i;
	}
	vertex_d L2 = tmp;
	i = 0;
	int dist_L1_t, dist_t_L2, dist_i_L2, dist_i_t, dist_i_t_2;
	for (tie(vi, vi_end) = vertices(graph); vi != vi_end; vi++, i++) {
		vertex_d j = *vi;
		dist_L1_t = my_Dijkstra_SP(graph, L1, goal, cost, dist, pred);
		dist_i_t = dist_L1_t - dist_L1_i[i];
		if (dist_i_t < 0) dist_i_t = dist_i_t * (-1);

		dist_i_L2 = my_Dijkstra_SP(graph, j, L2, cost, dist, pred);
		dist_t_L2 = my_Dijkstra_SP(graph, goal, L2, cost, dist, pred);
		dist_i_t_2 = dist_i_L2 - dist_t_L2;
		if (dist_i_t_2 < 0) dist_i_t_2 = dist_i_t_2 * (-1);
		if (dist_i_t > dist_i_t_2) heuristic[*vi] = dist_i_t;
		else heuristic[*vi] = dist_i_t_2;
	}
}

int my_Dijkstra_SP(Graph& graph,vertex_d& start, vertex_d& goal, edge_property_map& cost , node_property_map& dist, node_property_map& pred) {
	priority_queue<vertex_d, std::vector<vertex_d>> q;
	vertex_t vi, vi_end;
	node_property_map node_state = get(&NodeProperty::node_state, graph);
	node_property_map gray_pred = get(&NodeProperty::gray_pred, graph);
	node_property_map gray_dist = get(&NodeProperty::gray_distance, graph);
	enum{WHITE,GRAY,BLACK};
	int minDist, tmp;
	dist[start] = 0;
	for (tie(vi, vi_end) = vertices(graph); vi != vi_end; vi++) {
		node_state[*vi] = WHITE;
		gray_dist[*vi] = INT_MAX;
	}
	graph_traits<Graph>::adjacency_iterator curr_node, end;
	node_state[start] = BLACK;
	q.push(start);
	vertex_d t;
	while (!q.empty() && node_state[goal] != BLACK) {
		vertex_d pred_node = q.top();
		q.pop();
		for (tie(curr_node, end) = adjacent_vertices(pred_node, graph); curr_node != end; curr_node++) {
			if (node_state[*curr_node] == WHITE) {
				q.push(*curr_node);
				
				pred[*curr_node] = pred_node;
				edge_d e = edge(pred_node, *curr_node, graph).first;
				dist[*curr_node] = dist[pred_node] + cost[e];
				node_state[*curr_node] = GRAY;
			}
			else if (node_state[*curr_node] == GRAY){
				q.push(*curr_node);
				
				gray_pred[*curr_node] = pred_node;
				edge_d e = edge(pred_node, *curr_node, graph).first;
				gray_dist[*curr_node] = dist[pred_node] + cost[e];
				if (gray_dist[*curr_node] < dist[*curr_node] || node_state[pred[*curr_node]] == BLACK) {
					dist[*curr_node] = gray_dist[*curr_node];
					pred[*curr_node] = gray_pred[*curr_node];
				}
			}
		}
		
		minDist = INT_MAX;
		while (!q.empty()) {
			vertex_d n = q.top();
			q.pop();
			tmp = dist[n];
			if (minDist > tmp) {
				minDist = tmp;
				t = n;
			}
		}
		if (node_state[t] == BLACK) t = pred[t];
		if (q.empty()) {
			node_state[t] = BLACK;
			q.push(t);
		}
	}

	t = vertex(goal, graph);
        int count = 0;
	
	while (t != start) {
		count++;
		t = pred[t];
	}

	return count;
}


int my_Astar(Graph& graph, vertex_d& start, vertex_d& goal, edge_property_map& cost, property_map<Graph,double NodeProperty::*>::type& heuristic, node_property_map& dist, node_property_map& pred) {
	priority_queue<vertex_d, std::vector<vertex_d>> q;
	vertex_t vi, vi_end;
	node_property_map node_state = get(&NodeProperty::node_state, graph);
	node_property_map gray_pred = get(&NodeProperty::gray_pred, graph);
	node_property_map gray_dist = get(&NodeProperty::gray_distance, graph);
	enum { WHITE, GRAY, BLACK };
	int minDist, tmp;
	dist[start] = 0;
	for (tie(vi, vi_end) = vertices(graph); vi != vi_end; vi++) {
		node_state[*vi] = WHITE;
	}
	graph_traits<Graph>::adjacency_iterator curr_node, end;
	node_state[start] = BLACK;
	q.push(start);
	vertex_d t;
	while (!q.empty() && node_state[goal] != BLACK) {
		vertex_d pred_node = q.top();
		q.pop();
		for (tie(curr_node, end) = adjacent_vertices(pred_node, graph); curr_node != end; curr_node++) {
			if (node_state[*curr_node] == WHITE) {
				q.push(*curr_node);
				pred[*curr_node] = pred_node;
				edge_d e = edge(pred_node, *curr_node, graph).first;
				dist[*curr_node] = dist[pred_node] + cost[e] + heuristic[*curr_node];
				node_state[*curr_node] = GRAY;
			}
			else if (node_state[*curr_node] == GRAY) {
				q.push(*curr_node);

				gray_pred[*curr_node] = pred_node;
				edge_d e = edge(pred_node, *curr_node, graph).first;
				gray_dist[*curr_node] = dist[pred_node] + cost[e] + heuristic[*curr_node];
				if (gray_dist[*curr_node] < dist[*curr_node] || node_state[pred[*curr_node]] == BLACK) {
					dist[*curr_node] = gray_dist[*curr_node];
					pred[*curr_node] = gray_pred[*curr_node];
				}
			}
		}
		minDist = INT_MAX;

		while (!q.empty()) {
			vertex_d n = q.top();
			q.pop();
			tmp = dist[n];
			if (minDist > tmp) {
				minDist = tmp;
				t = n;
			}
		}
		if (node_state[t] == BLACK) t = pred[t];
		if (q.empty()) {
			node_state[t] = BLACK;
			q.push(t);
		}
	}
	t = vertex(goal, graph);
        int count = 0;

        while (t != start) {
                count++;
                t = pred[t];
        }

	return count;
}
