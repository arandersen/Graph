#include "Graph.h"
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <queue>
#include <algorithm>
#include <unordered_map>
#include <cfloat>
#include <set>

using namespace std;

Graph::Graph(const char* const & edgelist_csv_fn) {
    ifstream file(edgelist_csv_fn); //open the file 
    string line;

    while(getline(file, line)){
        istringstream ss(line);
        string node1, node2, weightStr;

        getline(ss, node1, ',');
        getline(ss, node2, ',');
        getline(ss, weightStr, '\n');

        double weight = stod(weightStr); //convert the string to a double

        //checks if the nodes exist in the graph, if DNE then add them
        if(node_map.find(node1) == node_map.end()){
            node_map[node1].id = node1;
        }
        if(node_map.find(node2) == node_map.end()){
            node_map[node2].id = node2;
        }

        // add the edges
        Edge edge1{node1, node2, weight};
        Edge edge2{node2, node1, weight};
        node_map[node1].edges.push_back(edge1);
        node_map[node2].edges.push_back(edge2);
        edges.push_back(edge1);
    }
    file.close();   //close the file 
}

//return the number of nodes
unsigned int Graph::num_nodes() {
    return node_map.size();
}

vector<string> Graph::nodes() {
    vector<string>result;
    for(const auto &node :node_map){
        result.push_back(node.first);
    } 
    return result;
}

unsigned int Graph::num_edges() {
    return edges.size();
}

unsigned int Graph::num_neighbors(string const & node_label) {
    if(node_map.find(node_label) != node_map.end()){
        return node_map[node_label].edges.size();
    }
    return 0;
}

double Graph::edge_weight(string const & u_label, string const & v_label) {
    for(const Edge& edge : edges){
        if((edge.source == u_label && edge.adress == v_label) ||
        (edge.source == v_label && edge.adress == u_label)){
                return edge.weight;
        }
    }
    return -1.0; //indicate there's no edge or node DNE (error)
}

vector<string> Graph::neighbors(string const & node_label) {
    vector<string> result;
    auto it = node_map.find(node_label);
    if(it != node_map.end()){
        for(const auto & edge : it->second.edges){
            result.push_back(edge.adress);
        }
    }
    return result;
}

vector<string> Graph::shortest_path_unweighted(string const & start_label, string const & end_label) {
    // first we check if start and end node are the same
    if(start_label == end_label){
        return vector<string>{start_label}; //vector with a single node
    }

    //create a map 
    unordered_map<string,string> previous;

    //queue for the BFS
    queue<string> Q;
    Q.push(start_label); //test

    //performing the BFS
    while(!Q.empty()){
        string current = Q.front();
        Q.pop();

        //find the adress then it will reconstruct the path 
        if(current == end_label){
            vector<string> path;
            for(string a = end_label; a != start_label; a = previous[a]){
                path.push_back(a);
            }
            path.push_back(start_label);    //push back to add the start node
            reverse(path.begin(), path.end());  //reverse the path to get from the start
            return path;
        }

        //visting all of the neighbors
        for(const Edge& edge: node_map[current].edges){
            //check if we have visit or not, add to queue and mark current
            if(previous.find(edge.adress) == previous.end()){
                Q.push(edge.adress);
                previous[edge.adress] = current;
            }
        }
    }
    //edge case: there's no path from start to end
    return vector<string>();
}

// struct CompareDist{
//     bool operator()(pair<double, string> n1, pair<double, string> n2){
//         return n1.first > n2.first;
//     }
// };

vector<tuple<string,string,double>> Graph::shortest_path_weighted(string const & start_label, string const & end_label) {
    // //trial 1 using multiset and dijikstra algorithm

    //check if start and end nodes are the same
    if(start_label == end_label){
        return vector<tuple<string,string,double>>{make_tuple(start_label,end_label,-1.0)}; //return a vector with a single node
    }

    //distance map and previous node map
    unordered_map<string, double> distance;
    unordered_map<string, string> previous;

    for(const auto& node: node_map){
        if(node.first == start_label){
            distance[node.first] = 0.0;
        }else{
            distance[node.first] = DBL_MAX;
        }
    }

    //set the distance from start node
    distance[start_label] = 0.0;

    // //djikstra algorithm
    multiset<pair<double, string>> Q;
    Q.insert({0,start_label});  //add start label to the queue with distance 0

    while(!Q.empty()){
        string u = Q.begin()->second;
        double d = Q.begin()->first;
        Q.erase(Q.begin());

        //edge case: ignore if current is already the shortest
        if(d>distance[u]){
            continue;
        }

         //find the address, reconstruct the path
        if(u == end_label){
            vector<tuple<string,string, double>> path;
            for(string a = end_label; a != start_label; a = previous[a]){
                //double weight = (a == previous[a]) ? -1 : edge_weight(previous[a],a);
                double weight = edge_weight(previous[a],a);
                path.push_back(make_tuple(previous[a], a, weight));
            }
            reverse(path.begin(), path.end());
            return path;
        }

        //visitng all of the neighbors
        for(const Edge& edge : node_map[u].edges){
            double arr = distance[u] + edge.weight;
            if(arr < distance[edge.adress]){
                //if the node is within the queue then remove
                auto it = Q.find({distance[edge.adress], edge.adress});
                if(it != Q.end()){
                    Q.erase(it);
                }

                //updating the distance and prev node, also add to the queue
                distance[edge.adress] = arr;
                previous[edge.adress] = u;
                Q.insert({arr, edge.adress});
            }
        }
    }

    //edge case:  no path from start to end
    return vector<tuple<string,string,double>>();
}


vector<vector<string>> Graph::connected_components(double const & threshold) {
    vector<vector<string>> components;

    unordered_map<string, bool> visited;
    for(const auto& node_match : node_map){
        visited[node_match.first] = false;
    }

    for(const auto& node_match : node_map){
        if(!visited[node_match.first]){
            vector<string> component;
            queue<string> Q;

            string u = node_match.first;
            Q.push(u);

            while(!Q.empty()){
                string v = Q.front();
                Q.pop();
                visited[v] = true;
                component.push_back(v);

                for(const Edge& edge : node_map[v].edges){
                    if(edge.weight <= threshold && !visited[edge.adress]){
                        Q.push(edge.adress);
                    }
                }
            }
            components.push_back(component);
        }
    }
    return components;
}

double Graph::smallest_connecting_threshold(string const & start_label, string const & end_label) {
    if(start_label == end_label){
        return 0.0;
    }
    
    if(node_map.find(start_label) == node_map.end() || node_map.find(end_label) == node_map.end()){
        return -1; //start or end node DNE
    }

    //sort edge based on weight
    vector<Edge> sorted_edges = edges;
    sort(sorted_edges.begin(), sorted_edges.end(), [](const Edge &edge1, const Edge &edge2){
        return edge1.weight < edge2.weight;
    });

    DisjointSet D(node_map);

    //go through all edges starting from the smallest weight
    for(const Edge& edge : sorted_edges){
        //node of current edges
        string node1 = edge.source;
        string node2 = edge.adress;

        //union the disjoint node
        D.Union(node1,node2);

        //check if the nodes of start and end are connected
        if(D.Find(start_label) == D.Find(end_label)){
            return edge.weight;
        }
    }
    return -1; //path DNE
}

//default constructor
DisjointSet:: DisjointSet(){}

//overload constructor that create a disjoint set from map to the node 
DisjointSet::DisjointSet(const unordered_map<string, Node>& nodes){
    for(const auto& node : nodes){
        parent[node.first] = node.first;    //each node is its parent
        rank[node.first] = 0;           //all nodes rank 0 intially
    }
}

//represent the set that the give node belongs to
string DisjointSet::Find(const string& x){
    if(x != parent[x]){
        parent[x] = Find(parent[x]);
    }
    return parent[x];
}

//merge two sets together
void DisjointSet::Union(const string& x, const string& y){
    string xRoot = Find(x);
    string yRoot = Find(y);

    //unionize by rank
    if(xRoot != yRoot){
        if(rank[xRoot] < rank[yRoot]){
            parent[xRoot] = yRoot;
        }else if(rank[xRoot] > rank[yRoot]){
            parent[yRoot] = xRoot;
        }else{
            parent[yRoot] = xRoot;
            ++rank[xRoot];
        }
    }
}
