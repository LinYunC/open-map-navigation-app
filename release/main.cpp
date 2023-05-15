/*main.cpp*/

//
// Program to input Nodes (positions), Buildings and Footways
// from an Open Street Map file.
//
// Prof. Joe Hummel
// Northwestern University
// CS 211: Winter 2023
//

#include <iostream>
#include <string>
#include <limits>
#include <queue>
#include <iomanip>
#include <utility>
#include <vector>
#include <stack>

#include "building.h"
#include "buildings.h"
#include "footway.h"
#include "footways.h"
#include "node.h"
#include "nodes.h"
#include "osm.h"
#include "dist.h"
#include "graph.h"
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

constexpr double INF = numeric_limits<double>::max();

class prioritize
{
public:
  bool operator()(const pair<long long, double> &p1, const pair<long long, double> &p2) const
  {
    return p1.second > p2.second;
  }
};

string dijkstra(long long startV, long long destV, int &visited_num, double &distance, graph &graph)
{
  map<long long, double> dist;
  map<long long, long long> pred;
  map<long long, bool> visited;

  priority_queue<pair<long long, double>, vector<pair<long long, double>>, prioritize> unvisitedQueue;

  for (long long currentV : graph.getVertices())
  {
    dist[currentV] = INF;
    pred[currentV] = 0;
    visited[currentV] = false;
    unvisitedQueue.push(pair<long long, double>(currentV, INF));
  }

  dist[startV] = 0;
  unvisitedQueue.push(pair<long long, double>(startV, 0));

  while (!unvisitedQueue.empty())
  {
    pair<long long, double> currentV = unvisitedQueue.top();
    unvisitedQueue.pop();

    if (dist[currentV.first] == INF)
    {
      break;
    }

    else if (visited[currentV.first])
    {
      continue;
    }

    else
    {
      visited[currentV.first] = true;

      for (long long adjV : graph.neighbors(currentV.first))
      {
        double edgeWeight;

        graph.getWeight(currentV.first, adjV, edgeWeight);

        double alternativePathDistance = dist[currentV.first] + edgeWeight;

        if (alternativePathDistance < dist[adjV])
        {
          dist[adjV] = alternativePathDistance;
          pred[adjV] = currentV.first;
          unvisitedQueue.push(pair<long long, double>(adjV, alternativePathDistance));
        }
      }
    }
  }

  distance = dist[destV];

  int counter = 0;

  for (auto const &v : visited)
  {
    if (v.second)
      counter += 1;
  }

  // cout << "counter: " << counter << endl;

  visited_num = counter;

  stack<long long> stack;
  long long current = destV;
  string path_string;

  stack.push(destV);

  while (dist[current] != 0)
  {
    stack.push(pred[current]);
    current = pred[current];
  }

  while (!stack.empty())
  {
    long long id = stack.top();
    stack.pop();
    path_string.append(to_string(id));

    if (id != destV)
    {
      path_string.append("->");
    }
  }

  return path_string;
}

void navigate(Nodes &nodes, Buildings &buildings, Footways &footways, graph &graph)
{
  string name;
  Building startB(0, "", "");
  Building destB(0, "", "");
  // vector<string> findBuilding;
  // string first_building;

  cout << "Enter start building name (partial or complete)>" << endl;
  getline(cin, name);

  for (Building &B : buildings.MapBuildings)
  {
    if (B.Name.find(name) != string::npos && startB.ID == 0)
    {
      startB = B;
      break;
      // findBuilding.push_back(B.Name);
      // first_building = findBuilding.front();
      // if (B.Name == first_building){
      //   startB = B;
      // }
    }
  }

  if (startB.ID == 0)
  {
    cout << "**Start building not found" << endl;
    return;
  }

  vector<double> start_location = startB.getLocation(nodes);

  double min_start_d = INF;
  long long min_start_node = 0;
  long long min_start_footway = 0;

  for (Footway &F : footways.MapFootways)
  {
    for (long long nodeid : F.NodeIDs)
    {
      double lat = 0.0;
      double lon = 0.0;
      bool entrance = false;

      nodes.find(nodeid, lat, lon, entrance);

      double distance = distBetween2Points(start_location[0], start_location[1], lat, lon);

      if (distance < min_start_d)
      {
        min_start_d = distance;
        min_start_node = nodeid;
        min_start_footway = F.ID;
      }
    }
  }

  cout << " Name: " << startB.Name << endl;

  cout << " Approximate location: (" << start_location[0] << ", " << start_location[1] << ")" << endl;

  cout << " Closest footway ID " << min_start_footway << ", node ID " << min_start_node << ", distance " << min_start_d << endl;

  cout << "Enter destination building name (partial or complete)>" << endl;

  getline(cin, name);

  for (Building &B : buildings.MapBuildings)
  {
    if (B.Name.find(name) != string::npos && destB.ID == 0)
    {
      destB = B;
      break;
    }
  }

  if (destB.ID == 0)
  {
    cout << "**Destination building not found" << endl;
    return;
  }

  vector<double> dest_location = destB.getLocation(nodes);

  double min_dest_d = INF;
  long long min_dest_node = 0;
  long long min_dest_footway = 0;

  int node_n = 0;

  for (Footway &F : footways.MapFootways)
  {
    for (long long nodeid : F.NodeIDs)
    {
      double lat = 0.0;
      double lon = 0.0;
      bool entrance = false;

      nodes.find(nodeid, lat, lon, entrance);

      double distance = distBetween2Points(dest_location[0], dest_location[1], lat, lon);

      if (distance < min_dest_d)
      {
        min_dest_d = distance;
        min_dest_node = nodeid;
        min_dest_footway = F.ID;
      }

      node_n += 1;
    }
  }

  cout << " Name: " << destB.Name << endl;

  cout << " Approximate location: (" << dest_location[0] << ", " << dest_location[1] << ")" << endl;

  cout << " Closest footway ID " << min_dest_footway << ", node ID " << min_dest_node << ", distance " << min_dest_d << endl;

  int visited_num;
  double distance;

  string path_string = dijkstra(min_start_node, min_dest_node, visited_num, distance, graph);

  cout << "Shortest weighted path: " << endl;
  if (distance == INF)
  {
    cout << "**Sorry, destination unreachable" << endl;
  }
  else
  {
    cout << " # nodes visited: " << visited_num << endl;
    cout << " Distance: " << distance << " miles" << endl;
    cout << " Path: " << path_string << "" << endl;
  }
}

void build_graph(graph &graph, Footways &footways, Nodes &nodes)
{

  for (auto &pair : nodes)
  {
    graph.addVertex(pair.first);
  }

  for (Footway &F : footways.MapFootways)
  {
    for (size_t i = 0; i < F.NodeIDs.size() - 1; i++)
    {

      long long node_1 = F.NodeIDs[i];
      long long node_2 = F.NodeIDs[i + 1];

      double lat1 = 0.0;
      double lon1 = 0.0;
      bool entrance1 = false;

      double lat2 = 0.0;
      double lon2 = 0.0;
      bool entrance2 = false;

      nodes.find(node_1, lat1, lon1, entrance1);
      nodes.find(node_2, lat2, lon2, entrance2);

      double distance = distBetween2Points(lat1, lon1, lat2, lon2);
      double distance_reverse = distBetween2Points(lat2, lon2, lat1, lon1);

      graph.addEdge(node_1, node_2, distance);
      graph.addEdge(node_2, node_1, distance_reverse);
    }
  }
}

//
// main
//
int main()
{
  XMLDocument xmldoc;
  Nodes nodes;
  Buildings buildings;
  Footways footways;
  graph graph;

  cout << setprecision(12);

  cout << "** NU open street map **" << endl;

  string filename;

  cout << endl;
  cout << "Enter map filename> " << endl;
  getline(cin, filename);

  //
  // 1. load XML-based map file
  //
  if (!osmLoadMapFile(filename, xmldoc))
  {
    // failed, error message already output
    return 0;
  }

  //
  // 2. read the nodes, which are the various known positions on the map:
  //
  nodes.readMapNodes(xmldoc);

  //
  // NOTE: let's sort so we can use binary search when we need
  // to lookup nodes.
  //
  nodes.sortByID();

  //
  // 3. read the university buildings:
  //
  buildings.readMapBuildings(xmldoc);

  //
  // 4. read the footways, which are the walking paths:
  //
  footways.readMapFootways(xmldoc);

  build_graph(graph, footways, nodes);

  //
  // 5. stats
  //
  cout << "# of nodes: " << nodes.getNumMapNodes() << endl;
  cout << "# of buildings: " << buildings.getNumMapBuildings() << endl;
  cout << "# of footways: " << footways.getNumMapFootways() << endl;
  cout << "# of graph vertices: " << graph.NumVertices() << endl;
  cout << "# of graph edges: " << graph.NumEdges() << endl;

  //
  // now let the user for search for 1 or more buildings:
  //
  while (true)
  {
    string name;

    cout << endl;
    cout << "Enter building name, * to list, @ to navigate, or $ to end> " << endl;

    getline(cin, name);

    if (name == "$")
    {
      break;
    }
    else if (name == "*")
    {
      buildings.print();
    }
    else if (name == "@")
    {
      navigate(nodes, buildings, footways, graph);
    }
    else
    {
      buildings.findAndPrint(name, nodes, footways);
    }

  } // while

  //
  // done:
  //
  cout << endl;
  cout << "** Done  **" << endl;
  cout << "# of calls to getID(): " << Node::getCallsToGetID() << endl;
  cout << "# of Nodes created: " << Node::getCreated() << endl;
  cout << "# of Nodes copied: " << Node::getCopied() << endl;
  cout << endl;

  return 0;
}
