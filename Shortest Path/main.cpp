/*
Name: Kirtan Patel
Section: CPSC2120-003
*/

#include <iostream>
#include <fstream>
#include <vector>
#include "graphics.h"
#include <cmath>
#include <queue>
#include <set>
#include <map>
using namespace std;

// Each point represents (x, y, z=elevation above sea level)
// units are in feet; x and y increase to the east and north, respectively

struct Point {
  double x, y, z;
};

struct Node{
  // Point p;
  double array[3];
  int index;
  // double x, y;
  Node *left, *right;
  Node (Point pos, int i) { array[0] = pos.x; array[1] = pos.y; array[2] = pos.z; index = i; left=right=NULL; }
};

Node* root = NULL;

priority_queue <pair<double, int>> pq;

// All the points in our LIDAR data set
vector<Point> all_points;

Point mins, maxes; // highest and lowest values in dataset
Point winmin, winmax; // corners of window being displayed on screen

// Size of our window on screen
double window_xsize = 1000, window_ysize = 800;

// Source and destination nodes for shortest path calculation
int source_node = -1;
int destination_node = -1;

Node* insert(int d, Point l, Node* root, int i)
{
  if (root == NULL) return new Node(l, i);
  if (d == 0){
    if (l.x < root->array[0]){
      root->left = insert((d+1)%2, l, root->left, i);
    }
    else root->right = insert((d+1)%2, l, root->right, i);
  }
  if (d == 1){
    if (l.y < root->array[1]){
      root->left = insert((d+1)%2, l, root->left, i);
    }
    else root->right = insert((d+1)%2, l, root->right, i);
  }
  return root;
}

void nearest(Node* root, double x, double y, int d)
{
  if (root == NULL) return;
  double dx = root->array[0]-x;
  double dy = root->array[1]-y;
  double dist = sqrt(dx*dx + dy*dy);
  pq.push(make_pair(dist, root->index));
  if (pq.size() > 10) pq.pop();
  double arr[2] = {x,y};
  Node* curr;
  Node* next;
  if (arr[d] < root->array[d]){
    curr = root->left;
    next = root->right;
  }
  else{
    curr = root->right;
    next = root->left;
  }
  nearest(curr, x, y, (d+1)%2);
  if (fabs(root->array[d] - arr[d]) < pq.top().first){
    nearest(next, x, y, (d+1)%2);
  }
  return;
}

double e_dist(Point X, Point Y){
  return sqrt(pow((X.x-Y.x),2) + pow((X.y-Y.y),2) + pow((X.z-Y.z),2) );
}

double slope (Point X, Point Y){
  return abs(X.z-Y.z)/sqrt(pow((X.x-Y.x),2) + pow((X.y-Y.y),2));
}

map <double, int> dists;
map <int, int> pred;
bool line = false;

bool dijkstra()
{
  line = false;
  set<pair<double,int>> S;
  double infinity = 999999999;
  for (unsigned int i=0; i < all_points.size(); i++) dists[i] = infinity;
  dists[source_node] = 0.0;
  S.insert(make_pair(0.0, source_node));
  while (!S.empty()) {
    int to_visit = S.begin()->second;
    if (to_visit == destination_node) { line = true; return true; }
    S.erase(S.begin());
    nearest(root, all_points[to_visit].x, all_points[to_visit].y, 0);
    while (!pq.empty()) {
      if (slope(all_points[to_visit], all_points[pq.top().second]) <= 1){
        if (dists[to_visit] + e_dist(all_points[to_visit], all_points[pq.top().second]) < dists[pq.top().second]) {
          dists[pq.top().second] = dists[to_visit] + e_dist(all_points[to_visit], all_points[pq.top().second]);
          S.insert(make_pair(dists[to_visit], pq.top().second));
          pred[pq.top().second] = to_visit;
        }
      }
      pq.pop();
    }
  }
  return false;
}

// Called whenever a key is pressed
void keyhandler(int key)
{
  double x_range = winmax.x - winmin.x;
  double y_range = winmax.y - winmin.y;
  double x_center = winmin.x + x_range/2;
  double y_center = winmin.y + y_range/2;

  if (key == KEY_LEFT) { winmin.x -= x_range/10; winmax.x -= x_range/10; }
  if (key == KEY_RIGHT) { winmin.x += x_range/10; winmax.x += x_range/10; }
  if (key == KEY_UP) { winmin.y += y_range/10; winmax.y += y_range/10; }
  if (key == KEY_DOWN) { winmin.y -= y_range/10; winmax.y -= y_range/10; }

  if (key == '=') { // Zoom in
    winmin.x = x_center - x_range / 2 * 0.8;
    winmax.x = x_center + x_range / 2 * 0.8;
    winmin.y = y_center - y_range / 2 * 0.8;
    winmax.y = y_center + y_range / 2 * 0.8;
  }
  if (key == '-') { // Zoom out
    winmin.x = x_center - x_range / 2 / 0.8;
    winmax.x = x_center + x_range / 2 / 0.8;
    winmin.y = y_center - y_range / 2 / 0.8;
    winmax.y = y_center + y_range / 2 / 0.8;
  }

  if (key == 'n') {
    // TBD: find the 10 nearest neighbors of (x_center, y_center)
    nearest(root, x_center, y_center, 0);
    // Then store them in a global structure so they can be highlighted in the rendering code
  }

  if (key == 's') { // set source
    // TBD: set source_node = nearest neighbor in (x,y) plane to (x_center, y_center)
    nearest(root, x_center, y_center, 0);
    while (!pq.empty()){
      if (pq.size() == 1){
        source_node = pq.top().second;
      }
      pq.pop();
    }
    cout << "Set source node = " << source_node << "\n";
  }
  if (key == 'd') { // set destination and compute shortest path
    // TBD: set destination_node = nearest neighbor in (x,y) plane to (x_center, y_center)
    nearest(root, x_center, y_center, 0);
    while (!pq.empty()){
      if (pq.size() == 1){
        destination_node = pq.top().second;
      }
      pq.pop();
    }
    cout << "Set destination node = " << destination_node << "\n";
  }
  if ((key == 's' || key == 'd') && source_node != -1 && destination_node != -1) {
    cout << "Computing shortest path from source to destination...\n";
    // TBD: compute shortest path from source to destination and record it so it can be visualized
    // if no path (e.g., all paths would require stepping at more than a 45 degree incline, print "No path"
    // if there is a path, print its length in feet
    if (dijkstra() == true){
      cout << "Shortest Distance: " << dists[destination_node] << endl;
    }
    else cout << "No Path" << endl;
  }

  if (key == 'q') exit(0);
}

// Returns where on the screen to plot a point, offsetting it slightly based on elevation
// from its usual (x,y) location to make tall things look tall
pair<double,double> get_point_screen_location(Point &p)
{
  double x = (p.x - winmin.x) / (winmax.x - winmin.x) * window_xsize;
  double y = window_ysize - (p.y - winmin.y) / (winmax.y - winmin.y) * window_ysize;
  double offset = (p.z - 700) / 5; // how much we offset pixel to emphasize its height
  return make_pair(x-offset, y-offset);
}

// Called whenever we need to re-render the current window
void render(void)
{
  // Feel welcome to turn off this message if you want...
  cout << "Rendering (" << winmin.x << "," << winmin.y << ") - (" << winmax.x << "," << winmax.y << ")\n";

  for (Point &p : all_points) {
    pair<double, double> loc = get_point_screen_location(p);
    double x = loc.first, y = loc.second;
    if (x >= 0 && y >= 0 && x < window_xsize && y < window_ysize) {
      double color = min(1.0, max(0.0, (p.z - 700) / 80.0)); // color in range [0,1] based on height
      set_color (color, 0.8, 0.5);
      draw_pixel (x, y);
    }
  }

  // Draw small crosshairs at center of window
  set_color(1,1,1);
  draw_line(window_xsize/2-3, window_ysize/2, window_xsize/2+3, window_ysize/2);
  draw_line(window_xsize/2, window_ysize/2-3, window_xsize/2, window_ysize/2+3);

  // TBD: highlight the points that were returned in response to asking for nearest neighbors
  pair <double, double> pos;
  set_color(1,1,1);
  while (!pq.empty()) {
    pos = get_point_screen_location(all_points[pq.top().second]);
    draw_line(pos.first-5, pos.second-5, pos.first+5, pos.second+5);
    draw_line(pos.first-5, pos.second+5, pos.first+5, pos.second-5);
    pq.pop();
  }
  // TBD: plot a sequence of line segments depicting the shortest path, if one has been computed
  int dest;
  int sour;
  pair<double,double> curr_loc;
  pair<double,double> next_loc;
  set_color(1,1,1);
  if (source_node != -1){
    pair<double, double> loc = get_point_screen_location(all_points[source_node]);
    double x = loc.first, y = loc.second;
    draw_line(x-5, y-5, x+5, y+5);
    draw_line(x-5, y+5, x+5, y-5);
  }
  if (destination_node != -1){
    pair<double, double> loc = get_point_screen_location(all_points[destination_node]);
    double x = loc.first, y = loc.second;
    draw_line(x-5, y-5, x+5, y+5);
    draw_line(x-5, y+5, x+5, y-5);
  }
  if (source_node != -1 && destination_node != -1){
    if (line){
      dest = destination_node;
      set_color(1,1,1);
      while (dest != source_node){
        curr_loc = get_point_screen_location(all_points[dest]);
        next_loc = get_point_screen_location(all_points[pred[dest]]);
        draw_line(curr_loc.first, curr_loc.second, next_loc.first, next_loc.second);
        dest = pred[dest];
      }
    }
  }
}

int main(int argc, char *argv[])
{
  // Read in all the data
  ifstream fin ("points.txt");
  Point p;
  int count = 0;
  while (fin >> p.x >> p.y >> p.z)
    all_points.push_back(p);

  // Find initial view window corner points
  mins = maxes = all_points[0];
  for (auto &p : all_points) {
    mins.x = min(mins.x, p.x);
    mins.y = min(mins.y, p.y);
    mins.z = min(mins.z, p.z);
    maxes.x = max(maxes.x, p.x);
    maxes.y = max(maxes.y, p.y);
    maxes.z = max(maxes.z, p.z);
  }
  // Re-adjust y to match aspect ratio of window
  winmin = mins; winmax = maxes;
  winmax.y = winmin.y + (winmax.x - winmin.x) * window_ysize / window_xsize;

  // TBD... build kd-tree here
  // Normally you'd want to insert them in random order but they are already randomly shuffled, so no need to shuffle again
  for (unsigned int i = 0; i < all_points.size(); i++){
    root = insert(0, all_points[i], root, i);
  }
  // Launch graphics; all remaining interaction is via callback to render and keyhandler function
  init_graphics(argc, argv, window_xsize, window_ysize, render, keyhandler);
  return 0;
}
