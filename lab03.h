#include <fstream>
#include <iostream>
#include <iomanip>
using namespace std;

class Table
{
private:

 int size;

 struct Point
  {
    double x;
    double y;
    Point(double a, double b) { x = a; y = b; }
    Point() { }
  };

  struct Node
  {
    Point p;
    Node *next;
    Node(Point p1, Node *n) { p = p1; next = n; }
    Node() { }
    Point getPoint() {return p;}
  };

  Node*** table;

public:
  Table();
  Table(int);
  ~Table();
  void read();
  int myhashX(Point);
  int myhashY(Point);
  double distance(Point, Point);
  double closestDist();
};
