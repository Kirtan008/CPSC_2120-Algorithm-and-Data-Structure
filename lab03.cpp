/*
Name: Kirtan Patel
*/

#include <iostream>
#include <fstream>
#include "lab03.h"
#include <math.h>
using namespace std;

class point{
  double x, y;
};

Table::Table(){

}

Table::Table(int s){
  size = s;
  table = new Node**[size];
  for (int i = 0; i < size; ++i)
  {
    table[i] = new Node* [size];
    for (int j = 0; j < size; ++j)
    {
      table[i][j] = NULL;
    }
  }
}

Table::~Table(){
  for (int i=0; i < size; i++)
  {
      for(int j = 0; j < size; j++)
      {
        while (table[i][j] != NULL)
        {
          Node *temp = table[i][j];
          table[i][j] = table[i][j]->next;
          delete temp;
        }
      }
      delete[] table[i];
  }
  delete[] table;
}

int Table::myhashX(Point p){
  int i = 1;
  int new_x = i*(p.x * size);
  return new_x;
}

int Table::myhashY(Point p){
  int i = 1;
  int new_y = i*(p.y * size);
  return new_y;
}

void Table::read(){
  ifstream input("points.txt");
  Point p;
  while(input >> p.x >> p.y){
    int i = myhashX(p);
    }
  }
}

double Table::distance(Point p1, Point p2){
  return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}

double Table::closestDist(){
  double currDist = 1;
  for(int i = 0; i < size; i++){
    for(int j = 0; j < size; j++){
      Node* temp = table[i][j];
      while(temp != NULL){
        for(int k = i-1; k < i+2; k++){
          for(int l = j-1; l < j+2;l++){
            if(k < 0 || l < 0) continue;
            if(k > size-1 || l > size-1) continue;
            Node* curr = table[k][l];
            while(curr != NULL){
              if(curr == temp) {curr = curr->next; continue;}
              double dist = distance(curr->getPoint(), temp->getPoint());
              if(dist < currDist){
                currDist = dist;
              }
              curr = curr->next;
            }
          }
        }
        temp = temp->next;
      }
    }
  }
  return currDist;
}

int main(){
  Table t(2000);
  t.read();
  cout << t.closestDist() << endl;
}
