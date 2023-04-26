/* \author Aaron Brown */

// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree

struct Node

{

std::vector<float> point;

int id;

Node* left;

Node* right;

Node(std::vector<float> arr, int setId)

: point(arr), id(setId), left(NULL), right(NULL)

{}

};

struct KdTree

{

Node* root;

KdTree()

: root(NULL)

{}

void inserthelper(Node** node,unsigned int dep,std::vector<float> point,int id){

if( *node==NULL ) {*node = new Node(point,id);}

else{

unsigned int Three_D = dep % 3;

if(point[Three_D] < ((*node)->point[Three_D]))inserthelper(&((*node)->left),dep+1,point,id);

else inserthelper(&((*node)->right),dep+1,point,id); }}

void insert(std::vector<float> point, int id)

{

inserthelper(&root,0,point, id);

}

void inserthelper_2(std::vector<float> target,Node* R,int dep,float distance,std::vector<int>& ids){

if(R !=NULL){

if(  (R->point[0] <= (target[0] + distance)) && (R->point[0] >= (target[0] - distance)) && (R->point[1] <= (target[1] + distance)) && (R->point[1] >= (target[1] - distance))&&(R->point[2] <= (target[2] + distance)) && (R->point[2] >= (target[2] - distance)))
{
  
/*&& abs(abs(R->point[0])-abs(target[0]))<=distance && abs(abs((R->point[1])-abs(target[1])))<=distance)*/

//float distance_in_x = (R->point[0])-(target[0]);

//float distance_in_y = (R->point[1])-(target[1]);

float distance_Between =sqrt(((R->point[0])-(target[0]))*((R->point[0])-(target[0]))*((R->point[1])-(target[1]))*((R->point[1])-(target[1]))*((R->point[2])-(target[2]))*((R->point[2])-(target[2])));

if(distance_Between<=distance) ids.push_back(R->id);
  
}

if((target[dep%3]-distance) < R->point[dep%3])

inserthelper_2(target,R->left,dep+1,distance,ids);

if((target[dep%3]+distance) > R->point[dep%3])

inserthelper_2(target,R->right,dep+1,distance,ids);


}
   }
// return a list of point ids in the tree that are within distance of target

std::vector<int> search(std::vector<float> target, float distanceTol){

std::vector<int> ids;

inserthelper_2(target,root,0,distanceTol,ids);

return ids;

}

};