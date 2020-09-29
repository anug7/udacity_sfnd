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
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

  void insertHelper(Node **ref, std::vector<float> points, int id, int depth){
    
    if(*ref == nullptr){
      *ref = new Node(points, id);
    }else{
      int sIdx = depth % 2;
      if(points[sIdx] < (*ref)->point[sIdx]){
        insertHelper(&((*ref)->left), points, id, depth + 1);
      }else{
        insertHelper(&((*ref)->right), points, id, depth + 1);
      }
    }
  }

  void searchHelper(Node *ref, std::vector<int> &ids, std::vector<float> target, 
                    float distanceTol, int depth)
  {
    if(ref != nullptr){
      if( target[0] >= (ref->point[0] - distanceTol) && target[0] <= (ref->point[0] + distanceTol)
          && target[1] >= (ref->point[1] - distanceTol) && target[1] <= (ref->point[1] + distanceTol)){
        float diffx = ref->point[0] - target[0],
              diffy = ref->point[1] - target[1];
        float dist = sqrt(diffx * diffx + diffy * diffy);
        if (dist <= distanceTol){
          ids.push_back(ref->id);
        }
      }
      int sIdx = depth % 2;
      if(target[sIdx] - distanceTol < ref->point[sIdx])
        searchHelper(ref->left, ids, target, distanceTol, depth + 1);
      if(target[sIdx] + distanceTol > ref->point[sIdx])
        searchHelper(ref->right, ids, target, distanceTol, depth + 1);
    }
  }

	void insert(std::vector<float> point, int id)
	{
    insertHelper(&root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
    searchHelper(root, ids, target, distanceTol, 0);
		return ids;
	}
	

};




