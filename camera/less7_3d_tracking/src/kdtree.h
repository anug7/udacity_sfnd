// Structure to represent node of kd tree

#include <vector>
#include <opencv2/opencv.hpp>

struct Node
{
  cv::KeyPoint point;
	int id;
	Node* left;
	Node* right;

	Node(cv::KeyPoint arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};


struct KdTree
{
	Node *root;
  int dim;

	KdTree()
	: root(NULL), dim(2) 
	{
  }
  ~KdTree(){
  }

  void insertHelper(Node **ref, cv::KeyPoint &points, int id, int depth){
    if(*ref == nullptr){
      *ref = new Node(points, id);
    }else{
      float qry, cur;
      int sIdx = depth % dim;
      if(sIdx){
          qry = points.pt.y;
          cur = (*ref)->point.pt.y;
      }else{
          qry = points.pt.x;
          cur = (*ref)->point.pt.x;
      }
      if(qry < cur){
        insertHelper(&((*ref)->left), points, id, depth + 1);
      }else{
        insertHelper(&((*ref)->right), points, id, depth + 1);
      }
    }
  }

  void searchHelper(Node *ref, std::vector<int> &ids, cv::KeyPoint &target, 
                    float distanceTol, int depth)
  {
    if(ref != nullptr){
      if (target.pt.x >= ref->point.pt.x - distanceTol && target.pt.x <= ref->point.pt.x + distanceTol &&
          target.pt.y >= ref->point.pt.y - distanceTol && target.pt.y <= ref->point.pt.y + distanceTol){
          ids.push_back(ref->id);
      }
      
      float qry, cur;
      int sIdx = depth % dim;
      if(sIdx){
          qry = target.pt.y;
          cur = ref->point.pt.y;
      }else{
          qry = target.pt.x;
          cur = ref->point.pt.x;
      }
      if(qry - distanceTol < cur)
        searchHelper(ref->left, ids, target, distanceTol, depth + 1);
      if(qry + distanceTol > cur)
        searchHelper(ref->right, ids, target, distanceTol, depth + 1);
    }
  }

	void insert(cv::KeyPoint &point, int id)
	{
    insertHelper(&root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
  std::vector<int> search(cv::KeyPoint &target, float distanceTol)
	{
    std::vector<int> ids;
    searchHelper(root, ids, target, distanceTol, 0);
		return ids;
	}
	
};

