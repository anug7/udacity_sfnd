/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <pcl/point_representation.h>
#include "../../render/render.h"


// Structure to represent node of kd tree
template <typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};


template <typename PointT>
struct KdTree
{
	Node<PointT> *root;
  int dim;

	KdTree()
	: root(NULL), dim(3),
    ppr(new pcl::DefaultPointRepresentation<PointT>)
	{
      dim = ppr->getNumberOfDimensions();
  }
  ~KdTree(){
  }

  void insertHelper(Node<PointT> **ref, PointT points, int id, int depth){
    if(*ref == nullptr){
      *ref = new Node<PointT>(points, id);
    }else{
      int sIdx = depth % dim;
      std::vector<float> tmp(dim), tmpq(dim);
      ppr->vectorize(static_cast<PointT>(points), tmpq);
      ppr->vectorize(static_cast<PointT>((*ref)->point), tmp);

      if(tmpq[sIdx] < tmp[sIdx]){
        insertHelper(&((*ref)->left), points, id, depth + 1);
      }else{
        insertHelper(&((*ref)->right), points, id, depth + 1);
      }
    }
  }

  void searchHelper(Node<PointT> *ref, pcl::PointIndices::Ptr ids, PointT target, 
                    float distanceTol, int depth)
  {
    if(ref != nullptr){
      std::vector<float> tmp(dim), tmpq(dim);
      ppr->vectorize(static_cast<PointT>(target), tmpq);
      ppr->vectorize(static_cast<PointT>(ref->point), tmp);
      
      int flag = 0;
      float dist = 0.0;
      for (int i = 0; i < dim; i++){
        flag += (tmpq[i] >= (tmp[i] - distanceTol) && tmpq[i] <= (tmp[i] +
                        distanceTol));
        dist += (tmpq[i] - tmp[i]) * (tmpq[i] - tmp[i]);
      }
      dist = sqrt(dist);
      if (flag == dim && dist <= distanceTol){
        ids->indices.push_back(ref->id);
      }
      
      int sIdx = depth % dim;
      if(tmpq[sIdx] - distanceTol < tmp[sIdx])
        searchHelper(ref->left, ids, target, distanceTol, depth + 1);
      if(tmpq[sIdx] + distanceTol > tmp[sIdx])
        searchHelper(ref->right, ids, target, distanceTol, depth + 1);
    }
  }

	void insert(PointT point, int id)
	{
    insertHelper(&root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
  pcl::PointIndices::Ptr search(PointT target, float distanceTol)
	{
    pcl::PointIndices::Ptr ids(new pcl::PointIndices);
    searchHelper(root, ids, target, distanceTol, 0);
		return ids;
	}
	
  private:
     typename pcl::PointRepresentation<PointT>::ConstPtr ppr;

};
