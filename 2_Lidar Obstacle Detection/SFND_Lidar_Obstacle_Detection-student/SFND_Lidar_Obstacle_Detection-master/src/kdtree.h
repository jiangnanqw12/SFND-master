/* \author Aaron Brown */
// Quiz on implementing kd tree
#ifndef kdtree_h
#define kdtree_h
//#include "../../render/render.h"
#include "render/box.h"
#include "render/render.h"
//#include "helper.h"
// Structure to represent node of kd tree
template <typename PointT>
struct Node
{
    //std::vector<float> point;
    PointT point;
    int id;
    Node *left;
    Node *right;

    Node(PointT arr, int setId)
        : point(arr), id(setId), left(NULL), right(NULL)
    {
    }
};
template <typename PointT>
struct KdTree_euclidean
{
    Node<PointT> *root;

    KdTree_euclidean()
        : root(NULL)
    {
    }
    void insertHelper(Node<PointT> **node, uint depth, PointT point, int id);
    void insertHelper_ref(Node<PointT> *&node, uint depth, PointT point, int id);

    void insert(PointT point, int id)
    {
        //printf("insert1\n");
        insertHelper(&root, 0, point, id);
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
    }
#define flag_test_search 0

    void searchHelper(PointT target, Node<PointT> **node, int depth,
                      float distanceTol, std::vector<int> &ids);
    void searchHelper_solution(PointT target, Node<PointT> **node, int depth,
                               float distanceTol, std::vector<int> &ids);

    // return a list of point ids in the tree that are within distance of target

    std::vector<int> search(PointT target, float distanceTol)
    {
        std::vector<int> ids;
        //searchHelper(target, &root, 0, distanceTol, ids);
        searchHelper(target, &root, 0, distanceTol, ids);
        return ids;
    }
};

#endif //kdtree.h