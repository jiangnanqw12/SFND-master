/* \author Aaron Brown */
// Quiz on implementing kd tree
#ifndef kdtree_h
#define kdtree_h
#include "../../render/render.h"

//#include "helper.h"
// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node *left;
    Node *right;

    Node(std::vector<float> arr, int setId)
        : point(arr), id(setId), left(NULL), right(NULL)
    {
    }
};

struct KdTree
{
    Node *root;

    KdTree()
        : root(NULL)
    {
    }
    void insertHelper(Node **node, uint depth, std::vector<float> point, int id)
    {
        //printf("insert2\n");
        if (*node == NULL)
        {
            *node = new Node(point, id);
        }
        // else
        // {
        //     unsigned int cd = depth % 2;

        //     if (point[cd] < ((*node)->point[cd]))
        //         insertHelper(&((*node)->left), depth + 1, point, id);
        //     else
        //         insertHelper(&((*node)->right), depth + 1, point, id);
        // }
        else
        {
            unsigned int cd = depth % 2;
            if ((*node)->point[cd] <= point[cd])
            {
                insertHelper(&((*node)->right), depth + 1, point, id);
            }
            else
            {
                insertHelper(&((*node)->left), depth + 1, point, id);
            }
        }
        // else if (root->point[depth % 2] < point[depth % 2])
        // {
        //     insertHelper(&((*node)->right), depth + 1, point, id);
        // }

        // else
        // {
        //     insertHelper(&((*node)->left), depth + 1, point, id);
        // }
        //printf("insert3\n");
    }
    void insert(std::vector<float> point, int id)
    {
        //printf("insert1\n");
        insertHelper(&root, 0, point, id);
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
    }
#define flag_test_search 1

    void searchHelper(std::vector<float> target, Node **node, int depth,
                      float distanceTol, std::vector<int> &ids);
    void searchHelper_solution(std::vector<float> target, Node **node, int depth,
                               float distanceTol, std::vector<int> &ids);

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        //searchHelper(target, &root, 0, distanceTol, ids);
        searchHelper(target, &root, 0, distanceTol, ids);
        return ids;
    }
};
#endif //kdtree.h