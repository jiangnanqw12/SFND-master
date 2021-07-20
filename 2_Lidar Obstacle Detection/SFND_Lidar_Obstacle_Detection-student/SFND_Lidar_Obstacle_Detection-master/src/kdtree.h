/* \author Aaron Brown */
// Quiz on implementing kd tree
#ifndef kdtree_h
#define kdtree_h
//#include "../../render/render.h"
#include "render/box.h"
#include "render/render.h"
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
    void insertHelper(Node **node, uint depth, std::vector<float> point, int id);
    void insertHelper_ref(Node *&node, uint depth, std::vector<float> point, int id);
    void insert(std::vector<float> point, int id)
    {
        //printf("insert1\n");
        insertHelper_ref(root, 0, point, id);
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
    }
#define flag_test_search 0

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
#define Clustertest 0
void euclideanClusterHelper_student(const std::vector<std::vector<float>> &points,
                                    std::vector<int> &cluster, std::vector<bool> &flag_process,
                                    int id, KdTree *tree, float distanceTol);
void euclideanClusterHelper_solution(int indice, const std::vector<std::vector<float>> points,
                                     std::vector<int> &cluster, std::vector<bool> &processed,
                                     KdTree *tree, float distanceTol);
void render2DTree(Node *node, pcl::visualization::PCLVisualizer::Ptr &viewer, Box window, int &iteration, uint depth = 0);
#endif //kdtree.h