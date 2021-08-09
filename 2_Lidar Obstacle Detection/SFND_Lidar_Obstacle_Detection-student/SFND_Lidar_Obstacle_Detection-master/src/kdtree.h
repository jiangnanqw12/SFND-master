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
public:
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
public:
    Node<PointT> *root;

    KdTree_euclidean()
        : root(NULL)
    {
    }
    void test1();
    void test2()
    {
        test1();
    };
    //void insertHelper(typename Node<PointT>::Node **node, uint depth, int id);
    //void insertHelper(typename Node<PointT>::Node **node, uint depth, PointT point, int id);
    // //template <typename PointT>
    void insertHelper(Node<PointT> **node, uint depth, PointT point, int id)
    {
        //printf("insert2\n");
        if (*node == NULL)
        {
            *node = new Node<PointT>(point, id);
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
            unsigned int cd = depth % 3;
            if (cd == 0)
            {
                if ((*node)->point.x <= point.x)
                {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
                else
                {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                }
            }
            else if (cd == 1)
            {
                if ((*node)->point.y <= point.y)
                {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
                else
                {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                }
            }
            else if (cd == 2)
            {
                if ((*node)->point.z <= point.z)
                {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
                else
                {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                }
            }
        }
    }
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
                      float distanceTol, std::vector<int> &ids)
    {
        uint cd = depth % 3;
#if flag_test_search
        printf("search 1\n");
#endif
        if ((*node) != NULL)
        {
#if flag_test_search
            printf("search 2\n");
#endif
            if ((*node)->point.x >= (target.x - distanceTol) && //left boundary
                (*node)->point.x <= (target.x + distanceTol) && //right boundary
                (*node)->point.y >= (target.y - distanceTol) && //up boundary
                (*node)->point.y <= (target.y + distanceTol) && //down boundary
                (*node)->point.z >= (target.z - distanceTol) &&
                (*node)->point.z <= (target.z + distanceTol))
            {
#if flag_test_search
                printf("search 2.1\n");
#endif

                float dist =
                    sqrt(((*node)->point.x - target.x) * ((*node)->point.x - target.x) +
                         ((*node)->point.y - target.y) * ((*node)->point.y - target.y) +
                         ((*node)->point.z - target.z) * ((*node)->point.z - target.z));

                if (dist <= distanceTol)
                {
#if flag_test_search
                    printf("search 2.2\n");
#endif
                    ids.push_back((*node)->id);
                }
            }
            if (cd == 0)
            {
                if (((*node)->point.x) > (target.x - distanceTol))
                {

                    searchHelper(target, &((*node)->left), depth + 1, distanceTol, ids);
                }
                if (((*node)->point.x) < (target.x + distanceTol))
                {

                    searchHelper(target, &((*node)->right), depth + 1, distanceTol, ids);
                }
            }
            if (cd == 1)
            {
                if (((*node)->point.y) > (target.y - distanceTol))
                {

                    searchHelper(target, &((*node)->left), depth + 1, distanceTol, ids);
                }
                if (((*node)->point.y) < (target.y + distanceTol))
                {

                    searchHelper(target, &((*node)->right), depth + 1, distanceTol, ids);
                }
            }
            if (cd == 2)
            {
                if (((*node)->point.z) > (target.z - distanceTol))
                {

                    searchHelper(target, &((*node)->left), depth + 1, distanceTol, ids);
                }
                if (((*node)->point.z) < (target.z + distanceTol))
                {

                    searchHelper(target, &((*node)->right), depth + 1, distanceTol, ids);
                }
            }
        }
    }

    void searchHelper_solution(PointT target, typename Node<PointT>::Node **node, int depth,
                               float distanceTol, std::vector<int> &ids);

    // return a list of point ids in the tree that are within distance of target
    //void searchHelper();
    //std::vector<int> search(typename pcl::PointCloud<PointT>::Ptr cloud, int id, float distanceTol);

    std::vector<int> search(PointT target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, &root, 0, distanceTol, ids);
        //searchHelper();
        return ids;
    }
};

#endif //kdtree.h