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
    //void insertHelper(Node<PointT> **node, uint depth, PointT point, int id);
    //template <typename PointT>
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
        //     uint cd = depth % 2;
        // #if flag_test_search
        //     printf("search 1\n");
        // #endif
        //     if ((*node) != NULL)
        //     {
        // #if flag_test_search
        //         printf("search 2\n");
        // #endif
        //         if ((*node)->point[0] >= (target[0] - distanceTol) && //left boundary
        //             (*node)->point[0] <= (target[0] + distanceTol) && //right boundary
        //             (*node)->point[1] >= (target[1] - distanceTol) && //up boundary
        //             (*node)->point[1] <= (target[1] + distanceTol))   //down boundary
        //         {
        // #if flag_test_search
        //             printf("search 2.1\n");
        // #endif
        //             float dist =
        //                 sqrt(((*node)->point[0] - target[0]) * ((*node)->point[0] - target[0]) +
        //                      ((*node)->point[1] - target[1]) * ((*node)->point[1] - target[1]));
        //             if (dist <= distanceTol)
        //             {
        // #if flag_test_search
        //                 printf("search 2.2\n");
        // #endif
        //                 ids.push_back((*node)->id);
        //             }
        //         }

        //         if (((*node)->point[cd]) > (target[cd] - distanceTol))
        //         {
        // #if flag_test_search
        //             printf("search 3\n");
        // #endif
        //             searchHelper(target, &((*node)->left), depth + 1, distanceTol, ids);
        //         }
        //         if (((*node)->point[cd]) < (target[cd] + distanceTol))
        //         {
        // #if flag_test_search
        //             printf("search 4\n");
        // #endif
        //             searchHelper(target, &((*node)->right), depth + 1, distanceTol, ids);
        //         }
        //     }
    }

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