#include "kdtree.h"
#include "pcl/impl/point_types.hpp"

// template <typename PointT>
// void KdTree_euclidean<PointT>::searchHelper(PointT target, Node<PointT> **node, int depth,
//                                             float distanceTol, std::vector<int> &ids)
// {
//     //     uint cd = depth % 2;
//     // #if flag_test_search
//     //     printf("search 1\n");
//     // #endif
//     //     if ((*node) != NULL)
//     //     {
//     // #if flag_test_search
//     //         printf("search 2\n");
//     // #endif
//     //         if ((*node)->point[0] >= (target[0] - distanceTol) && //left boundary
//     //             (*node)->point[0] <= (target[0] + distanceTol) && //right boundary
//     //             (*node)->point[1] >= (target[1] - distanceTol) && //up boundary
//     //             (*node)->point[1] <= (target[1] + distanceTol))   //down boundary
//     //         {
//     // #if flag_test_search
//     //             printf("search 2.1\n");
//     // #endif
//     //             float dist =
//     //                 sqrt(((*node)->point[0] - target[0]) * ((*node)->point[0] - target[0]) +
//     //                      ((*node)->point[1] - target[1]) * ((*node)->point[1] - target[1]));
//     //             if (dist <= distanceTol)
//     //             {
//     // #if flag_test_search
//     //                 printf("search 2.2\n");
//     // #endif
//     //                 ids.push_back((*node)->id);
//     //             }
//     //         }

//     //         if (((*node)->point[cd]) > (target[cd] - distanceTol))
//     //         {
//     // #if flag_test_search
//     //             printf("search 3\n");
//     // #endif
//     //             searchHelper(target, &((*node)->left), depth + 1, distanceTol, ids);
//     //         }
//     //         if (((*node)->point[cd]) < (target[cd] + distanceTol))
//     //         {
//     // #if flag_test_search
//     //             printf("search 4\n");
//     // #endif
//     //             searchHelper(target, &((*node)->right), depth + 1, distanceTol, ids);
//     //         }
//     //     }
// }
// template <typename PointT>
// void KdTree_euclidean<PointT>::insertHelper(Node<PointT> **node, uint depth, PointT point, int id)
// {
//     //printf("insert2\n");
//     if (*node == NULL)
//     {
//         *node = new Node<PointT>(point, id);
//     }
//     // else
//     // {
//     //     unsigned int cd = depth % 2;

//     //     if (point[cd] < ((*node)->point[cd]))
//     //         insertHelper(&((*node)->left), depth + 1, point, id);
//     //     else
//     //         insertHelper(&((*node)->right), depth + 1, point, id);
//     // }
//     else
//     {
//         unsigned int cd = depth % 3;
//         if (cd == 0)
//         {
//             if ((*node)->point.x <= point.x)
//             {
//                 insertHelper(&((*node)->right), depth + 1, point, id);
//             }
//             else
//             {
//                 insertHelper(&((*node)->left), depth + 1, point, id);
//             }
//         }
//         else if (cd == 1)
//         {
//             if ((*node)->point.y <= point.y)
//             {
//                 insertHelper(&((*node)->right), depth + 1, point, id);
//             }
//             else
//             {
//                 insertHelper(&((*node)->left), depth + 1, point, id);
//             }
//         }
//         else if (cd == 2)
//         {
//             if ((*node)->point.z <= point.z)
//             {
//                 insertHelper(&((*node)->right), depth + 1, point, id);
//             }
//             else
//             {
//                 insertHelper(&((*node)->left), depth + 1, point, id);
//             }
//         }
//     }
//     // else if (root->point[depth % 2] < point[depth % 2])
//     // {
//     //     insertHelper(&((*node)->right), depth + 1, point, id);
//     // }

//     // else
//     // {
//     //     insertHelper(&((*node)->left), depth + 1, point, id);
//     // }
//     //printf("insert3\n");
// }
// template <typename PointT>
// void KdTree_euclidean<PointT>::insertHelper_ref(Node<PointT> *&node,
//                                                 uint depth, PointT point, int id)
// {
//     if (node == NULL)
//     {
//         node = new Node<PointT>(point, id);
//     }

//     else
//     {
//         unsigned int cd = depth % 3;
//         if (cd == 0)
//         {
//             if ((node)->point.x <= point.x)
//             {
//                 insertHelper_ref(((node)->right), depth + 1, point, id);
//             }
//             else
//             {
//                 insertHelper_ref(((node)->left), depth + 1, point, id);
//             }
//         }
//         else if (cd == 1)
//         {
//             if ((node)->point.y <= point.y)
//             {
//                 insertHelper_ref(((node)->right), depth + 1, point, id);
//             }
//             else
//             {
//                 insertHelper_ref(((node)->left), depth + 1, point, id);
//             }
//         }
//         else if (cd == 2)
//         {
//             if ((node)->point.z <= point.z)
//             {
//                 insertHelper_ref(((node)->right), depth + 1, point, id);
//             }
//             else
//             {
//                 insertHelper_ref(((node)->left), depth + 1, point, id);
//             }
//         }
//     }
// }
