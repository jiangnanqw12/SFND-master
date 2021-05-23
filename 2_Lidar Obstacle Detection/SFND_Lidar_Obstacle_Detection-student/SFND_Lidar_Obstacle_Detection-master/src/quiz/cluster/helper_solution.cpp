#include "kdtree.h"
void KdTree::searchHelper_solution(std::vector<float> target, Node **node, int depth,
                                   float distanceTol, std::vector<int> &ids)
{
    uint cd = depth % 2;
#if flag_test_search
    printf("search 1\n");
#endif
    if ((*node) != NULL)
    {
#if flag_test_search
        printf("search 2\n");
#endif
        if ((*node)->point[0] >= (target[0] - distanceTol) && //left boundary
            (*node)->point[0] <= (target[0] + distanceTol) && //right boundary
            (*node)->point[1] >= (target[1] - distanceTol) && //up boundary
            (*node)->point[1] <= (target[1] + distanceTol))   //down boundary
        {
#if flag_test_search
            printf("search 2.1\n");
#endif
            float distance =
                sqrt(((*node)->point[0] - target[0]) * ((*node)->point[0] - target[0]) +
                     ((*node)->point[1] - target[1]) * ((*node)->point[1] - target[1]));

            if (distance <= distanceTol)
            {
#if flag_test_search
                printf("search 2.2\n");
#endif
                ids.push_back((*node)->id);
            }
        }

        // check accross boundary
        if ((target[cd] - distanceTol) < (*node)->point[cd])
        {
#if flag_test_search
            printf("search 3\n");
#endif
            searchHelper_solution(target, &((*node)->left), depth + 1, distanceTol, ids);
        }

        if ((target[cd] + distanceTol) > (*node)->point[cd])
        {
#if flag_test_search
            printf("search 4\n");
#endif
            searchHelper_solution(target, &((*node)->right), depth + 1, distanceTol, ids);
        }
    }
}