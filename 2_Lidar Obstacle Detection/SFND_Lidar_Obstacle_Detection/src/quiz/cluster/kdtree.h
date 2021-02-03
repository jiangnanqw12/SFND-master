/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, 0, point, id);
	}
	void insertHelper(Node **nodePP, uint depth, std::vector<float> point, int id)
	{
		//if tree is empty
		if (*nodePP == NULL)
		{
			*nodePP = new Node(point, id);
		}
		else
		{
			uint cd = depth % 2;
			if (point[cd] < (*nodePP)->point[cd])
			{
				insertHelper(&((*nodePP)->left), depth + 1, point, id);
			}
			else
			{
				insertHelper(&((*nodePP)->right), depth + 1, point, id);
			}
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		std::cout << "1" << std::endl;
		searchHelper(target, distanceTol, ids, 0, root);
		return ids;
	}
	std::vector<int> searchHelper(std::vector<float> target, float distanceTol, std::vector<int> &ids, int depth, Node *node)
	{
		// float x0 = target[0];
		// float y0 = target[1];
		// float x = node->point[0];
		// float y = node->point[1];
		if (node != NULL)
		{
			std::cout << "2" << std::endl;

			if ((node->point[0] >= target[0] - distanceTol) && (node->point[0] <= target[0] + distanceTol) && (node->point[1] >= target[1] - distanceTol) && (node->point[1] <= target[1] + distanceTol))
			{
				std::cout << "id1: " << node->id << std::endl;
				float dist = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]));
				if (dist <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}
			//x<= left boundary ,the left of select point wont have chance to be in the boxes
			if (node->point[depth % 2] > target[depth % 2] - distanceTol)
			{
				std::cout << "id2: " << node->id << std::endl;
				searchHelper(target, distanceTol, ids, depth + 1, node->left);
				std::cout << "id22: " << node->id << std::endl;
			}
			if (node->point[depth % 2] < target[depth % 2] + distanceTol)
			{
				std::cout << "id3: " << node->id << std::endl;
				searchHelper(target, distanceTol, ids, depth + 1, node->right);
			}
		}
	}
};
