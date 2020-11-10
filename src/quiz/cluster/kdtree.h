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

	void inserthelper(Node **Root, int depth, std::vector<float> vpoint, int idvalue)
	{
		if (*Root == NULL)
		{
			*Root = new struct Node(vpoint, idvalue);
		}
		uint cd = depth % 2;
		if (vpoint[cd] < (*Root)->point[cd])
		{
			inserthelper(&(*Root)->left, depth + 1, vpoint, idvalue);
		}
		else if (vpoint[cd] > (*Root)->point[cd])
		{
			inserthelper(&(*Root)->right, depth + 1, vpoint, idvalue);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		inserthelper(&root, 0, point, id);
	}

	void searchHelper(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int> &ids)
	{
		if (node != NULL)
		{
			auto x = node->point[0];
			auto y = node->point[1];
			auto tx = target[0];
			auto ty = target[1];

			if (x >= tx - distanceTol && x <= tx + distanceTol && y >= ty - distanceTol && y <= ty + distanceTol)
			{
				float distance = sqrt((x - tx) * (x - tx) + (y - ty) * (y - ty));
				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			// Split left or right depending on depth and recurse
			auto split_lr = depth % 2;
			if (target[split_lr] - distanceTol < node->point[split_lr])
				searchHelper(target, node->left, depth + 1, distanceTol, ids);
			if (target[split_lr] + distanceTol > node->point[split_lr])
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		// searchhelper(target, root, 0, distanceTol, ids);

		return ids;
	}
	void searchhelper(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int> &ids)
	{

		if (node != NULL)
		{
			auto x = node->point[0];
			auto y = node->point[1];
			auto tx = target[0];
			auto ty = target[1];

			if (x >= tx - distanceTol && x <= tx + distanceTol && y >= ty + distanceTol && y <= ty + distanceTol)
			{
				float distance = sqrt((x - tx) * (x - tx) + (y - ty) * (y - ty));
				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}
		
		auto cd = depth % 2;
		if (target[cd] - distanceTol < node->point[cd])
		{
			searchhelper(target, node->left, depth + 1, distanceTol, ids);
		}
		else if (target[cd] + distanceTol > node->point[cd])
		{
			searchhelper(target, node->right, depth + 1, distanceTol, ids);
		}
	}
	}
};
