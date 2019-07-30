
// Structure to represent node of kd tree
struct TreeNode
{
	std::vector<float> point;
	int id;
	TreeNode* left;
	TreeNode* right;

	TreeNode(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	TreeNode* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		TreeNode* tmp = new TreeNode(point,id);
		// the function should create a new node and place correctly with in the root 
		int depth=0;
		TreeNode** cmp = &root;
		while((*cmp)!=NULL)
		{
			int index =  depth%3;
			if(point[index]<(*cmp)->point[index])
			{
				cmp=&((*cmp)->left);
			}
			else
			{
				cmp=&((*cmp)->right);
			}
			depth++;
		}
		*cmp = tmp;
	}

	void searchHelper(TreeNode* tmp,int depth,float distanceTol,std::vector<int> &ids,std::vector<float> target)
	{
		if(tmp==NULL)
			return ;
		if(fabs(target[0]-tmp->point[0])<=distanceTol && fabs(target[1]-tmp->point[1])<=distanceTol && fabs(target[2]-tmp->point[2])<=distanceTol)
		{
			auto distance = sqrt((target[0]-tmp->point[0])*(target[0]-tmp->point[0]) + (target[1] - tmp->point[1])*(target[1] - tmp->point[1]) + (target[2] - tmp->point[2])*(target[2] - tmp->point[2]));
			if(distance <=distanceTol)
			{
				ids.push_back(tmp->id);
			}
		}
		if(target[depth%3]-distanceTol < tmp->point[depth%3])
			searchHelper(tmp->left,depth+1,distanceTol,ids,target);
		if(target[depth%3]+distanceTol > tmp->point[depth%3])
			searchHelper(tmp->right,depth+1,distanceTol,ids,target);
		
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root,0,distanceTol,ids,target);
		return ids;
	}
	

};




