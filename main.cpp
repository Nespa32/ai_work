
#include <stdio.h>
#include <cstring>
#include <cmath>

// data structures
#include <list>
#include <vector>
#include <unordered_set> // hash map

// time measurement
#include <chrono>
#include <ctime>

// debugging
#include <cassert>

int __debug__ = 1; /* toggle for debug prints*/

#define DEBUG_LOG(...)                  \
    do {                                \
        if (__debug__  != 0) {          \
            printf(__VA_ARGS__);        \
            printf("\n");               \
        }                               \
    } while (0)


/// global vars
int init[9] = {
// 24 steps
//	3, 4, 2,
//	5, 1, 7,
//	6, 0, 8,

// test, 3 steps
//	1, 2, 3,
//	8, 4, 5,
//	7, 0, 6,

// test, 6 steps
    0, 2, 3,
    1, 8, 5,
    7, 4, 6,
};

int goal[9] = {
	1, 2, 3,
	8, 0, 4,
	7, 6, 5,
};

// just for measuring
int node_counter = 0;

struct Node;

std::list<Node*> nodeQueue;
Node* root; // it's ze tree

enum SearchType
{
    SEARCH_TYPE_BFS,
    SEARCH_TYPE_DFS,
    SEARCH_TYPE_IDFS,
    SEARCH_TYPE_GREEDY,
    SEARCH_TYPE_A_STAR,
};

SearchType searchType;

struct Node
{
	Node()
	{
        node_counter_n = node_counter;
        
        ++node_counter;
        if (node_counter % 1000 == 0)
            printf("Node::Node - allocated %d nodes\n", node_counter);
        
		memset(state, 0, sizeof(state));
		parent = nullptr;
		move = 0;
		depth = 0;
		cost = 0;
	}

	int state[9];
	Node* parent;
	int move; // applied move, the number that was switched with '0'
	int depth; // calculated from parent
	int cost; // effective cost since root, calculated from parent
    
    int node_counter_n;

	bool IsGoal()
	{
		return memcmp(state, goal, sizeof(goal)) == 0;
	}

	// (i, j) should contain 0, we just want to switch the values of the 2 coordinate sets
	void DoMove(int i, int j, int ni, int nj)
	{
		move = state[ni + nj * 3];
		state[i + j * 3] = move;
		state[ni + nj * 3] = 0;
        
        cost = depth + HeuristicMahatthanDist(this);
        
        if (node_counter % 1000 == 0)
        {
            for (int i = 0; i < 9; ++i)
                printf("%d ", state[i]);
            
            printf("\n");
            printf("Cost is %d, depth is %d\n", cost, depth);
            printf("Cost of the front of the queue is %d, depth is %d\n",
                nodeQueue.front()->cost, nodeQueue.front()->depth);
        }
	}
    
    
    static int HeuristicOffset(Node const* node)
    {
        int result = 0;
        for (int i = 0; i < 9; ++i)
        {
            if (node->state[i] == 0) // skip white spot
                continue;
            
            if (node->state[i] != goal[i])
                ++result;
        }
        
        return result;
    }

    static int HMD_helper(int state_i, int state_j, int val)
    {
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                if (val == goal[i + j * 3])
                    return std::abs(state_i - i) + std::abs(state_j - j);
            }
        }
        
        printf("HMD_helper - couldn't find val %d, state_i is %d, state_j is %d\n", val, state_i, state_j);
        assert(false);
        return 0; // should never happen
    }

    static int HeuristicMahatthanDist(Node const* node)
    {
        int result = 0;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                int stateIdx = i + j * 3;
                if (node->state[stateIdx] == 0)
                    continue;
                
                result += HMD_helper(i, j, node->state[stateIdx]);
            }
        }
        
        return result;
    }
};

void FillFirstNode()
{
	Node* node = new Node();
	// init states
	memcpy(node->state, init, sizeof(init));

	root = node;

	nodeQueue.push_back(node);
}

#define IS_VALID(x, y) x >= 0 && x < 3 && y >= 0 && y < 3

std::list<Node*> MakeDescendants(Node* parent)
{
	std::list<Node*> list;

	// ain't this some ugly shit
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (parent->state[i + j * 3] == 0)
			{
				if (IS_VALID(i + 1, j))
				{
                    Node* node = new Node();
                    node->parent = parent;
                    node->depth = parent->depth + 1;
                    memcpy(node->state, parent->state, sizeof(init));
					node->DoMove(i, j, i + 1, j);
					list.push_back(node);
				}
                
				if (IS_VALID(i, j + 1))
				{
                    Node* node = new Node();
                    node->parent = parent;
                    node->depth = parent->depth + 1;
                    memcpy(node->state, parent->state, sizeof(init));
                    node->DoMove(i, j, i, j + 1);
					list.push_back(node);
				}
                
				if (IS_VALID(i - 1, j))
				{
                    Node* node = new Node();
                    node->parent = parent;
                    node->depth = parent->depth + 1;
                    memcpy(node->state, parent->state, sizeof(init));
                    node->DoMove(i, j, i - 1, j);
					list.push_back(node);
				}
                
				if (IS_VALID(i, j - 1))
				{
                    Node* node = new Node();
                    node->parent = parent;
                    node->depth = parent->depth + 1;
                    memcpy(node->state, parent->state, sizeof(init));
                    node->DoMove(i, j, i, j - 1);
					list.push_back(node);
				} 

			}
		}
	}

	return list;
}

void AddToQueueBFS(std::list<Node*> descList)
{
	for (Node* node : descList)
		nodeQueue.push_back(node);
}

void AddToQueueDFS(std::list<Node*> descList)
{
	for (Node* node : descList)
		nodeQueue.push_front(node);
}

// IDFS = Iterative DFS
void AddToQueueIDFS(std::list<Node*> descList)
{
    // this only works because the program will only do one search before exiting
    static int IDFS_depth = 0;
    static std::vector<Node*> nodesToDelete;
        
	for (Node* node : descList)
    {
        if (node->depth <= IDFS_depth)
            nodeQueue.push_front(node);
        
        nodesToDelete.push_back(node);
    }
    
    // IDFS cycle
    // increase depth, clear all previous data
    if (nodeQueue.empty())
    {
        nodesToDelete.push_back(root);
        
        for (Node* node : nodesToDelete)
            delete node;
        
        nodesToDelete.clear();
        
        node_counter = 0; // reset counter, previous nodes were freed
        
        ++IDFS_depth;
        FillFirstNode(); // mister bones' wild ride never ends
        
        assert(nodeQueue.size() == 1);
    }
}

bool CompareNodes(Node* left, Node* right)
{
    return Node::HeuristicMahatthanDist(left) <= Node::HeuristicMahatthanDist(right);
}

// @todo: instead of a list, a std::map<int, Node*> would be more appropriate and would automatically sort
void AddToQueueGreedy(std::list<Node*> descList)
{
    // blind adding
	for (Node* node : descList)
		nodeQueue.push_front(node);
    
    // re-sort the whole list
    nodeQueue.sort(CompareNodes);
}

bool CompareNodesAStar(Node* left, Node* right)
{
    if (left->cost != right->cost)
        return left->cost < right->cost;
    
    assert(left->node_counter_n != right->node_counter_n);
    return left->node_counter_n < right->node_counter_n;
}

struct NodeState
{
    NodeState(int s[9])
    {
        memcpy(state, s, sizeof(state));
    }
    
    int state[9];
    
    bool operator== (NodeState const& node) const { return memcmp(state, node.state, sizeof(state)) == 0; }
   
    bool operator!= (NodeState const& node) const { return memcmp(state, node.state, sizeof(state)) != 0; }
    bool operator< (NodeState const& node) const { return false; }

};

// needed to be able to use std::unordered_set<NodeState> (a hash map)
namespace std
{
    template <>
    struct hash<NodeState>
    {
        std::size_t operator()(const NodeState& nodeState) const
        {
            using std::size_t;
            using std::hash;
            using std::string;

            hash<int> hasher;
      
            std::size_t h = 0;
            for (int i = 0; i < 9; ++i)
                h = h * 31 + hasher(nodeState.state[i]);
      
            return h;
        }
    };
}

void AddToQueueAStar(std::list<Node*> descList)
{
    static std::unordered_set<NodeState> visited;
        // blind adding
	for (Node* node : descList)
    {
        NodeState state = node->state;
        if (visited.find(state) == visited.end())
        {
            visited.insert(state);
            nodeQueue.push_front(node);
        }
    }
    
    // re-sort the whole list
    nodeQueue.sort(CompareNodesAStar);
}

void Finish(Node* node)
{
	std::list<int> moveList;
	while (node->parent)
	{
		moveList.push_front(node->move);
		node = node->parent;
	}

	for (int i : moveList)
		printf("%d : ", i);

	printf("\n");

	printf("Solution size: %zu\n", moveList.size());
}

void search(int type)
{
    FillFirstNode();

	assert(!nodeQueue.empty());

	while (nodeQueue.empty() == false)
	{
		Node* node = nodeQueue.front();
		nodeQueue.pop_front();
		
		if (node->IsGoal())
		{
			Finish(node);
			return;
		}

		std::list<Node*> descList = MakeDescendants(node);
        // options:
        // AddToQueueBFS
        // AddToQueueDFS
        // AddToQueueIDFS
        // AddToQueueGreedy
        // AddToQueueAStar
        
		AddToQueueIDFS(descList);
	}

	printf("No bueno\n");
}

/*
    help
    
    --start_config [list of 9 numbers]
    i.e: --start_config 1 2 3 8 0 4 7 6 5
    equals to
    {
        1, 2, 3,
        8, 0, 4,
        7, 6, 5
    }
    
    --end_config [list of 9 numbers]

    --search_type=<type>
    types:
    - bfs
    - dfs
    - idfs
    - greedy
    - astar
    
    i.e: --search_type=greedy
*/

// arg parsing helper
bool stringEndsWith(std::string s, std::string end) {
    if (s.length() < end.length())
        return false;
    
    return s.compare(s.length() - end.length(), end.length(), end) == 0;
}

int main(int argc, char** argv)
{
    for (int i = 0; i < argc; ++i)
    {
        if (strcmp(argv[i], "--start_config") == 0)
        {
            // eat up the next 9 digits
        }
        else if (strcmp(argv[i], "--end_config") == 0)
        {
            // eat up the next 9 digits
        }
        else if (strcmp(argv[i], "--search_type") == 0)
        {
            if (stringEndsWith(argv[i], "bfs"))
                ;
            else if (stringEndsWith(argv[i], "bfs"))
                ;
            else if (stringEndsWith(argv[i], "dfs"))
                ;
            else if (stringEndsWith(argv[i], "idfs"))
                ;
            else if (stringEndsWith(argv[i], "greedy"))
                ;
            else if (stringEndsWith(argv[i], "astar"))
                ;
            else
            {
                printf("Bad search type (input string: %s)", argv[i]);
                return 1;
            }
            
        }
    }

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    
    search(0);
    
    end = std::chrono::system_clock::now();
 
    std::chrono::duration<float> duration = end - start;
 
    printf("Nodes allocated: %d\n", node_counter);
    printf("Search duration: %f second(s)\n", duration.count());
    
	return 0;
}
