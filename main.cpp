
#include <stdio.h>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>

// data structures
#include <list>
#include <vector>
#include <unordered_set> // hash map

// time measurement
#include <chrono>
#include <ctime>

// debugging
#include <cassert>

// project files
#include "node.h"

int __debug__ = 0; /* toggle for debug prints*/

/// global vars
int initial_config[MATRIX_SIZE] = {
// 24 steps
	3, 4, 2,
	5, 1, 7,
	6, 0, 8,

// invalid
//    6, 2, 7,
//    5, 0, 3,
//    8, 1, 4,

// test, 3 steps
//	1, 2, 3,
//	8, 4, 5,
//	7, 0, 6,

// test, 6 steps
//    0, 2, 3,
//    1, 8, 5,
//    7, 4, 6,
};

int final_config[MATRIX_SIZE] = {
	1, 2, 3,
	8, 0, 4,
	7, 6, 5,
};

// total node measuring
int node_counter_total = 0;
int node_counter_exiting = 0;

// visited node states
std::unordered_set<NodeState> visitedStates;
std::list<Node*> nodeQueue; // nodes that have not been expanded yet

SearchType searchType = SEARCH_TYPE_BFS;

int main(int argc, char** argv)
{
    // handle program options
    HandleArgs(argc, argv);
    
    // check if final_config can be reached from initial_config
    if (!IsValidConfig())
        return 1;

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    
    GeneralSearchAlgorithm();
    
    end = std::chrono::system_clock::now();
 
    std::chrono::duration<float> duration = end - start;
 
    printf("Total nodes allocated throughout search: %d\n", node_counter_total);
    printf("Total nodes allocated right now: %d\n", node_counter_exiting);
    printf("Search duration: %f second(s)\n", duration.count());
    
	return 0;
}

void FillFirstNode()
{
    NodeState nodeState = initial_config;
	Node* node = new Node(nodeState);
    nodeQueue.push_back(node);
}

void AddToQueueBFS(Node* parent, std::list<NodeState> descList)
{
	for (NodeState nodeState : descList)
    {
        if (visitedStates.find(nodeState) != visitedStates.end())
            continue;
        
        visitedStates.insert(nodeState);
        
        Node* node = new Node(parent, nodeState);

		nodeQueue.push_back(node);
    }
}

void AddToQueueDFS(Node* parent, std::list<NodeState> descList)
{
	for (NodeState nodeState : descList)
    {
        if (visitedStates.find(nodeState) != visitedStates.end())
            continue;
        
        visitedStates.insert(nodeState);
        
        Node* node = new Node(parent, nodeState);
        
        // add to parent list
        parent->_childNodes.push_back(node);
        
		nodeQueue.push_front(node);
    }
    
    // no child nodes with parent references, free up memory
    while (parent->_childNodes.empty())
    {
        // it's turtles all the way up
        Node* child = parent;
        parent = child->_parent;
        delete child;
        
        if (!parent) // we've reached the root and deleted everything
            break;
        
        parent->_childNodes.remove(child);
    }
}

// IDFS = Iterative DFS
void AddToQueueIDFS(Node* parent, std::list<NodeState> descList)
{
    // this only works because the program will only do one search before exiting
    static int IDFS_depth = 0;

    if (parent->_depth < IDFS_depth)
    {
        for (NodeState nodeState : descList)
        {
            Node* node = new Node(parent, nodeState);
            
            // add to parent list
            parent->_childNodes.push_back(node);
            
            nodeQueue.push_front(node);
        }
    }
    
    // no child nodes with parent references, free up memory
    while (parent->_childNodes.empty())
    {
        // it's turtles all the way up
        Node* child = parent;
        parent = child->_parent;
        delete child;
        
        if (!parent) // we've reached the root and deleted everything
            break;
        
        parent->_childNodes.remove(child);
    }
    
    // IDFS cycle
    // increase depth, clear all previous data
    if (nodeQueue.empty())
    {
        DEBUG_LOG("IDFS: Moving to new depth: %d", IDFS_depth);
        
        ++IDFS_depth;
        FillFirstNode(); // mister bones' wild ride never ends
        
        assert(nodeQueue.size() == 1);
    }
}

void AddToQueueGreedy(Node* parent, std::list<NodeState> descList)
{
	for (NodeState nodeState : descList)
    {
        if (visitedStates.find(nodeState) != visitedStates.end())
            continue;
        
        visitedStates.insert(nodeState);
        
        Node* node = new Node(parent, nodeState);
        
        int cost = node->_cost;

        // manually *sort* the list
        // search through the list up to the point where nodes have a higher cost, and insert our node right before
        // an std::map<int, Node*> (a balanced tree) would automatically sort and would have a log(n) insert op, but we need to use the queue for other searches
        auto itr = nodeQueue.begin();
        for (; itr != nodeQueue.end(); ++itr)
        {
            Node* n = *itr;
            if (cost <= n->_cost)
                break;
        }

        nodeQueue.insert(itr, node);
    }
}

void AddToQueueAStar(Node* parent, std::list<NodeState> descList)
{
	for (NodeState nodeState : descList)
    {
        Node* node = new Node(parent, nodeState);
        
        int cost = node->_cost;

        // manually *sort* the list
        // search through the list up to the point where nodes have a higher cost, and insert our node right before
        // an std::map<int, Node*> (a balanced tree) would automatically sort and would have a log(n) insert op, but we need to use the queue for other searches
        auto itr = nodeQueue.begin();
        for (; itr != nodeQueue.end(); ++itr)
        {
            Node* n = *itr;
            if (cost <= n->_cost)
                break;
        }

        nodeQueue.insert(itr, node);
    }
}

void Finish(Node* node)
{
	std::list<int> moveList;
	while (node->_parent)
	{
		moveList.push_front(node->_state._move);
		node = node->_parent;
	}

	for (int i : moveList)
		printf("%d -> ", i);

	printf("Done\n");

	printf("Solution size: %zu\n", moveList.size());
}

void GeneralSearchAlgorithm()
{
    switch (searchType)
    {
        case SEARCH_TYPE_BFS:
            printf("Using BFS search...\n");
            break;
        case SEARCH_TYPE_DFS:
            printf("Using DFS search...\n");
            break;
        case SEARCH_TYPE_IDFS:
            printf("Using IDFS search...\n");
            break;
        case SEARCH_TYPE_GREEDY:
            printf("Using Greedy search...\n");
            break;
        case SEARCH_TYPE_A_STAR:
            printf("Using A* search...\n");
            break;
        default:
            assert(false);
            break;
    }

    FillFirstNode();

    assert(!nodeQueue.empty());

	while (nodeQueue.empty() == false)
	{
        // remove front node
		Node* node = nodeQueue.front();
        nodeQueue.pop_front();
		
        // test if goal
		if (node->IsGoal())
		{
			Finish(node);
			return;
		}

        // get child nodes
		std::list<NodeState> descList = node->MakeDescendants();
        
        // apply search type
        switch (searchType)
        {
        case SEARCH_TYPE_BFS:
            AddToQueueBFS(node, descList);
            break;
        case SEARCH_TYPE_DFS:
            AddToQueueDFS(node, descList);
            break;
        case SEARCH_TYPE_IDFS:
            AddToQueueIDFS(node, descList);
            break;
        case SEARCH_TYPE_GREEDY:
            AddToQueueGreedy(node, descList);
            break;
        case SEARCH_TYPE_A_STAR:
            AddToQueueAStar(node, descList);
            break;
        default:
            assert(false);
            break;
        }
	}

	printf("Goal node not found\n");
}

/*
    help
    
    --initial_config [list of 9 numbers]
    i.e: --initial_config 1 2 3 8 0 4 7 6 5
    equals to
    {
        1, 2, 3,
        8, 0, 4,
        7, 6, 5
    }
    
    --final_config [list of 9 numbers]

    --search_type=<type>
    types:
    - bfs
    - dfs
    - idfs
    - greedy
    - astar
    
    i.e: --search_type=greedy
*/

bool IsValidConfig()
{
    int sorted_config[MATRIX_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    
    for (int i = 0; i < MATRIX_SIZE; ++i)
    {
		if (final_config[i] != 0)
            sorted_config[final_config[i] - 1] = initial_config[i];
		else
            sorted_config[8] = initial_config[i];
    }

	int inversions = 0;
	for (int i = 0; i < MATRIX_SIZE; ++i)
    {
		for (int j = i + 1; j < MATRIX_SIZE; ++j)
        {
			if (sorted_config[i] > sorted_config[j] && sorted_config[i] && sorted_config[j])
				++inversions;
        }
    }

	if (inversions % 2 != 0) // odd number of inversions
    {
        printf("Final state not reachable (%d inversions)\n", inversions);
        return false;
    }
    
    return true;
}

// handles options, sets the search type
void HandleArgs(int argc, char** argv)
{
    searchType = SEARCH_TYPE_BFS;
    
    bool isInitialConfigSet = false;
    bool isFinalConfigSet = false;
    bool isSearchTypeSet = false;
    
    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--initial_config") == 0)
        {
            isInitialConfigSet = true;
            
            // eat up the next 9 digits
            for (int j = 0; j < MATRIX_SIZE; ++j)
            {
                if (++i >= argc)
                {
                    printf("--initial_config reading error - missing digits?\n");
                    exit(1);
                }
                
                initial_config[j] = atoi(argv[i]);
            }
        }
        else if (strcmp(argv[i], "--final_config") == 0)
        {
            isFinalConfigSet = true;
            
            // eat up the next 9 digits
            for (int j = 0; j < MATRIX_SIZE; ++j)
            {
                if (++i >= argc)
                {
                    printf("--final_config reading error - missing digits?\n");
                    exit(1);
                }
                
                final_config[j] = atoi(argv[i]);
            }
        }
        else if (strncmp(argv[i], "--search_type", strlen("--search_type")) == 0)
        {
            isSearchTypeSet = true;
            
            if (StringEndsWith(argv[i], "bfs"))
                searchType = SEARCH_TYPE_BFS;
            else if (StringEndsWith(argv[i], "idfs")) // need to check before "dfs" since this string also ends with "dfs"
                searchType = SEARCH_TYPE_IDFS;
            else if (StringEndsWith(argv[i], "dfs"))
                searchType = SEARCH_TYPE_DFS;
            else if (StringEndsWith(argv[i], "greedy"))
                searchType = SEARCH_TYPE_GREEDY;
            else if (StringEndsWith(argv[i], "astar"))
                searchType = SEARCH_TYPE_A_STAR;
            else
            {
                printf("Bad search type (input string: %s)", argv[i]);
                exit(1);
            }
        }
        else
        {
            printf("Unrecognised arg - <%s>\n", argv[i]);
            exit(1);
        }
    }
    
    if (!isInitialConfigSet)
    {
        printf("Initial config not set - using [ ");
        
        for (int i = 0; i < MATRIX_SIZE; ++i)
            printf("%d ", initial_config[i]);
        
        printf("] by default\n");
    }

    if (!isFinalConfigSet)
    {
        printf("Final config not set - using [ ");
        
        for (int i = 0; i < MATRIX_SIZE; ++i)
            printf("%d ", final_config[i]);
        
        printf("] by default\n");
    }
    
    if (!isSearchTypeSet)
        printf("Search type not set - using bfs by default\n");
}
    
// arg parsing helper
bool StringEndsWith(std::string s, std::string end) {
    if (s.length() < end.length())
        return false;
    
    return s.compare(s.length() - end.length(), end.length(), end) == 0;
}

Node::Node(NodeState& nodeState)
{
    ++node_counter_exiting;
    ++node_counter_total;
    if (node_counter_total % 1000 == 0)
        DEBUG_LOG("Node::Node - allocated %d nodes", node_counter_total);

    _parent = nullptr;
    _depth = 0;
    _state = nodeState;
    _cost = GetNodeCost();
}

Node::Node(Node* parent, NodeState& nodeState)
{
    ++node_counter_exiting;
    ++node_counter_total;
    if (node_counter_total % 1000 == 0)
        DEBUG_LOG("Node::Node - allocated %d nodes", node_counter_total);
    
    _parent = parent;
    _depth = parent->_depth + 1;
    _state = nodeState;
    _cost = GetNodeCost();
}

Node::~Node()
{
    --node_counter_exiting;
    assert(_childNodes.empty());
}

bool Node::IsGoal()
{
    return _state == final_config;
}

std::list<NodeState> Node::MakeDescendants() const
{
    std::list<NodeState> descList;

    static const std::vector<std::vector<int>> offsets = { {1, 0}, {0, 1}, {-1, 0}, {0, -1} };
    
    for (int idx = 0; idx < MATRIX_SIZE; ++idx)
    {
        if (_state._state[idx] != 0)
            continue;
        
        int i, j;
        IdxToCoords(i, j, idx);
        
        for (std::vector<int> offset : offsets)
        {
#define IS_VALID_COORDS(x, y) (x >= 0 && x < MATRIX_SIDE_SIZE && y >= 0 && y < MATRIX_SIDE_SIZE)

            if (!IS_VALID_COORDS(i + offset[0], j + offset[1]))
                continue;
            
            int newIdx = CoordsToIdx(i + offset[0], j + offset[1]);

            NodeState nodeState = _state;
            
            nodeState._move = nodeState._state[idx] = nodeState._state[newIdx];
            nodeState._state[newIdx] = 0;
            
            descList.push_back(nodeState);
        }
    }

    return descList;
}

int Node::GetManhattanDist() const
{
    int result = 0;
    for (int idx = 0; idx < MATRIX_SIZE; ++idx)
    {
        if (_state._state[idx] == 0)
            continue;

        // find the equivalent in the goal matrix
        for (int k = 0; k < MATRIX_SIZE; ++k)
        {
            if (_state._state[idx] == final_config[k])
            {
                // get our x/y coordinates
                int i, j;
                IdxToCoords(i, j, idx);
                
                // get the goal's x/y coordinates
                int x, y;
                IdxToCoords(x, y, k);
            
                result += std::abs(i - x) + std::abs(j - y);
                break;
            }
        }
    }
    
    return result;
}

int Node::CoordsToIdx(int i, int j) const
{
    return i + j * MATRIX_SIDE_SIZE;
}

void Node::IdxToCoords(int& i, int& j, int idx) const
{
    i = idx % MATRIX_SIDE_SIZE;
    j = idx / MATRIX_SIDE_SIZE;
}

int Node::GetNodeCost() const
{
    switch (searchType)
    {
    case SEARCH_TYPE_GREEDY:
        return GetManhattanDist();
    case SEARCH_TYPE_A_STAR:
        return _depth + GetManhattanDist();
    default:
        return _depth;
    }
}