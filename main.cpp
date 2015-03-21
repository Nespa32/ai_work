
#include <stdio.h>
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
int init[MATRIX_SIZE] = {
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

int goal[MATRIX_SIZE] = {
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

int main(int argc, char** argv)
{
    SearchType searchType = SEARCH_TYPE_BFS;
    
    for (int i = 1; i < argc; ++i)
    {
        if (strcmp(argv[i], "--start_config") == 0)
        {
            // eat up the next 9 digits
            for (int j = 0; j < MATRIX_SIZE; ++j)
            {
                if (++i >= argc)
                {
                    printf("--start_config reading error - missing digits?\n");
                    return 1;
                }
                
                init[j] = atoi(argv[i]);
            }
        }
        else if (strcmp(argv[i], "--end_config") == 0)
        {
            // eat up the next 9 digits
            for (int j = 0; j < MATRIX_SIZE; ++j)
            {
                if (++i >= argc)
                {
                    printf("--end_config reading error - missing digits?\n");
                    return 1;
                }
                
                goal[j] = atoi(argv[i]);
            }
        }
        else if (strncmp(argv[i], "--search_type", strlen("--search_type")) == 0)
        {
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
                return 1;
            }
        }
        else
        {
            printf("Unrecognised arg - <%s>\n", argv[i]);
            return 1;
        }
    }
    
    ///
    int sortedConfig[MATRIX_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    
    for (int i = 0; i < MATRIX_SIZE; ++i)
    {
		if (goal[i] != 0)
            sortedConfig[goal[i] - 1] = init[i];
		else
            sortedConfig[8] = init[i];
    }

	int inversions = 0;
	for (int i = 0; i < MATRIX_SIZE; ++i)
    {
		for (int j = i + 1; j < MATRIX_SIZE; ++j)
        {
			if (sortedConfig[i] > sortedConfig[j] && sortedConfig[i] && sortedConfig[j])
				++inversions;
        }
    }

	if (inversions % 2 != 0) // odd number of inversions
    {
        printf("Final state not reachable (%d inversions)\n", inversions);
        return 1;
    }
    ///

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    
    GeneralSearchAlgorithm(searchType);
    
    end = std::chrono::system_clock::now();
 
    std::chrono::duration<float> duration = end - start;
 
    printf("Total nodes allocated throughout search: %d\n", node_counter_total);
    printf("Total nodes allocated right now: %d\n", node_counter_exiting);
    printf("Search duration: %f second(s)\n", duration.count());
    
	return 0;
}

void FillFirstNode()
{
	Node* node = new Node();
	// init states
    node->_state = init;

	nodeQueue.push_back(node);
}

void AddToQueueBFS(Node* /*parent*/, std::list<Node*> descList)
{
	for (Node* node : descList)
    {
        NodeState& state = node->_state;
        if (visitedStates.find(state) != visitedStates.end())
            continue;
        
        visitedStates.insert(state);
        
		nodeQueue.push_back(node);
    }
}

void AddToQueueDFS(Node* parent, std::list<Node*> descList)
{
	for (Node* node : descList)
    {
        NodeState& state = node->_state;
        if (visitedStates.find(state) != visitedStates.end())
        {
            delete node;
            continue;
        }
        
        visitedStates.insert(state);
        
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
void AddToQueueIDFS(Node* parent, std::list<Node*> descList)
{
    // this only works because the program will only do one search before exiting
    static int IDFS_depth = 0;
        
	for (Node* node : descList)
    {
        if (node->_depth > IDFS_depth)
        {
            delete node; // avoid memory leaks
            continue;
        }
        
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

void AddToQueueGreedy(Node* parent, std::list<Node*> descList)
{
	for (Node* node : descList)
    {
        NodeState& state = node->_state;
        if (visitedStates.find(state) != visitedStates.end())
            continue;
        
        visitedStates.insert(state);
        
        int cost = node->GetManhattanDist();
        
        // manually *sort* the list
        // search through the list up to the point where nodes have a higher cost, and insert our node right before
        // an std::map<int, Node*> (a balanced tree) would automatically sort and would have a log(n) insert op, but we need to use the queue for other searches
        auto itr = nodeQueue.begin();
        for (; itr != nodeQueue.end(); ++itr)
        {
            if (cost <= (*itr)->GetManhattanDist())
                break;
        }

		nodeQueue.insert(itr, node);
    }
}

void AddToQueueAStar(Node* parent, std::list<Node*> descList)
{
	for (Node* node : descList)
    {
        NodeState& state = node->_state;
        if (visitedStates.find(state) != visitedStates.end())
            continue;
        
        visitedStates.insert(state);
        
        int cost = node->_depth + node->GetManhattanDist();
        
        // manually *sort* the list
        // search through the list up to the point where nodes have a higher cost, and insert our node right before
        // an std::map<int, Node*> (a balanced tree) would automatically sort and would have a log(n) insert op, but we need to use the queue for other searches
        auto itr = nodeQueue.begin();
        for (; itr != nodeQueue.end(); ++itr)
        {
            Node* n = *itr;
            if (cost <= n->_depth + n->GetManhattanDist())
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
		moveList.push_front(node->_move);
		node = node->_parent;
	}

	for (int i : moveList)
		printf("%d : ", i);

	printf("\n");

	printf("Solution size: %zu\n", moveList.size());
}

void GeneralSearchAlgorithm(SearchType searchType)
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
		std::list<Node*> descList = node->MakeDescendants();
        
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
bool StringEndsWith(std::string s, std::string end) {
    if (s.length() < end.length())
        return false;
    
    return s.compare(s.length() - end.length(), end.length(), end) == 0;
}

Node::Node()
{
    ++node_counter_exiting;
    ++node_counter_total;
    if (node_counter_total % 1000 == 0)
        DEBUG_LOG("Node::Node - allocated %d nodes", node_counter_total);

    _parent = nullptr;
    _move = 0;
    _depth = 0;
}

Node::~Node()
{
    --node_counter_exiting;
    assert(_childNodes.empty());
}

bool Node::IsGoal()
{
    return _state == goal;
}

std::list<Node*> Node::MakeDescendants()
{
    std::list<Node*> list;

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
            
            Node* node = new Node();
            node->_parent = this;
            node->_depth = _depth + 1;
            node->_state = _state;
            
            node->DoMove(idx, newIdx); // must be after state initialization
            
            list.push_back(node);
        }
    }

    return list;
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
            if (_state._state[idx] == goal[k])
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

void Node::DoMove(int a, int b)
{
    assert(_state._state[a] == 0 && a != b);
    
    _move = _state._state[a] = _state._state[b];
    _state._state[b] = 0;
}



