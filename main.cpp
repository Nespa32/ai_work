
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

// project files
#include "node.h"

int __debug__ = 1; /* toggle for debug prints*/

/// global vars
int init[MATRIX_SIZE] = {
// 24 steps
	3, 4, 2,
	5, 1, 7,
	6, 0, 8,

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

// just for measuring
int node_counter = 0;

std::list<Node*> nodeQueue;
Node* root = nullptr;

int main(int argc, char** argv)
{
    SearchType searchType = SEARCH_TYPE_BFS;
    
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
            if (StringEndsWith(argv[i], "bfs"))
                ;
            else if (StringEndsWith(argv[i], "bfs"))
                ;
            else if (StringEndsWith(argv[i], "dfs"))
                ;
            else if (StringEndsWith(argv[i], "idfs"))
                ;
            else if (StringEndsWith(argv[i], "greedy"))
                ;
            else if (StringEndsWith(argv[i], "astar"))
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
    
    GeneralSearchAlgorithm(searchType);
    
    end = std::chrono::system_clock::now();
 
    std::chrono::duration<float> duration = end - start;
 
    printf("Nodes allocated: %d\n", node_counter);
    printf("Search duration: %f second(s)\n", duration.count());
    
	return 0;
}

void FillFirstNode()
{
	Node* node = new Node();
	// init states
    node->_state = init;
    
	root = node;

	nodeQueue.push_back(node);
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
        if (node->_depth <= IDFS_depth)
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

void AddToQueueGreedy(std::list<Node*> descList)
{
	for (Node* node : descList)
    {
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

// needed to be able to use std::unordered_set<NodeState> (a hash map)
namespace std
{
    template <>
    struct hash<NodeState>
    {
        std::size_t operator()(const NodeState& nodeState) const
        {
            std::hash<int> hasher;
      
            std::size_t h = 0;
            for (int i = 0; i < MATRIX_SIZE; ++i)
                h = h * 31 + hasher(nodeState._state[i]);
      
            return h;
        }
    };
}

void AddToQueueAStar(std::list<Node*> descList)
{
    static std::unordered_set<NodeState> visited;
    
	for (Node* node : descList)
    {
        NodeState& state = node->_state;
        if (visited.find(state) != visited.end())
            continue;
        
        visited.insert(state);
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
            // ...
        default:
            break;
        }
        
        // options:
        // AddToQueueBFS
        // AddToQueueDFS
        // AddToQueueIDFS
        // AddToQueueGreedy
        // AddToQueueAStar
        
		AddToQueueAStar(descList);
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
bool StringEndsWith(std::string s, std::string end) {
    if (s.length() < end.length())
        return false;
    
    return s.compare(s.length() - end.length(), end.length(), end) == 0;
}

Node::Node()
{
    ++node_counter;
    if (node_counter % 1000 == 0)
        DEBUG_LOG("Node::Node - allocated %d nodes", node_counter);

    _parent = nullptr;
    _move = 0;
    _depth = 0;
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



