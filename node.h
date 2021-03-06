

#ifndef _NODE_H_
#define _NODE_H_

extern int __debug__; /* toggle for debug prints*/

#define DEBUG_LOG(...)                  \
    do {                                \
        if (__debug__  != 0) {          \
            printf(__VA_ARGS__);        \
            printf("\n");               \
        }                               \
    } while (0)

enum SearchType
{
    SEARCH_TYPE_BFS,
    SEARCH_TYPE_DFS,
    SEARCH_TYPE_IDFS,
    SEARCH_TYPE_GREEDY,
    SEARCH_TYPE_A_STAR,
};

enum
{
    // we could *mostly* make it work with a 15 puzzle by editing this
    MATRIX_SIDE_SIZE    = 3,
    MATRIX_SIZE         = MATRIX_SIDE_SIZE * MATRIX_SIDE_SIZE,
};

#define MATRIX_STRUCT_SIZE (sizeof(int) * MATRIX_SIZE)

struct NodeState
{
    NodeState() { memset(_state, 0, MATRIX_STRUCT_SIZE); }
    NodeState(int state[MATRIX_SIZE])
    {
        memcpy(_state, state, MATRIX_STRUCT_SIZE);
    }
    
    int _state[MATRIX_SIZE];
    int _move; // applied move, the number that was switched with '0'
    
    bool operator== (NodeState const& node) const { return memcmp(_state, node._state, MATRIX_STRUCT_SIZE) == 0; }
    bool operator== (int array[MATRIX_SIZE]) const { return memcmp(_state, array, MATRIX_STRUCT_SIZE) == 0; }
   
    bool operator!= (NodeState const& node) const { return memcmp(_state, node._state, MATRIX_STRUCT_SIZE) != 0; }
    bool operator< (NodeState const& node) const { return false; }
};

struct Node
{
public:
	Node(NodeState& state);
    Node(Node* parent, NodeState& state);
    ~Node();

	NodeState _state;
	Node* _parent;
    
	int _depth; // calculated from parent
    int _cost;
    
    int _nChildNodes; // number of child nodes, used for DFS/IDFS
    
public:
	bool IsGoal();

    std::list<NodeState> MakeDescendants() const;
    
    int CoordsToIdx(int i, int j) const;
    void IdxToCoords(int& i, int& j, int idx) const;
    
private:
    // used heuristic
    int GetManhattanDist() const;
    // depends on search type, used on node initialization
    // needs to be called *after* _depth is initialized
    int GetNodeCost() const;
    
public:
    bool operator< (Node const& node) const { return _cost < node._cost; }
};

void FillFirstNode();
void AddToQueueBFS(std::list<Node*> descList);
void AddToQueueDFS(std::list<Node*> descList);
void AddToQueueIDFS(std::list<Node*> descList);
void AddToQueueGreedy(std::list<Node*> descList);
void AddToQueueAStar(std::list<Node*> descList);

void Finish(Node* node);

void GeneralSearchAlgorithm();

// checks if final_config can be reached from initial_config
bool IsValidConfig();

// handles options, sets the search type
void HandleArgs(int argc, char** argv);

// arg parsing helper
bool StringEndsWith(std::string s, std::string end);

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

#endif // _NODE_H_
