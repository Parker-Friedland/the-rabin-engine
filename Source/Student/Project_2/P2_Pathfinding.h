#pragma once
#include "Misc/PathfindingDetails.hpp"

struct Node
{
    Node(GridPos pos, Heuristic h, GridPos goal) :
        _pos(pos), _parent(nullptr), _g(0), _f(AStarPather::Distance(h, pos, goal)) {}

    Node(GridPos pos, Node* parent, float g, float f) :
        _pos(pos), _parent(parent), _g(g), _f(f) {}

    Node(GridPos pos, Node* parent, Heuristic h, GridPos goal, bool diag) :
        _pos(pos), _parent(parent), _g(parent->_g + diag ? sqrtf(2) : 1), _f(_g + AStarPather::Distance(h, pos, goal)) {}

    bool IsOpen(MAP m)
    {
        _f == m.find(_pos)->second._f;
    }

    GridPos _pos;
    const Node* _parent;
    float _g; // given cost
    float _f; // given cost + heuristic cost

    bool operator<(const Node& rhs) const
    {
        return _f < rhs._f;
    }

    bool operator==(const Node& rhs) const
    {
        return _pos == rhs._pos;
    }

    bool operator!=(const Node& rhs) const
    {
        return _pos != rhs._pos;
    }
};

typedef std::priority_queue<Node, std::vector<Node>, std::less<Node>> QUEUE;
typedef std::unordered_map<GridPos, Node> MAP;
typedef MAP::iterator IT;

class AStarPather
{
public:
    /* 
        The class should be default constructible, so you may need to define a constructor.
        If needed, you can modify the framework where the class is constructed in the
        initialize functions of ProjectTwo and ProjectThree.
    */

    /* ************************************************** */
    // DO NOT MODIFY THESE SIGNATURES
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);
    /* ************************************************** */

    /*
        You should create whatever functions, variables, or classes you need.
        It doesn't all need to be in this header and cpp, structure it whatever way
        makes sense to you.
    */

    QUEUE _openList;
    MAP _allNodes;

    void AddAdj(const Node &curr, const GridPos& next, const GridPos& goal, const Heuristic& h)
    {
        IT i = _allNodes.find(next);
        if (i != _allNodes.end())
        {
            float f, g;

            g = curr._g + 1;
            f = g + Distance(h, next, goal);
            if (f < i->second._f)
            {
                i->second._f = f;
                i->second._g = g;
                i->second._parent = &curr;
                _openList.emplace(next, curr, g, f);
            }
        }
    }

    void AddDiag(const Node &curr, const GridPos &next, const GridPos &goal, const Heuristic &h)
    {
        IT i = _allNodes.find(next);
        if (i != _allNodes.end())
        {
            float f, g;

            g = curr._g + sqrtf(2);
            f = g + Distance(h, next, goal);
            if (f < i->second._f)
            {
                i->second._f = f;
                i->second._g = g;
                i->second._parent = &curr;
                _openList.emplace(next, curr, g, f);
            }
        }
    }

    static float Distance(Heuristic const &h, GridPos const &start, GridPos const &end)
    {
        return Distance(h, end.col - start.col, end.row - start.row);
    }

    static float Distance(Heuristic const &h, int const &x, int const &y)
    {
        switch (h)
        {
        case Heuristic::OCTILE:
            return Octile(x, y);
        case Heuristic::CHEBYSHEV:
            return Chebyshev(x, y);
        case Heuristic::MANHATTAN:
            return Manhattan(x, y);
        case Heuristic::EUCLIDEAN:
            return Euclidean(x, y);
        default:
            return 0;
        }
    }

    static float Euclidean(int x, int y)
    {
        return std::sqrtf(static_cast<float>(x * x) + static_cast<float>(y * y));
    }

    static float Manhattan(int x, int y)
    {
        return static_cast<float>(std::abs(x) + std::abs(y));
    }

    static float Chebyshev(int x, int y)
    {
        return static_cast<float>(std::max(std::abs(x), std::abs(y)));
    }

    static float Octile(int x, int y)
    {
        return Manhattan(x, y) + (std::sqrtf(2) - 1) * Chebyshev(x, y);
    }
};