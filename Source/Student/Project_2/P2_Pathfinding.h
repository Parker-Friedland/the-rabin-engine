#pragma once
#include "Misc/PathfindingDetails.hpp"

#include <cmath>
#include <numbers>
#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

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

    struct Node
    {
        Node(GridPos pos, Heuristic h, GridPos goal) :
            _pos(pos), _parent(nullptr), _g(0), _f(AStarPather::Distance(h, pos, goal)) {}

        Node(GridPos pos, Node* parent, float g, float f) :
            _pos(pos), _parent(parent), _g(g), _f(f) {}

        Node(GridPos pos, Node* parent, Heuristic h, GridPos goal, bool diag) :
            _pos(pos), _parent(parent), _g(parent->_g + diag ? sqrtf(2) : 1), _f(_g + AStarPather::Distance(h, pos, goal)) {}

        struct Hash
        {
            Hash() : _grid_width(terrain->get_map_width()) {}

            int _grid_width;

            size_t operator() (const Node& node) const
            {
                return std::hash<int>()(_grid_width * node._pos.row + node._pos.col);
            }
        };

        bool IsOpen(std::unordered_set<Node, Hash> s)
        {
            return _f == s.find(*this)->_f;
        }

        bool IsOpen()
        {
            return true;
        }

        GridPos _pos;
        Node* _parent;
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

    struct GridHash
    {
        GridHash() : _grid_width(terrain->get_map_width()) {}

        int _grid_width;

        size_t operator() (const GridPos& pos) const
        {
            return std::hash<int>()(_grid_width * pos.row + pos.col);
        }
    };

    typedef std::priority_queue<Node, std::vector<Node>, std::less<Node>> QUEUE;
    typedef std::unordered_set<Node, Node::Hash> SET;

    typedef std::unordered_map<GridPos, Node, GridHash> MAP;
    typedef MAP::iterator ITER;

    QUEUE _openList;
    MAP _allNodes;

    template <bool diag>
    void AddNeighboor(Node &curr, const GridPos& next, const GridPos& goal, const Heuristic& h)
    {
        float g = curr._g + diag?sqrtf(2):1;
        float f = g + Distance(h, next, goal);

        ITER i;
        if ((i = _allNodes.find(next)) != _allNodes.end())
        {
            if (f >= i->second._f)
                return;

            i->second._f = f;
            i->second._g = g;
            i->second._parent = &curr;
        }
        
        _openList.emplace(next, &curr, g, f);
    }

    void AddAdj(Node& curr, const GridPos& next, const GridPos& goal, const Heuristic& h)
    {
        AddNeighboor<false>(curr, next, goal, h);
    }

    void AddDiag(Node& curr, const GridPos& next, const GridPos& goal, const Heuristic& h)
    {
        AddNeighboor<true>(curr, next, goal, h);
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