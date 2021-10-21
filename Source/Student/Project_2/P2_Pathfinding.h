#pragma once
#include "Misc/PathfindingDetails.hpp"

#include <cmath>
#include <numbers>
#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

struct GridHash
{
    GridHash();
    int _grid_width;
    size_t operator() (const GridPos& pos) const;
};

struct Node
{
    Node();
    Node(const GridPos& pos, const Heuristic& h, const GridPos& goal);
    Node(const GridPos& pos, Node& parent, float g, float f);
    Node(const GridPos& pos, Node& parent, Heuristic h, const GridPos& goal, bool diag);
    bool IsOpen(std::unordered_map<GridPos, Node, GridHash> m);
    bool IsOpen();

    GridPos _pos;
    Node* _parent;
    float _g; // given cost
    float _f; // given cost + heuristic cost

    bool operator<(const Node& rhs) const;
    bool operator==(const Node& rhs) const;
    bool operator!=(const Node& rhs) const;
};

using QUEUE = std::priority_queue<Node, std::vector<Node>, std::less<Node>>;
using MAP = std::unordered_map<GridPos, Node, GridHash>;
using ITER = MAP::iterator;

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
    
    GridPos _start;
    GridPos _goal;
    Heuristic _h;
    bool _debugColor;

    QUEUE _openList;
    MAP _allNodes;

    void InitRequest(const PathRequest& request);
    void ProcessRequest(Node& curr);
    void FinishRequest(PathRequest& request, const Node& goal);

    inline void ColorInit();
    inline void ColorOpen(const GridPos& open);
    inline void ColorClosed(const GridPos& closed);

    template <bool diag>
    void AddNeighboor(Node &curr, const GridPos& nextPos)
    {
        Node& next = _allNodes[nextPos];
        next._pos = nextPos;

        float g = curr._g + diag ? sqrtf(2) : 1;
        float f = g + Distance(_h, nextPos, _goal);

        if (f < next._f)
        {
            next._f = f;
            next._g = g;
            next._parent = &curr;
            _openList.emplace(nextPos, curr, g, f);

            if (_debugColor)
                ColorOpen(nextPos);
        }
    }

    inline void AddAdj(Node& curr, const GridPos& next)
    {
        AddNeighboor<false>(curr, next);
    }

    inline void AddDiag(Node& curr, const GridPos& next)
    {
        AddNeighboor<true>(curr, next);
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