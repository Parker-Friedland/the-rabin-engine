#pragma once
#include "Misc/PathfindingDetails.hpp"

#include <cmath>
#include <numbers>
#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

static int grid_width; // Don't like that this had to be public and static but it was the ONLY way this would work
                       //
                       // Can't believe I spent like 3+ hours figuring out how to pass this STUPID variable to GridHash
                       //
                       // Feel like vs is messing with me
                       //
                       // Whoever is grading this, if you know why c++ wouldn't let me just make this variable a static 
                       // member of GridHash and set it in void AStarPather::InitRequest(const PathRequest& request)
                       // please tell me for what reason that wasn't allowed

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
    
    float _weight;
    int _goal;
    Heuristic _h;
    bool _debugColor;

    struct GridHash
    {
        GridHash() {}

        size_t operator() (const GridPos& pos) const
        {
            return std::hash<int>()(grid_width * pos.row + pos.col);
        }
    };

    GridHash _hash;

    struct Node
    {
        Node() : _f(std::numeric_limits<float>::max()) {}

        Node(int pos, int goal, float weight, Heuristic h) :
            _self(pos), _parent(-1), _g(0), _f(weight * AStarPather::Distance_SE(h, pos, goal)) {}

        Node(int pos, int parent, float g, float f) :
            _self(pos), _parent(parent), _g(g), _f(f) {}

        bool IsOpen(std::unordered_map<int, Node> m)
        {
            return _f == m.find(_self)->second._f;
        }

        bool IsOpen()
        {
            return true;
        }

        //GridPos _pos;
        int _self;
        int _parent;
        float _g; // given cost
        float _f; // given cost + heuristic cost

        bool operator>(const Node& rhs) const
        {
            return _f > rhs._f;
        }

        bool operator==(const Node& rhs) const
        {
            return _self == rhs._self;
        }

        bool operator!=(const Node& rhs) const
        {
            return _self != rhs._self;
        }
    };

    // pos
    // parent
    // f
    typedef std::priority_queue<Node, std::vector<Node>, std::greater<Node>> QUEUE;
    // self
    // pos
    // g
    // f
    typedef std::unordered_map<int, Node> MAP;
    // self
    typedef MAP::iterator ITER;

    QUEUE _openList;
    MAP _allNodes;

    bool wait = false;

    void InitRequest(const PathRequest& request);
    void FinishRequest(PathRequest& request);
    void Rubberbanding(PathRequest& request);

    inline void ColorInit(int start);
    inline void ColorOpen(int open);
    inline void ColorClosed(int closed);

    void AddNeighboors(Node& curr);

    template <bool diag>
    void AddNeighboor(Node &curr, int nextPos)
    {
        Node& next = _allNodes[nextPos];
        next._self = nextPos;

        float g = curr._g + (diag ? sqrtf(2) : 1);
        float f = g + (_weight * Distance_SE(_h, nextPos, _goal));

        if (curr._g < 0)
        {
            bool whatthefuck = true;
        }

        if (f < next._f)
        {
            next._f = f;
            next._g = g;
            next._parent = curr._self;
            _openList.emplace(nextPos, curr._self, g, f);

            if (_debugColor)
                ColorOpen(nextPos);
        }
    }

    inline void AddAdj(Node& curr, int next)
    {
        AddNeighboor<false>(curr, next);
    }

    inline void AddDiag(Node& curr, int next)
    {
        AddNeighboor<true>(curr, next);
    }

    static float Distance_SE(Heuristic const &h, int start, int end)
    {
        return Distance_XY(h, IntToCol(end) - IntToCol(start), IntToRow(end) - IntToRow(start));
    }

    static float Distance_XY(Heuristic const &h, int x, int y)
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

    static float Min(int x, int y)
    {
        return static_cast<float>(std::min(std::abs(x), std::abs(y)));
    }

    static float Octile(int x, int y)
    {
        return Chebyshev(x, y) + (std::sqrtf(2) - 1) * Min(x, y);
    }

    inline static int GridToInt(const GridPos& p)
    {
        return grid_width * p.row + p.col;
    }

    inline static const GridPos& IntToGrid(int i)
    {
        return GridPos{ IntToRow(i), IntToCol(i) };
    }

    inline static int IntToRow(int i)
    {
        return i / grid_width;
    }

    inline static int IntToCol(int i)
    {
        return i % grid_width;
    }

    inline static int CoordToInt(int r, int c)
    {
        return grid_width * r + c;
    }
};