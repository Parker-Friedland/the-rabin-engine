#pragma once
#include "Misc/PathfindingDetails.hpp"

#include <cmath>
#include <numbers>
#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <bitset>
#include <iostream>
#include <cstring>

static int grid_width; // Don't like that this had to be public and static but it was the ONLY way this would work
                       //
                       // Can't believe I spent like 3+ hours figuring out how to pass this STUPID variable to GridHash
                       //
                       // Feel like vs is messing with me
                       //
                       // Whoever is grading this, if you know why c++ wouldn't let me just make this variable a static 
                       // member of GridHash and set it in void AStarPather::InitRequest(const PathRequest& request)
                       // please tell me for what reason that wasn't allowed

static float sqrt2 = std::sqrtf(2);

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
    
    static float _weight;
    static int _start;
    static int _goal;
    static Heuristic _h;
    bool _debugColor;

    typedef int CountT;
    typedef char ParentT;

    struct NodeCore
    {
        NodeCore() : _c(0), _d(0) {};
        NodeCore(CountT c, CountT d) : _c(c), _d(d) {};

        CountT _c; // cardinal
        CountT _d; // diagnal
        ParentT _parent;
    };

    typedef std::vector<NodeCore> VECTOR;

    typedef int PosT;

    struct Node
    {
        Node(PosT pos) :
            _pos(pos), _c(0), _d(0) {}

        Node(PosT pos, CountT c, CountT d) :
            _pos(pos), _c(c), _d(d) {}

        bool IsOpen(VECTOR v)
        {
            const NodeCore& node = v[_pos];
            return node._c == _c
                && node._d == _d;
        }

        PosT _pos;
        CountT _c;
        CountT _d;

        bool operator>(const Node& rhs) const
        {
            return AStarPather::GetCostEst(_c, _d, _pos) 
                 > AStarPather::GetCostEst(rhs._c, rhs._d, rhs._pos);
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

    typedef std::priority_queue<Node, std::vector<Node>, std::greater<Node>> QUEUE;

    QUEUE _openList;
    VECTOR _allNodes;

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

    static float GetCostEst(int c, int d, int start)
    {
        return c + sqrt2 * d + DistanceToGoal(start);
    }

    static float DistanceToGoal(int pos)
    {
        return Distance_XY(IntToCol(_goal) - IntToCol(pos), IntToRow(_goal) - IntToRow(pos));
    }

    static float Distance_XY(int x, int y)
    {
        switch (_h)
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