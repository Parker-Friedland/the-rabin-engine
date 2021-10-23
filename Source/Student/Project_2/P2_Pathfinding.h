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


#include <pch.h>
#include <sstream>
#include "Misc/Stopwatch.h"
#include <iomanip>
#include <fstream>

// Don't like that this had to be public and static but it was the ONLY way this would work
static float weight;
static int grid_width;
static int goal;
static Heuristic h;
static bool debug;

static bool red = true;

//static Stopwatch pathtimer;

//static std::chrono::nanoseconds::rep initTime;
//static std::chrono::nanoseconds::rep addTime;
//static std::chrono::nanoseconds::rep finalTime;

static const float sqrt2 = std::sqrtf(2);

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

    bool _debugColor;

    typedef int CountT;
    typedef char DirectT;

    static constexpr int _x_comp[8] = { 1,  0, -1,  0,
                                        1, -1, -1,  1 };
    static constexpr int _y_comp[8] = { 0,  1,  0, -1,
                                        1,  1, -1, -1 };

    static constexpr int cardStart = 0;
    static constexpr int cardEnd = 4;
    static constexpr int diagStart = 4;
    static constexpr int diagEnd = 8;
    static constexpr int numEach = 4;
    static constexpr int done = 8;

    //int _x_comp[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
    //int _y_comp[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };

    struct NodeCore
    {
        NodeCore() : _c(std::numeric_limits<CountT>::max()), _d(std::numeric_limits<CountT>::max()) {}

        NodeCore(CountT c, CountT d) : _c(c), _d(d) {}

        NodeCore(const NodeCore& other) : _c(other._c), _d(other._d) {}

        void SetStart()
        {
            _c = 0;
            _d = 0;
            _parent = done;
        }

        NodeCore& operator=(const NodeCore& other) = default;

        CountT _c; // cardinal
        CountT _d; // diagnal
        DirectT _parent;
    };

    //NodeCore unexplored = { std::numeric_limits<CountT>::max(), std::numeric_limits<CountT>::max() };

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
            const NodeCore& core = v[_pos];
            return core._c == _c
                && core._d == _d;
        }

        PosT _pos;
        CountT _c;
        CountT _d;

        bool operator>(const Node& rhs) const
        {
            return AStarPather::CostEst(_c, _d, _pos)
                 > AStarPather::CostEst(rhs._c, rhs._d, rhs._pos);
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

    //Stopwatch mytimer;

    //static std::chrono::microseconds::rep initTime;
    void InitRequest(const PathRequest& request);

    //static std::chrono::microseconds::rep finishTime;
    void FinishRequest(PathRequest& request);

    void Rubberbanding(PathRequest& request);

    inline void ColorInit(int start);
    inline void ColorOpen(int open);
    inline void ColorClosed(int closed);

    //static std::chrono::microseconds::rep addTime;
    void AddNeighboors(Node& curr);

    template <bool diag>
    void AddNeighboor(Node &curr, int pos, DirectT direct)
    {
        NodeCore& core = _allNodes[pos];

        if (CostGiven(curr._c + !diag, curr._d + diag) < CostGiven(core._c, core._d))
        {
            core._c = curr._c + !diag;
            core._d = curr._d +  diag;
            core._parent = direct;
            _openList.emplace(pos, core._c, core._d);

            if (debug)
                ColorOpen(pos);
        }
    }

    inline void AddCard(Node& curr, int pos, DirectT direct)
    {
        AddNeighboor<false>(curr, pos, direct);
    }

    inline void AddDiag(Node& curr, int pos, DirectT direct)
    {
        AddNeighboor<true>(curr, pos, direct);
    }

    static float CostEst(int c, int d, int pos)
    {
        return CostGiven(c, d) + weight * DistanceToGoal(pos);
    }

    static float CostGiven(int c, int d)
    {
        return c + sqrt2 * d;
    }

    static float DistanceToGoal(int pos)
    {
        return Distance_XY(IntToCol(goal) - IntToCol(pos), IntToRow(goal) - IntToRow(pos));
    }

    static float Distance_XY(int x, int y)
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