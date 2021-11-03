#pragma once
#include "Misc/PathfindingDetails.hpp"

#include <bitset>

// Don't like that this had to be public and static but it was the ONLY way this would work
static float weight;
static int grid_width;
static int goal;
static Heuristic h;
static bool debug;
static bool reset_fw;

static int pos;

//static bool red = true;

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

    typedef std::bitset<8> ValidT;
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
    static constexpr int numTot = 8;
    static constexpr int done = 8;

    //int _x_comp[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
    //int _y_comp[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };

    struct NodeCore
    {
        NodeCore() : _c(std::numeric_limits<CountT>::max()), _d(std::numeric_limits<CountT>::max())
        {
            int x = IntToCol(pos);
            int y = IntToRow(pos);

            for (DirectT i = cardStart; i < cardEnd; ++i)
            {
                int col = x + _x_comp[i];
                int row = y + _y_comp[i];
                _valid.set(i, terrain->is_valid_grid_position(row, col) && !terrain->is_wall(row, col));
            }

            for (DirectT i = diagStart; i < diagEnd; ++i)
            {
                if (_valid[i % numEach] && _valid[(i + 1) % numEach])
                {
                    int col = x + _x_comp[i];
                    int row = y + _y_comp[i];
                    _valid.set(i, !terrain->is_wall(row, col));
                }
            }
        }

        void Reset()
        {
            _c = _d = std::numeric_limits<CountT>::max();
        }

        //NodeCore(CountT c, CountT d) : _c(c), _d(d) {}

        void SetCost(CountT c, CountT d)
        {
            _c = c;
            _d = d;
        }

        //NodeCore(const NodeCore& other) : _c(other._c), _d(other._d) {}

        void SetStart()
        {
            _c = 0;
            _d = 0;
            _parent = done;
        }

        NodeCore& operator=(const NodeCore& other) = default;

        ValidT _valid;
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
    std::vector<std::vector<int>> _oracle;

    void InitRequest(const PathRequest& request);
    void FinishRequest(PathRequest& request);
    void FinishFloyedRequest(PathRequest& request)
    {

    }

    void PreProcess()
    {
        grid_width = terrain->get_map_width();

        const int size = terrain->get_map_width() * terrain->get_map_height();
        _allNodes.clear();
        _allNodes.reserve(size);
        _oracle.resize(size);

        std::vector<std::vector<float>> path =
            std::vector<std::vector<float>>(size,
                std::vector<float>(size, std::numeric_limits<int>::max()));

        for (int i = 0; i < size; ++i)
        {
            _allNodes.emplace_back();

            _oracle[i].resize(size);

            path[i][i] = 0;

            int y = IntToCol(i);
            int x = IntToRow(i);

            for (DirectT d = 0; d < numTot; ++d)
            {
                if (_allNodes[d]._valid[i])
                {
                    int j = CoordToInt(y + _y_comp[d], x + _x_comp[d]);
                    path[i][j] = d < numEach ? 1.f : sqrt2;
                    _oracle[i][j] = j;
                }
            }
        }

        for (int k = 0; k < size; ++k)
            for (int i = 0; i < size; ++i)
                for (int j = 0; j < size; ++j)
                {
                    float newPath = path[i][k] + path[k][j];
                    if (newPath < path[i][j])
                    {
                        path[i][j] = newPath;
                        _oracle[i][j] = k;
                    }
                }
    }

    void Floyd(PathRequest& request)
    {

    }

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

    void AddCard(Node& curr, int pos, DirectT direct)
    {
        AddNeighboor<false>(curr, pos, direct);
    }

    void AddDiag(Node& curr, int pos, DirectT direct)
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

    static int GridToInt(const GridPos& p)
    {
        return grid_width * p.row + p.col;
    }

    static const GridPos& IntToGrid(int i)
    {
        return GridPos{ IntToRow(i), IntToCol(i) };
    }

    static int IntToRow(int i)
    {
        return i / grid_width;
    }

    static int IntToCol(int i)
    {
        return i % grid_width;
    }

    static int CoordToInt(int r, int c)
    {
        return grid_width * r + c;
    }
};