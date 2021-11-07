#pragma once
#include "Misc/PathfindingDetails.hpp"

//#include "Agent/AStarAgent.h"

#include <bitset>

// Don't like that this had to be public and static but it was the ONLY way this would work
static float weight;
static int grid_width;
static int goal;
static Heuristic h;
static bool debug;

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

    static constexpr int _r_comp[8] = { 0,  1,  0, -1,
                                        1,  1, -1, -1 };
    static constexpr int _c_comp[8] = { 1,  0, -1,  0,
                                        1, -1, -1,  1 };

    static constexpr DirectT cardStart = 0;
    static constexpr DirectT cardEnd = 4;
    static constexpr DirectT diagStart = 4;
    static constexpr DirectT diagEnd = 8;
    static constexpr DirectT numEach = 4;
    static constexpr DirectT numTot = 8;
    static constexpr DirectT done = 8;

    struct NodeCore
    {
        NodeCore() : _c(std::numeric_limits<CountT>::max()), _d(std::numeric_limits<CountT>::max()) {}

        void SetSurrondingTerain(int pos)
        {
            int r = IntToRow(pos);
            int c = IntToCol(pos);

            for (DirectT i = cardStart; i < cardEnd; ++i)
            {
                int row = r + _r_comp[i];
                int col = c + _c_comp[i];
                _valid.set(i, terrain->is_valid_grid_position(row, col) && !terrain->is_wall(row, col));
            }

            for (DirectT i = diagStart; i < diagEnd; ++i)
            {
                if (_valid[i % numEach] && _valid[(i + 1) % numEach])
                {
                    int row = r + _r_comp[i];
                    int col = c + _c_comp[i];
                    _valid.set(i, !terrain->is_wall(row, col));
                }
            }
        }

        void Reset()
        {
            _c = _d = std::numeric_limits<CountT>::max();
        }

        void SetCost(CountT c, CountT d)
        {
            _c = c;
            _d = d;
        }

        void SetStart()
        {
            _c = 0;
            _d = 0;
            _parent = -1;
        }

        NodeCore& operator=(const NodeCore& other) = default;

        ValidT _valid;
        CountT _c; // cardinal
        CountT _d; // diagnal
        //DirectT _parent;
        int _parent;
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

    typedef std::vector<int> PATH;

    QUEUE _openList;
    VECTOR _allNodes;
    std::vector<std::vector<int>> _oracle;

    void InitRequest(const PathRequest& request);
    void FinishRequest(PathRequest& request);
    void MakePath(PathRequest& request);

    PathResult Floyd(PathRequest& request)
    {
        goal = GridToInt(terrain->get_grid_position(request.goal));
        int start = GridToInt(terrain->get_grid_position(request.start));

        if (_oracle[start][goal] == -1)
            return PathResult::IMPOSSIBLE;

        if (request.settings.rubberBanding)
        {
            InitRequest(request);
            FloydCopy(start);
            MakePath(request);
        }
        else
            MakeFloydPath(request, start);

        return PathResult::COMPLETE;
    }

    void MakeFloydPath(PathRequest& request, int start)
    {
        do
            request.path.push_front(terrain->get_world_position(IntToRow(goal), IntToCol(goal)));
        while ((goal = _oracle[goal][start]) >= 0);
    }

    void FloydCopy(int start)
    {
        int next;

        while((next = _oracle[start][goal]) != goal)
        {
            _allNodes[next]._parent = start;
            start = next;
        }

        _allNodes[next]._parent = start;
    }

    int Direction(int child, int parent)
    {
        const int L = 0;
        const int LU = 1;
        const int U = 2;
        const int RU = 3;
        const int R = 4;
        const int RD = 5;
        const int D = 6;
        const int LD = 7;
        const int error = -1;

        int r = IntToRow(parent) - IntToRow(child);
        int c = IntToCol(parent) - IntToCol(child);

        switch (r)
        {
        case -1:
            switch (c)
            {
            case -1:
                return RD;
            case 0:
                return D;
            case 1:
                return LD;
            default:
                return error;
            }
        case 0:
            switch (c)
            {
            case -1:
                return R;
            case 0:
                return done;
            case 1:
                return L;
            default:
                return error;
            }
        case 1:
            switch (c)
            {
            case -1:
                return RU;
            case 0:
                return U;
            case 1:
                return LU;
            default:
                return error;
            }
        default:
            return error;
        }
    }

    struct Cost
    {
        Cost() :
            _c(std::numeric_limits<CountT>::max()),
            _d(std::numeric_limits<CountT>::max())
        {}

        void SetToC()
        {
            _c = 1;
            _d = 0;
        }

        void SetToD()
        {
            _c = 0;
            _d = 1;
        }

        void SetTo0()
        {
            _c = 0;
            _d = 0;
        }

        CountT _c;
        CountT _d;

        bool operator>(const Node& rhs) const
        {
            return static_cast<float>(    _c) + sqrt2 * static_cast<float>(    _d)
                 > static_cast<float>(rhs._c) + sqrt2 * static_cast<float>(rhs._d);
        }

        bool operator<(const Node& rhs) const
        {
            return static_cast<float>(    _c) + sqrt2 * static_cast<float>(    _d)
                 < static_cast<float>(rhs._c) + sqrt2 * static_cast<float>(rhs._d);
        }
    };

    void PreProcess()
    {
        grid_width = terrain->get_map_width();

        const int size = terrain->get_map_width() * terrain->get_map_height();
        _allNodes.clear();
        _allNodes.reserve(size);

        _oracle.clear();
        _oracle.resize(size);

        /*std::vector<std::vector<float>> path =
            std::vector<std::vector<float>>(size,
                std::vector<float>(size, std::numeric_limits<float>::max()));*/

        std::vector<std::vector<Cost>> path =
            std::vector<std::vector<Cost>>(size,
                std::vector<Cost>(size, Cost()));

        for (int i = 0; i < size; ++i)
        {
            _allNodes.emplace_back();
            _allNodes[i].SetSurrondingTerain(i);

            _oracle[i].resize(size);

            for (int j = 0; j < size; ++j)
                _oracle[i][j] = -1;

            path[i][i] = 0;

            int r = IntToRow(i);
            int c = IntToCol(i);

            for (DirectT d = 0; d < numTot; ++d)
            {
                if (_allNodes[i]._valid[d])
                {
                    int j = CoordToInt(r + _r_comp[d], c + _c_comp[d]);
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

        for (int k = 0; k < size; ++k)
            for (int i = 0; i < size; ++i)
                for (int j = 0; j < size; ++j)
                {
                    float newPath = path[i][k] + path[k][j];
                    if (newPath < path[i][j])
                    {
                        bool debug = true;
                    }
                }
    }

    void Rubberbanding();

    inline void ColorInit(int start);
    inline void ColorOpen(int open);
    inline void ColorClosed(int closed);

    void AddNeighboors(Node& curr);

    template <bool diag>
    void AddNeighboor(Node &curr, int pos, int parent)
    {
        NodeCore& core = _allNodes[pos];

        if (CostGiven(curr._c + !diag, curr._d + diag) < CostGiven(core._c, core._d))
        {
            core._c = curr._c + !diag;
            core._d = curr._d +  diag;
            core._parent = parent;
            _openList.emplace(pos, core._c, core._d);

            if (debug)
                ColorOpen(pos);
        }
    }

    void AddCard(Node& curr, int pos, int parent)
    {
        AddNeighboor<false>(curr, pos, parent);
    }

    void AddDiag(Node& curr, int pos, int parent)
    {
        AddNeighboor<true>(curr, pos, parent);
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