#pragma once
#include "Misc/PathfindingDetails.hpp"

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

    template <bool diag>
    void AddAdj(const Node &curr, const GridPos& next, const GridPos& goal, const Heuristic& h)
    {
        float g = curr._g + constexpr(diag?sqrt(2):1);
        float f = g + Distance(h, next, goal);

        IT i;
        if ((i = _allNodes.find(next)) != _allNodes.end())
        {
            if (f >= i->second._f)
                return;

            i->second._f = f;
            i->second._g = g;
            i->second._parent = &curr;
        }
        
        _openList.emplace(next, curr, g, f);
    }

    void AddDiag(const Node &curr, const GridPos &next, const GridPos &goal, const Heuristic &h)
    {
        float g = curr._g + sqrtf(2);
        float f = g + Distance(h, next, goal);

        IT i;
        if ((i = _allNodes.find(next)) != _allNodes.end())
        {
            if (f >= i->second._f)
                return;

            i->second._f = f;
            i->second._g = g;
            i->second._parent = &curr;
        }

        _openList.emplace(next, curr, g, f);
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