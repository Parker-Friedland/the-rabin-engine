#pragma once
#include "Misc/PathfindingDetails.hpp"

typedef std::priority_queue<Node, std::vector<Node>, std::less<Node>> QUEUE;
typedef std::unordered_map<GridPos, Node> MAP;

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

    static float Distance(Heuristic h, GridPos start, GridPos end)
    {
        return Distance(h, end.col - start.col, end.row - start.row);
    }

    static float Distance(Heuristic h, int x, int y)
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