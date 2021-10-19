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
        return std::sqrtf(x * x + y * y);
    }

    static float Manhattan(int x, int y)
    {
        return std::abs(x) + std::abs(y);
    }

    static float Chebyshev(int x, int y)
    {
        return std::max(std::abs(x), std::abs(y));
    }

    static float Octile(int x, int y)
    {
        return Manhattan(x, y) + (std::sqrtf(2) - 1) * Chebyshev(x, y);
    }
};