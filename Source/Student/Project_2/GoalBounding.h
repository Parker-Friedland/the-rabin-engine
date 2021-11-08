#pragma once

#include "P2_Pathfinding.h"
#include <queue>

typedef std::priority_queue<int, std::vector<int>, std::greater<int>> GB_QUEUE;
typedef std::vector<AStarPather::DirectT> LOG;

static constexpr AStarPather::DirectT unvisited = 8;

struct AStarPather::BoundBox
{
    int r1, r2, c1, c2;

    bool InBounds(int pos)
    {
        int r = AStarPather::IntToRow(pos);
        int c = AStarPather::IntToCol(pos);

        return r >= r1 && r <= r2
            && c >= c1 && c <= c2;
    }

    void UpdateBounds(int r, int c)
    {
        if (r < r1)
            r1 = r;
        if (r > r2)
            r2 = r;

        if (c < c1)
            c1 = c;
        if (c > c2)
            c2 = c;
    }
};

struct AStarPather::GoalBound
{
    BoundBox boxes[numTot];

    bool InBounds(int pos, DirectT d)
    {
        return boxes[d].InBounds(pos);
    }

    GoalBound(VECTOR& terrain, LOG &log, GB_QUEUE& diags, GB_QUEUE& cards, int pos)
    {
        for (auto i = log.begin(); i < log.end(); ++i)
            *i = unvisited;

        auto start = terrain[pos];

        int r = IntToRow(pos);
        int c = IntToCol(pos);

        for (DirectT d = 0; d < numTot; ++d)
            if (start._valid[d])
            {
                int next = CoordToInt(r + _r_comp[d], c + _c_comp[d]);
                if (log[next] == unvisited)
                {
                    log[next] = d;
                    if (d < numEach)
                        cards.emplace(next);
                    else
                        diags.emplace(next);
                }
            }

        CalculateBounds(terrain, log, diags, cards);
    }

    void CalculateBounds(VECTOR& terrain, LOG& log, GB_QUEUE& diags, GB_QUEUE& cards)
    {
        while (!diags.empty() || !cards.empty())
        {
            int curr;

            if (!diags.empty())
            {
                curr = diags.top();
                diags.pop();
            }
            else
            {
                curr = cards.top();
                cards.pop();
            }

            int r = IntToRow(curr);
            int c = IntToCol(curr);

            DirectT direction = log[curr];
            boxes[direction].UpdateBounds(r, c);
            AddNeighboors(log, diags, cards, terrain[curr], r, c, direction);
        }
    }

    void AddNeighboors(LOG& log, GB_QUEUE &diags, GB_QUEUE &cards, const NodeCore &terrain, int r, int c, DirectT direction)
    {
        for (DirectT d = 0; d < numTot; ++d)
            if (terrain._valid[d])
            {
                int next = CoordToInt(r + _r_comp[d], c + _c_comp[d]);
                if (log[next] == unvisited)
                {
                    log[next] = direction;
                    if (d < numEach)
                        cards.emplace(next);
                    else
                        diags.emplace(next);
                }
            }
    }
};

void AStarPather::PreProcessGoalBounds(int size)
{
    GB_QUEUE diags = GB_QUEUE();
    GB_QUEUE cards = GB_QUEUE();
    LOG log = LOG();
    log.resize(size);

    _goalBounds.clear();

    for (int i = 0; i < size; ++i)
    {
        _goalBounds.emplace_back(_allNodes, log, diags, cards, i);
    }
}

