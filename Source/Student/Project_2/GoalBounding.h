#pragma once

#include "P2_Pathfinding.h"
#include <queue>

typedef std::priority_queue<int, std::vector<int>, std::greater<int>> GB_QUEUE;
typedef std::vector<AStarPather::DirectT> LOG;

static constexpr AStarPather::DirectT unvisited = 8;

void AStarPather::PreProcessGoalBounds(int size)
{
    GB_QUEUE diags = GB_QUEUE();
    GB_QUEUE cards = GB_QUEUE();
    LOG log = LOG();
    log.resize(size);

    _goalBounds.clear();
    _goalBounds.reserve(size);

    for (int i = 0; i < size; ++i)
    {
        _goalBounds.emplace_back(_allNodes, log, diags, cards, i);
    }
}

