#pragma once

#include <vector>
#include <random>

#include <pch.h>
#include "Projects/ProjectOne.h"
#include "Agent/CameraAgent.h"

static constexpr int BOARD_SIZE = 8;

struct Info
{
	enum piece
	{
		knight1 = 0,
		knight2,
		bishop1,
		bishop2,
		num
	};

	enum state
	{
		idle,
		threatened,
		dead,
		num
	};

	enum turn
	{
		setup,
		knights,
		bishops,
		num
	};

	enum phase
	{
		start,
		wait,
		capture,
		num
	};

	void ClearBoard()
	{
		for (int i = 0; i < 8; ++i)
			for (int j = 0; j < 8; ++j)
				board[i][j] = 0;
	}

	void CreateAgents()
	{
		BehaviorAgent* agent;
		
		agent = agents->create_behavior_agent("Knight1", BehaviorTreeTypes::Example);
		agent->set_position(GetTranslationVec(CreatePiece(piece::knight1)));

		agent = agents->create_behavior_agent("Knight2", BehaviorTreeTypes::Example);
		agent->set_position(GetTranslationVec(CreatePiece(piece::knight2)));

		agent = agents->create_behavior_agent("Bishop1", BehaviorTreeTypes::Example);
		agent->set_position(GetTranslationVec(CreatePiece(piece::bishop1)));

		agent = agents->create_behavior_agent("Bishop2", BehaviorTreeTypes::Example);
		agent->set_position(GetTranslationVec(CreatePiece(piece::bishop2)));
	}

	std::pair<int, int> CreatePiece(piece p)
	{
		std::pair<int, int> coordinates;

		do
		{
			coordinates.first = std::rand() % BOARD_SIZE;

			switch (p)
			{
			case Info::knight1:
			case Info::knight2:
				coordinates.second = std::rand() % BOARD_SIZE;
			case Info::bishop1:
				coordinates.second = 2 * (std::rand() % (BOARD_SIZE/2))
					+ (coordinates.first % 2) ? 0 : 1;
			case Info::bishop2:
				coordinates.second = 2 * (std::rand() % (BOARD_SIZE / 2))
					+ (coordinates.first % 2) ? 1 : 0;
			}
		} while (!TryPlacingPiece(p, coordinates));
	}

	Vec3 GetTranslationVec(std::pair<int, int> coordinates)
	{
		return Vec3(0, 0, 0);
	}

	bool TryPlacingPiece(piece p, std::pair<int, int> coordinates)
	{
		for (int i = 0; i < piece::num; ++i)
		{
			if (prevPos[i] == coordinates)
				return false;
		}

		prevPos[p] = coordinates;
		return true;
	}

	void GenorateMoves()
	{
		for(int i = 0; i < piece)
	}

	void GenorateBoard()
	{

	}

	bool KnightsLosing()
	{
		return numBishops > numKnights;
	}

	bool BishopsLosing()
	{
		return numKnights > numBishops;
	}

	char board[8][8];

	std::vector<std::vector<std::pair<int, int>>> moves;

	std::vector<std::pair<int, int>> prevPos;
	std::vector<std::pair<int, int>> nextPos;

	std::vector<state> states;

	int numKnights;
	int numBishops;

	bool currPieces[4];

	piece pieceCreated;

	turn turn;
	phase phase;
};