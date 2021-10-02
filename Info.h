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
		first = 0,
		knight1 = 0,
		knight2,
		bishop1,
		bishop2,
		num,

		none = -1
	};

	piece GetPiece(int id)
	{
		switch (id)
		{
		case 0:
			return piece::knight1;
		case 1:
			return piece::knight2;
		case 2:
			return piece::bishop1;
		case 3:
			return piece::bishop2;
		default:
			return piece::none;
		}
	}

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
		std::pair<int, int> coordinates;

		for (int p = piece::knight1; p <= piece::knight2; ++p)
		{
			if (states[p] != state::dead)
			{
				for (int x = -1; x <= 1; x += 2)
				{
					for (int y = -1; y <= 1; y += 2)
					{
						coordinates = nextPos[p];
						coordinates.first += 2 * x;
						coordinates.second += y;
						if (InBounds(coordinates))
							moves[p].push_back(coordinates);

						coordinates = nextPos[p];
						coordinates.first += x;
						coordinates.second += 2 * y;
						if (InBounds(coordinates))
							moves[p].push_back(coordinates);
					}
				}
			}
		}

		for (int p = piece::bishop1; p <= piece::bishop2; ++p)
		{
			if (states[p] != state::dead)
			{
				for (int i = 1; i < BOARD_SIZE; ++i)
				{
					for (int x = -1; x <= 1; x += 2)
					{
						for (int y = -1; y <= 1; y += 2)
						{
							coordinates = nextPos[p];
							coordinates.first += i*x;
							coordinates.second += i*y;
							if (InBounds(coordinates))
								moves[p].push_back(coordinates);
						}
					}
				}
			}
		}
	}

	void GenorateBoard()
	{
		ClearBoard();

		for (int p = piece::first; p < piece::num; ++p)
		{
			for (int i = 0; i < moves[p].size(); ++i)
			{
				board[p][moves[p][i].first][moves[p][i].second] = true;
			}
		}
	}

	void ClearBoard()
	{
		for(int p = piece::first; p < piece::num; ++p)
			for (int i = 0; i < BOARD_SIZE; ++i)
				for (int j = 0; j < BOARD_SIZE; ++j)
					board[p][i][j] = false;
	}

	bool InBounds(std::pair<int, int> coordinates)
	{
		return coordinates.first >= 0
			&& coordinates.first < BOARD_SIZE
			&& coordinates.second >= 0
			&& coordinates.second < BOARD_SIZE;
	}

	bool KnightsLosing()
	{
		return numBishops > numKnights;
	}

	bool BishopsLosing()
	{
		return numKnights > numBishops;
	}

	bool board[piece::num][BOARD_SIZE][BOARD_SIZE];

	std::vector<std::vector<std::pair<int, int>>> moves;

	std::vector<std::pair<int, int>> prevPos;
	std::vector<std::pair<int, int>> nextPos;

	std::vector<state> states;

	int numKnights;
	int numBishops;

	turn turn;
	phase phase;
};