#pragma once

#include <vector>
#include <random>

#include <pch.h>
#include "Projects/ProjectOne.h"
#include "Agent/CameraAgent.h"

static constexpr int BOARD_SIZE = 8;
static constexpr int MAX_PROB = 100;

struct Info
{
	enum Piece
	{
		First = 0,
		Knight1 = 0,
		Knight2,
		Bishop1,
		Bishop2,
		Num,

		None = -1
	};

	Piece GetPiece(int id)
	{
		switch (id)
		{
		case 0:
			return Piece::Knight1;
		case 1:
			return Piece::Knight2;
		case 2:
			return Piece::Bishop1;
		case 3:
			return Piece::Bishop2;
		default:
			return Piece::None;
		}
	}

	enum State
	{
		Idle,
		Threatened,
		Captured,
		Num
	};

	enum Turn
	{
		Setup,
		Knights,
		Bishops,
		Num
	};

	enum Phase
	{
		Start,
		Wait,
		Capture,
		Num
	};

	void CreateAgents()
	{
		BehaviorAgent* agent;
		
		agent = agents->create_behavior_agent("K1", BehaviorTreeTypes::Example);
		agent->set_position(GetTranslationVec(CreatePiece(Piece::Knight1)));

		agent = agents->create_behavior_agent("K2", BehaviorTreeTypes::Example);
		agent->set_position(GetTranslationVec(CreatePiece(Piece::Knight2)));

		agent = agents->create_behavior_agent("B1", BehaviorTreeTypes::Example);
		agent->set_position(GetTranslationVec(CreatePiece(Piece::Bishop1)));

		agent = agents->create_behavior_agent("B2", BehaviorTreeTypes::Example);
		agent->set_position(GetTranslationVec(CreatePiece(Piece::Bishop2)));
	}

	std::pair<int, int> CreatePiece(Piece p)
	{
		std::pair<int, int> coordinates;

		do
		{
			coordinates.first = std::rand() % BOARD_SIZE;

			switch (p)
			{
			case Info::Knight1:
			case Info::Knight2:
				coordinates.second = std::rand() % BOARD_SIZE;
			case Info::Bishop1:
				coordinates.second = 2 * (std::rand() % (BOARD_SIZE/2))
					+ (coordinates.first % 2) ? 0 : 1;
			case Info::Bishop2:
				coordinates.second = 2 * (std::rand() % (BOARD_SIZE / 2))
					+ (coordinates.first % 2) ? 1 : 0;
			}
		} while (!TryPlacingPiece(p, coordinates));
	}

	Vec3 GetTranslationVec(std::pair<int, int> coordinates)
	{
		return Vec3(0, 0, 0);
	}

	bool TryPlacingPiece(Piece p, std::pair<int, int> coordinates)
	{
		for (int i = 0; i < Piece::Num; ++i)
		{
			if (prevPos[i] == coordinates)
				return false;
		}

		prevPos[p] = coordinates;
		return true;
	}

	void InitTurn()
	{
		GenorateMoves();
		GenorateBoard();
		InitTargets();
		UpdateDodgeFlag();
		turn = Turn::Setup;
		phase = Phase::Start;
	}

	void GenorateMoves()
	{
		std::pair<int, int> coordinates;

		for (int p = Piece::Knight1; p <= Piece::Knight2; ++p)
		{
			if (states[p] != State::Captured)
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

		for (int p = Piece::Bishop1; p <= Piece::Bishop2; ++p)
		{
			if (states[p] != State::Captured)
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

		for (int p = Piece::First; p < Piece::Num; ++p)
		{
			for (int i = 0; i < moves[p].size(); ++i)
			{
				board[p][moves[p][i].first][moves[p][i].second] = true;
			}
		}
	}

	void ClearBoard()
	{
		for(int p = Piece::First; p < Piece::Num; ++p)
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

	bool IsLoosing()
	{
		switch (turn)
		{
		case Info::Knights:
			return KnightsLosing();
		case Info::Bishops:
			return BishopsLosing();
		}
	}

	bool KnightsLosing()
	{
		return numBishops > numKnights;
	}

	bool BishopsLosing()
	{
		return numKnights > numBishops;
	}

	void InitTargets()
	{
		capture = Piece::None;
		fork1 = Piece::None;
		fork2 = Piece::None;
	}

	bool UpdateDodgeFlag()
	{
		allowDodges = dodgeProb < std::rand() % MAX_PROB;
		if (allowDodges)
			--dodgeProb;
	}

	void UpdateTurn()
	{
		switch (turn)
		{
		case Turn::Setup:
			turn = Turn::Knights;
			break;
		case Turn::Knights:
			turn = Turn::Bishops;
			break;
		case Turn::Bishops:
			turn = Turn::Knights;
			break;
		}

		phase = Phase::Start;
	}

	void UpdatePhase()
	{
		switch (phase)
		{
		case Phase::Start:
			phase = Phase::Wait;
			break;
		case Phase::Wait:
			phase = Phase::Capture;
			break;
		case Phase::Capture:
			phase = Phase::Start;
			break;
		}
	}

	bool board[Piece::Num][BOARD_SIZE][BOARD_SIZE];

	std::vector<std::vector<std::pair<int, int>>> moves;

	std::vector<std::pair<int, int>> prevPos;
	std::vector<std::pair<int, int>> nextPos;

	std::vector<State> states;

	int numKnights;
	int numBishops;

	Turn turn;
	Phase phase;

	Piece capture;
	Piece fork1;
	Piece fork2;

	int dodgeProb = MAX_PROB - 1;
	bool allowDodges;
};