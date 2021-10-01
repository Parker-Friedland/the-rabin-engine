#pragma once

#include <vector>
#include <random>

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

	enum turn
	{
		setup,
		knights,
		bishops,
	};

	enum phase
	{
		start,
		wait,
		capture,
	};

	void ClearBoard()
	{
		for (int i = 0; i < 8; ++i)
			for (int j = 0; j < 8; ++j)
				board[i][j] = 0;
	}

	void PlacePiece(piece p)
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

	bool TryPlacingPiece(piece p, std::pair<int, int> coordinates)
	{
		for (int i = 0; i < piece::num; ++i)
		{
			if (piecePrevPos[i] == coordinates)
				return false;
		}

		piecePrevPos[p] = coordinates;
		return true;
	}

	void GenorateMoves()
	{

	}

	bool KnightsLosing()
	{
		return bishops > knights;
	}

	bool BishopsLosing()
	{
		return knights > bishops;
	}

	char board[8][8];

	std::vector<std::vector<std::pair<int, int>>> moves;

	std::vector<std::pair<int, int>> piecePrevPos;
	std::vector<std::pair<int, int>> pieceNextPos;

	int knights;
	int bishops;

	bool currPieces[4];

	turn turn;
	phase phase;
};