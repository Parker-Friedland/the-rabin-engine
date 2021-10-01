#pragma once

#include <vector>
#include <random>

#define BOARD_SIZE 8;

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
		switch (p)
		{
		case Info::knight1:
		case Info::knight2:
			do
			{ }
			while(!TryPlacingPiece(p, std::pair<int, int>(std::rand() % BOARD_SIZE, std::rand() % BOARD_SIZE)))
			break;
		case Info::bishop1:
			do
			{
			} while (!TryPlacingPiece(p, std::pair<int, int>(std::rand() % (BOARD_SIZE/2), std::rand() % (BOARD_SIZE/2))))
				break;
		case Info::bishop2:
			break;
		}
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

	/*bool PieceOnBoard(piece p)
	{
		return piecePrevPos[p].first >= 0;
	}*/

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

	std::vector<std::pair<int, int>> knightMoves;
	std::vector<std::pair<int, int>> bishopsMoves;

	std::vector<std::pair<int, int>> piecePrevPos;
	std::vector<std::pair<int, int>> pieceNextPos;

	std::pair<int, int> knight1prev;
	std::pair<int, int> knight1next;
	std::pair<int, int> knight2prev;
	std::pair<int, int> knight2next;

	std::pair<int, int> bishop1prev;
	std::pair<int, int> bishop1next;
	std::pair<int, int> bishop2prev;
	std::pair<int, int> bishop2next;

	int knights;
	int bishops;

	bool currPieces[4];

	turn turn;
	phase phase;
};