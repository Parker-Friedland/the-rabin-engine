#pragma once

#include <vector>

struct Info
{
	void ClearBoard()
	{
		for (int i = 0; i < 8; ++i)
			for (int j = 0; j < 8; ++j)
				board[i][j] = 0;
	}

	void PlacePiece()
	{

	}

	bool PieceOnBoard(piece p)
	{
		
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

	std::vector<std::pair<int, int>> knightMoves;
	std::vector<std::pair<int, int>> bishopsMoves;

	std::pair<int, int> knight1prex;
	std::pair<int, int> knight1next;
	std::pair<int, int> knight2prev;
	std::pair<int, int> knight2next;

	std::pair<int, int> bishop1prev;
	std::pair<int, int> bishop1next;
	std::pair<int, int> bishop2prev;
	std::pair<int, int> bishop2next;

	int knights;
	int bishops;

	enum piece
	{
		knight1,
		knight2,
		bishop1,
		bishop2,
	};

	bool currPieces[4];

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

	turn turn;
	phase phase;
};