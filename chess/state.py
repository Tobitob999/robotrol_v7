from chess.types import ChessState


def board_to_fen(board, white_bottom: bool = True):
    size = len(board)
    if size == 0:
        return "8/8/8/8/8/8/8/8"
    rows = board
    if white_bottom:
        rows = board
    else:
        rows = list(reversed(board))

    fen_rows = []
    for r in range(size):
        empty = 0
        row = ""
        for c in range(size):
            piece = rows[r][c]
            if piece == ".":
                empty += 1
            else:
                if empty:
                    row += str(empty)
                    empty = 0
                row += piece
        if empty:
            row += str(empty)
        fen_rows.append(row)
    return "/".join(fen_rows)


def build_state(board, white_bottom: bool, confidence: float):
    fen = board_to_fen(board, white_bottom=white_bottom)
    return ChessState(board=board, fen=fen, confidence=confidence)
