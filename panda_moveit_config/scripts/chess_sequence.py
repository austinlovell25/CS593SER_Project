#!/usr/bin/env python3
"""Run a Stockfish-vs-Stockfish chess sequence.

This script keeps the board state in python-chess, asks a UCI chess engine for
each move, and emits a placeholder command object for the future robot bridge.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

try:
    import chess
    import chess.engine
except ImportError as exc:
    raise SystemExit(
        "Missing python-chess. Rebuild the Docker image after this change, or install "
        "the dependency with: python3 -m pip install chess"
    ) from exc


WHITE = "white"
BLACK = "black"


@dataclass(frozen=True)
class BoardPose:
    x: float
    y: float
    z: float
    yaw: float = 0.0

    def compact(self) -> str:
        return f"({self.x:.3f}, {self.y:.3f}, {self.z:.3f})"


@dataclass(frozen=True)
class PieceTransfer:
    model_name: str
    from_square: str
    to_square: str
    from_pose: BoardPose
    to_pose: BoardPose
    reason: str = "move"


@dataclass(frozen=True)
class RobotMoveCommand:
    ply: int
    fullmove_number: int
    color: str
    move_uci: str
    move_san: str
    moving_piece: str
    moving_model: str
    transfer: PieceTransfer
    captured_piece: Optional[str]
    captured_model: Optional[str]
    capture_square: Optional[str]
    capture_pose: Optional[BoardPose]
    promotion: Optional[str]
    is_castling: bool
    is_en_passant: bool
    supporting_transfers: Tuple[PieceTransfer, ...]
    fen_before: str


@dataclass(frozen=True)
class EngineSettings:
    path: str
    skill_level: Optional[int]
    uci_elo: Optional[int]
    depth: Optional[int]
    think_time: float


class DryRunRobotBridge:
    """Placeholder for the future robot/Gazebo execution layer."""

    def __init__(self, verbose: bool) -> None:
        self.verbose = verbose

    def execute_move(self, command: RobotMoveCommand) -> None:
        if not self.verbose:
            return

        capture = ""
        if command.captured_piece and command.capture_square:
            capture = (
                f"; capture {command.captured_piece} at {command.capture_square}"
                f" ({command.captured_model})"
            )
        promotion = f"; promote to {command.promotion}" if command.promotion else ""
        flags = []
        if command.is_castling:
            flags.append("castling")
        if command.is_en_passant:
            flags.append("en passant")
        flag_text = f"; {'; '.join(flags)}" if flags else ""

        print(
            "  [robot placeholder] "
            f"{command.moving_model}: {command.transfer.from_square}"
            f" {command.transfer.from_pose.compact()} -> "
            f"{command.transfer.to_square} {command.transfer.to_pose.compact()}"
            f"{capture}{promotion}{flag_text}"
        )
        for transfer in command.supporting_transfers:
            print(
                "  [robot placeholder] "
                f"{transfer.model_name}: {transfer.from_square}"
                f" {transfer.from_pose.compact()} -> "
                f"{transfer.to_square} {transfer.to_pose.compact()}"
                f" ({transfer.reason})"
            )


class ChessSequenceRunner:
    def __init__(
        self,
        white_engine: chess.engine.SimpleEngine,
        black_engine: chess.engine.SimpleEngine,
        white_settings: EngineSettings,
        black_settings: EngineSettings,
        robot_bridge: DryRunRobotBridge,
        max_plies: int,
        delay: float,
        print_board: bool,
    ) -> None:
        self.board = chess.Board()
        self.white_engine = white_engine
        self.black_engine = black_engine
        self.white_settings = white_settings
        self.black_settings = black_settings
        self.robot_bridge = robot_bridge
        self.max_plies = max_plies
        self.delay = delay
        self.print_board = print_board
        self.square_models = initial_square_models()
        self.moves_played: List[str] = []

    def play(self) -> None:
        print(
            "Starting Stockfish chess sequence "
            f"(white={self.white_settings.path}, black={self.black_settings.path}, "
            f"max_plies={self.max_plies})"
        )
        if self.print_board:
            print(self.board)

        ply = 0
        while ply < self.max_plies and not self.board.is_game_over(claim_draw=True):
            engine = self.white_engine if self.board.turn == chess.WHITE else self.black_engine
            settings = self.white_settings if self.board.turn == chess.WHITE else self.black_settings
            move = self.get_engine_move(engine, settings)

            command = self.build_robot_command(move, ply)
            print(self.describe_move(command))
            self.robot_bridge.execute_move(command)

            self.apply_model_move(move)
            self.board.push(move)
            self.moves_played.append(move.uci())

            if self.print_board:
                print(self.board)

            ply += 1
            if self.delay > 0.0:
                time.sleep(self.delay)

        print(f"Game result: {self.board.result(claim_draw=True)} ({self.game_reason(ply)})")
        print(f"Final FEN: {self.board.fen()}")
        print(f"Move list: {format_move_list(self.moves_played)}")

    def get_engine_move(
        self, engine: chess.engine.SimpleEngine, settings: EngineSettings
    ) -> chess.Move:
        limit = chess.engine.Limit(
            time=settings.think_time if settings.depth is None else None,
            depth=settings.depth,
        )
        result = engine.play(self.board, limit)
        if result.move is None:
            raise RuntimeError("Engine did not return a move for a non-terminal board")
        if result.move not in self.board.legal_moves:
            raise RuntimeError(f"Engine returned illegal move: {result.move.uci()}")
        return result.move

    def build_robot_command(self, move: chess.Move, ply: int) -> RobotMoveCommand:
        moving_piece = self.board.piece_at(move.from_square)
        if moving_piece is None:
            raise RuntimeError(f"No piece on {chess.square_name(move.from_square)}")

        moving_model = self.square_models.get(move.from_square)
        if moving_model is None:
            raise RuntimeError(
                f"No Gazebo model tracked on {chess.square_name(move.from_square)}"
            )

        capture_square = capture_square_for(self.board, move)
        captured_piece = (
            self.board.piece_at(capture_square) if capture_square is not None else None
        )
        captured_model = (
            self.square_models.get(capture_square) if capture_square is not None else None
        )

        transfer = PieceTransfer(
            model_name=moving_model,
            from_square=chess.square_name(move.from_square),
            to_square=chess.square_name(move.to_square),
            from_pose=square_pose(move.from_square),
            to_pose=square_pose(move.to_square),
        )

        supporting_transfers = []
        if self.board.is_castling(move):
            rook_transfer = self.castling_rook_transfer(move)
            if rook_transfer is not None:
                supporting_transfers.append(rook_transfer)

        return RobotMoveCommand(
            ply=ply + 1,
            fullmove_number=self.board.fullmove_number,
            color=color_name(self.board.turn),
            move_uci=move.uci(),
            move_san=self.board.san(move),
            moving_piece=piece_name(moving_piece),
            moving_model=moving_model,
            transfer=transfer,
            captured_piece=piece_name(captured_piece) if captured_piece else None,
            captured_model=captured_model,
            capture_square=chess.square_name(capture_square)
            if capture_square is not None
            else None,
            capture_pose=square_pose(capture_square) if capture_square is not None else None,
            promotion=chess.piece_name(move.promotion) if move.promotion else None,
            is_castling=self.board.is_castling(move),
            is_en_passant=self.board.is_en_passant(move),
            supporting_transfers=tuple(supporting_transfers),
            fen_before=self.board.fen(),
        )

    def describe_move(self, command: RobotMoveCommand) -> str:
        turn = (
            f"{command.fullmove_number}."
            if command.color == WHITE
            else f"{command.fullmove_number}..."
        )
        capture = ""
        if command.captured_piece and command.capture_square:
            capture = f" captures {command.captured_piece} on {command.capture_square}"
        promotion = f" promotes to {command.promotion}" if command.promotion else ""
        return (
            f"{turn} {command.color.capitalize()} {command.move_san} "
            f"({command.move_uci}): {command.moving_piece} "
            f"({command.moving_model}) {command.transfer.from_square}"
            f"->{command.transfer.to_square}{capture}{promotion}"
        )

    def castling_rook_transfer(self, move: chess.Move) -> Optional[PieceTransfer]:
        rook_squares = castling_rook_squares(move)
        if rook_squares is None:
            return None

        rook_from, rook_to = rook_squares
        rook_model = self.square_models.get(rook_from)
        if rook_model is None:
            return None

        return PieceTransfer(
            model_name=rook_model,
            from_square=chess.square_name(rook_from),
            to_square=chess.square_name(rook_to),
            from_pose=square_pose(rook_from),
            to_pose=square_pose(rook_to),
            reason="castling rook move",
        )

    def apply_model_move(self, move: chess.Move) -> None:
        capture_square = capture_square_for(self.board, move)
        if capture_square is not None:
            self.square_models.pop(capture_square, None)

        moving_model = self.square_models.pop(move.from_square)
        self.square_models[move.to_square] = moving_model

        rook_squares = castling_rook_squares(move)
        if rook_squares is not None:
            rook_from, rook_to = rook_squares
            rook_model = self.square_models.pop(rook_from)
            self.square_models[rook_to] = rook_model

    def game_reason(self, ply: int) -> str:
        if self.board.is_checkmate():
            return "checkmate"
        if self.board.is_stalemate():
            return "stalemate"
        if self.board.is_insufficient_material():
            return "insufficient material"
        if self.board.is_seventyfive_moves():
            return "seventy-five move rule"
        if self.board.is_fivefold_repetition():
            return "fivefold repetition"
        if self.board.can_claim_fifty_moves():
            return "fifty-move rule claim available"
        if self.board.can_claim_threefold_repetition():
            return "threefold repetition claim available"
        if ply >= self.max_plies:
            return f"stopped at max plies ({self.max_plies})"
        return "unfinished"


def capture_square_for(board: chess.Board, move: chess.Move) -> Optional[chess.Square]:
    if not board.is_capture(move):
        return None
    if board.is_en_passant(move):
        return chess.square(chess.square_file(move.to_square), chess.square_rank(move.from_square))
    return move.to_square


def castling_rook_squares(move: chess.Move) -> Optional[Tuple[chess.Square, chess.Square]]:
    rook_moves = {
        (chess.E1, chess.G1): (chess.H1, chess.F1),
        (chess.E1, chess.C1): (chess.A1, chess.D1),
        (chess.E8, chess.G8): (chess.H8, chess.F8),
        (chess.E8, chess.C8): (chess.A8, chess.D8),
    }
    return rook_moves.get((move.from_square, move.to_square))


def color_name(color: chess.Color) -> str:
    return WHITE if color == chess.WHITE else BLACK


def piece_name(piece: chess.Piece) -> str:
    return f"{color_name(piece.color)} {chess.piece_name(piece.piece_type)}"


def square_pose(square: chess.Square) -> BoardPose:
    # These constants mirror panda_description/worlds/tabletop_chess.sdf.
    spacing = 0.076
    return BoardPose(
        x=0.335 + spacing * chess.square_rank(square),
        y=-0.266 + spacing * chess.square_file(square),
        z=0.294,
    )


def initial_square_models() -> Dict[chess.Square, str]:
    models: Dict[chess.Square, str] = {}
    for rank in (1, 2, 7, 8):
        for file_index in range(8):
            square = chess.square(file_index, rank - 1)
            models[square] = f"piece{rank}{file_index + 1}"
    return models


def configure_engine(engine: chess.engine.SimpleEngine, settings: EngineSettings) -> None:
    options = {}
    if settings.skill_level is not None and "Skill Level" in engine.options:
        options["Skill Level"] = settings.skill_level
    if settings.uci_elo is not None:
        if "UCI_LimitStrength" in engine.options:
            options["UCI_LimitStrength"] = True
        if "UCI_Elo" in engine.options:
            options["UCI_Elo"] = settings.uci_elo
    if options:
        engine.configure(options)


def resolve_engine_path(path: str) -> str:
    resolved = shutil.which(path)
    if resolved:
        return resolved
    if os.path.exists(path):
        return path
    if path == "stockfish" and os.path.exists("/usr/games/stockfish"):
        return "/usr/games/stockfish"
    raise SystemExit(
        f"Could not find engine executable '{path}'. Rebuild the Docker image after "
        "this change, or install Stockfish with: apt-get update && apt-get install -y stockfish"
    )


def open_engine(settings: EngineSettings) -> chess.engine.SimpleEngine:
    engine = chess.engine.SimpleEngine.popen_uci(settings.path)
    configure_engine(engine, settings)
    return engine


def format_move_list(moves: Sequence[str]) -> str:
    if not moves:
        return "(none)"
    chunks = []
    for index in range(0, len(moves), 2):
        move_number = index // 2 + 1
        white_move = moves[index]
        black_move = moves[index + 1] if index + 1 < len(moves) else ""
        chunks.append(f"{move_number}. {white_move} {black_move}".strip())
    return " ".join(chunks)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a Stockfish-vs-Stockfish game and emit placeholder robot commands. "
            "The robot/Gazebo execution hook is intentionally dry-run for now."
        )
    )
    parser.add_argument(
        "--white-engine",
        default="stockfish",
        help="Path or executable name for the white UCI engine.",
    )
    parser.add_argument(
        "--black-engine",
        default="stockfish",
        help="Path or executable name for the black UCI engine.",
    )
    parser.add_argument(
        "--white-skill",
        type=int,
        default=6,
        help="Stockfish Skill Level option for white, when supported.",
    )
    parser.add_argument(
        "--black-skill",
        type=int,
        default=6,
        help="Stockfish Skill Level option for black, when supported.",
    )
    parser.add_argument(
        "--white-elo",
        type=int,
        default=None,
        help="Stockfish UCI_Elo option for white, when supported.",
    )
    parser.add_argument(
        "--black-elo",
        type=int,
        default=None,
        help="Stockfish UCI_Elo option for black, when supported.",
    )
    parser.add_argument(
        "--depth",
        type=int,
        default=None,
        help="Search depth for each engine move. Overrides --think-time when set.",
    )
    parser.add_argument(
        "--think-time",
        type=float,
        default=0.10,
        help="Seconds each engine can think per move when --depth is not set.",
    )
    parser.add_argument(
        "--max-plies",
        type=int,
        default=160,
        help="Maximum half-moves before stopping unfinished games.",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.0,
        help="Seconds to wait between moves.",
    )
    parser.add_argument(
        "--print-board",
        action="store_true",
        help="Print an ASCII board after each move.",
    )
    parser.add_argument(
        "--quiet-placeholder",
        action="store_true",
        help="Suppress per-move robot placeholder details.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    white_path = resolve_engine_path(args.white_engine)
    black_path = resolve_engine_path(args.black_engine)

    white_settings = EngineSettings(
        path=white_path,
        skill_level=args.white_skill,
        uci_elo=args.white_elo,
        depth=args.depth,
        think_time=args.think_time,
    )
    black_settings = EngineSettings(
        path=black_path,
        skill_level=args.black_skill,
        uci_elo=args.black_elo,
        depth=args.depth,
        think_time=args.think_time,
    )

    white_engine = open_engine(white_settings)
    black_engine = open_engine(black_settings)
    try:
        runner = ChessSequenceRunner(
            white_engine=white_engine,
            black_engine=black_engine,
            white_settings=white_settings,
            black_settings=black_settings,
            robot_bridge=DryRunRobotBridge(verbose=not args.quiet_placeholder),
            max_plies=args.max_plies,
            delay=args.delay,
            print_board=args.print_board,
        )
        runner.play()
    except KeyboardInterrupt:
        print("\nChess sequence interrupted.", file=sys.stderr)
    finally:
        white_engine.quit()
        black_engine.quit()


if __name__ == "__main__":
    main()
