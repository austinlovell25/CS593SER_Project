#!/usr/bin/env python3
"""Run a Stockfish-vs-Stockfish chess sequence."""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import time
from dataclasses import dataclass
from typing import Optional

try:
    import chess
    import chess.engine
except ImportError as exc:
    raise SystemExit(
        "Missing python-chess. Rebuild the Docker image after this change, or install "
        "the dependency with: python3 -m pip install chess"
    ) from exc

from chess_robot_common import (
    ChessCommandTracker,
    ChessMoveExecutor,
    DualArmChessRobotBridge,
    DryRunRobotBridge,
    format_move_list,
)


@dataclass(frozen=True)
class EngineSettings:
    path: str
    skill_level: Optional[int]
    uci_elo: Optional[int]
    depth: Optional[int]
    think_time: float


class ChessSequenceRunner:
    def __init__(
        self,
        white_engine: chess.engine.SimpleEngine,
        black_engine: chess.engine.SimpleEngine,
        white_settings: EngineSettings,
        black_settings: EngineSettings,
        robot_bridge,
        max_plies: int,
        delay: float,
        print_board: bool,
    ) -> None:
        self.tracker = ChessCommandTracker()
        self.white_engine = white_engine
        self.black_engine = black_engine
        self.white_settings = white_settings
        self.black_settings = black_settings
        self.robot_bridge = robot_bridge
        self.max_plies = max_plies
        self.delay = delay
        self.print_board = print_board

    def play(self) -> None:
        print(
            "Starting Stockfish chess sequence "
            f"(white={self.white_settings.path}, black={self.black_settings.path}, "
            f"max_plies={self.max_plies})"
        )
        if self.print_board:
            print(self.tracker.board)

        ply = 0
        while ply < self.max_plies and not self.tracker.board.is_game_over(claim_draw=True):
            engine = (
                self.white_engine if self.tracker.board.turn == chess.WHITE else self.black_engine
            )
            settings = (
                self.white_settings
                if self.tracker.board.turn == chess.WHITE
                else self.black_settings
            )
            move = self.get_engine_move(engine, settings)

            command = self.tracker.build_robot_command(move, ply)
            print(self.tracker.describe_move(command))
            self.robot_bridge.execute_move(command)
            self.tracker.apply_model_move(move)
            if hasattr(self.robot_bridge, "sync_board_scene"):
                self.robot_bridge.sync_board_scene(self.tracker.square_models)

            if self.print_board:
                print(self.tracker.board)

            ply += 1
            if self.delay > 0.0:
                time.sleep(self.delay)

        print(
            f"Game result: {self.tracker.board.result(claim_draw=True)} "
            f"({self.tracker.game_reason(ply, self.max_plies)})"
        )
        print(f"Final FEN: {self.tracker.board.fen()}")
        print(f"Move list: {format_move_list(self.tracker.moves_played)}")

    def get_engine_move(
        self, engine: chess.engine.SimpleEngine, settings: EngineSettings
    ) -> chess.Move:
        limit = chess.engine.Limit(
            time=settings.think_time if settings.depth is None else None,
            depth=settings.depth,
        )
        result = engine.play(self.tracker.board, limit)
        if result.move is None:
            raise RuntimeError("Engine did not return a move for a non-terminal board")
        if result.move not in self.tracker.board.legal_moves:
            raise RuntimeError(f"Engine returned illegal move: {result.move.uci()}")
        return result.move


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a Stockfish-vs-Stockfish game and execute the resulting chess moves."
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
        "--dry-run",
        action="store_true",
        help="Do not move the robot; print the planned chess-piece transfers only.",
    )
    parser.add_argument(
        "--single-arm",
        action="store_true",
        help="Use the original single Panda arm for both white and black moves.",
    )
    parser.add_argument(
        "--quiet-placeholder",
        action="store_true",
        help="Suppress per-move details when using --dry-run.",
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
    robot_bridge = None
    try:
        if args.dry_run:
            robot_bridge = DryRunRobotBridge(verbose=not args.quiet_placeholder)
        else:
            import rclpy

            rclpy.init()
            robot_bridge = ChessMoveExecutor() if args.single_arm else DualArmChessRobotBridge()
            robot_bridge.wait_until_ready()
            robot_bridge.initialize_board_scene(ChessCommandTracker().square_models)

        runner = ChessSequenceRunner(
            white_engine=white_engine,
            black_engine=black_engine,
            white_settings=white_settings,
            black_settings=black_settings,
            robot_bridge=robot_bridge,
            max_plies=args.max_plies,
            delay=args.delay,
            print_board=args.print_board,
        )
        runner.play()
    except KeyboardInterrupt:
        print("\nChess sequence interrupted.", file=sys.stderr)
    finally:
        if robot_bridge is not None and hasattr(robot_bridge, "destroy_node"):
            robot_bridge.destroy_node()
            import rclpy

            rclpy.shutdown()
        white_engine.quit()
        black_engine.quit()


if __name__ == "__main__":
    main()
