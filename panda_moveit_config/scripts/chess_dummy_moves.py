#!/usr/bin/env python3
"""Execute canned or user-supplied chess moves without the engines."""

from __future__ import annotations

import argparse
import sys
import time

try:
    import chess
except ImportError as exc:
    raise SystemExit(
        "Missing python-chess. Rebuild the Docker image after this change, or install "
        "the dependency with: python3 -m pip install chess"
    ) from exc

import rclpy

from chess_robot_common import ChessCommandTracker, ChessMoveExecutor, format_move_list


SCENARIOS = {
    #"opening": ["e2e4", "e7e5", "g1f3", "b8c6"],
    "opening": ["e2e4", "a2a3", "d2d5"],
    "capture": ["e2e4", "d7d5", "e4d5"],
    "castle": ["e2e4", "e7e5", "g1f3", "b8c6", "f1c4", "g8f6", "e1g1"],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Execute a short predefined or explicit sequence of chess moves."
    )
    parser.add_argument(
        "--scenario",
        choices=sorted(SCENARIOS),
        default="opening",
        help="Built-in move sequence to execute when --moves is not provided.",
    )
    parser.add_argument(
        "--moves",
        nargs="*",
        help="Explicit list of UCI moves to execute, for example: e2e4 e7e5 g1f3",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.0,
        help="Seconds to wait between moves.",
    )
    return parser.parse_args()


def resolve_moves(args: argparse.Namespace) -> list[str]:
    if args.moves:
        return list(args.moves)
    return list(SCENARIOS[args.scenario])


def main() -> None:
    args = parse_args()
    moves = resolve_moves(args)

    rclpy.init()
    executor = ChessMoveExecutor()
    tracker = ChessCommandTracker()

    try:
        executor.wait_until_ready()
        executor.initialize_board_scene(tracker.square_models)

        print(f"Executing dummy move sequence: {format_move_list(moves)}")
        for ply, move_uci in enumerate(moves):
            move = chess.Move.from_uci(move_uci)
            if move not in tracker.board.legal_moves:
                raise RuntimeError(f"Illegal move at step {ply + 1}: {move_uci}")

            command = tracker.build_robot_command(move, ply)
            print(tracker.describe_move(command))
            executor.execute_move(command)
            tracker.apply_model_move(move)

            if args.delay > 0.0:
                time.sleep(args.delay)
    except KeyboardInterrupt:
        print("\nDummy chess sequence interrupted.", file=sys.stderr)
    finally:
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
