#!/usr/bin/env python3
"""Shared helpers for chess move generation and robot execution."""

from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import chess
import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

from grasp_planner import GraspPlanner


WHITE = "white"
BLACK = "black"
WORLD_NAME = "tabletop_world"


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


def color_name(color: chess.Color) -> str:
    return WHITE if color == chess.WHITE else BLACK


def piece_name(piece: chess.Piece) -> str:
    return f"{color_name(piece.color)} {chess.piece_name(piece.piece_type)}"


def square_pose(square: chess.Square) -> BoardPose:
    spacing = 0.076
    return BoardPose(
        x=0.335 + spacing * chess.square_rank(square),
        y=-0.266 + spacing * chess.square_file(square),
        z=0.209,
    )


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


def initial_square_models() -> Dict[chess.Square, str]:
    models: Dict[chess.Square, str] = {}
    for rank in (1, 2, 7, 8):
        for file_index in range(8):
            square = chess.square(file_index, rank - 1)
            models[square] = f"piece{rank}{file_index + 1}"
    return models


class ChessCommandTracker:
    """Tracks board state and turns legal moves into robot commands."""

    def __init__(self) -> None:
        self.board = chess.Board()
        self.square_models = initial_square_models()
        self.moves_played: List[str] = []

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

        self.board.push(move)
        self.moves_played.append(move.uci())

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

    def game_reason(self, ply: int, max_plies: int) -> str:
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
        if ply >= max_plies:
            return f"stopped at max plies ({max_plies})"
        return "unfinished"


class DryRunRobotBridge:
    """Logs the moves without touching ROS or Gazebo."""

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


class ChessMoveExecutor(GraspPlanner):
    """Physics-driven chess piece executor."""

    TOP_DOWN_ORIENTATION = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
    PIECE_BASE_DIMS = (0.03, 0.03, 0.05)
    PIECE_COLLISION_DIMS = (0.035, 0.035, 0.06)
    GRIPPER_CLOSED = [0.0040, 0.0040]
    PRE_GRASP_LIFT = 0.12
    TRANSIT_Z = 0.42
    RETREAT_LIFT = 0.12
    MICRO_LIFT = 0.03
    GRASP_Z_OFFSET = 0.004
    PLACE_Z_OFFSET = 0.018
    APPROACH_TOLERANCE = 0.03
    GRASP_TOLERANCE = 0.01
    GRASP_FAIL_GAP = 0.012
    GRASP_SETTLE_SEC = 1.0

    def __init__(self) -> None:
        super().__init__()
        self.discard_count = 0
        self.square_models: Dict[chess.Square, str] = {}

    def wait_until_ready(self) -> None:
        self.get_logger().info("Waiting for /joint_states ...")
        while self.latest_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info("Joint states ready")

    def initialize_board_scene(self, square_models: Dict[chess.Square, str]) -> None:
        self.square_models = dict(square_models)
        self._publish_board_collision_scene()
        self.get_logger().info(
            f"Initialized planning scene with {len(self.square_models)} chess piece obstacles"
        )

    def execute_move(self, command: RobotMoveCommand) -> None:
        if command.promotion:
            raise NotImplementedError("Promotion is not implemented in v1")
        if command.is_en_passant:
            raise NotImplementedError("En passant is not implemented in v1")

        self.pause_pub.publish(self._bool_msg(True))
        self._manipulating = True
        try:
            if not self.move_arm_to_joints(self.INITIAL_JOINTS):
                raise RuntimeError("Failed to move to the initial joint configuration")

            if command.captured_model and command.capture_pose:
                self.get_logger().info(
                    f'Discarding captured piece "{command.captured_model}" from '
                    f'{command.capture_square}'
                )
                self.pick_piece(command.captured_model, command.capture_pose)
                self._remove_model_from_board(command.captured_model)
                self.discard_piece(command.captured_model)
                self._publish_board_collision_scene()

            self.get_logger().info(
                f'Moving "{command.moving_model}" from '
                f'{command.transfer.from_square} to {command.transfer.to_square}'
            )
            self.pick_piece(command.moving_model, command.transfer.from_pose)
            self._remove_model_from_board(command.moving_model)
            self.place_piece(command.moving_model, command.transfer.to_pose)
            self._place_model_on_square(command.moving_model, command.transfer.to_square)
            self._publish_board_collision_scene()

            for transfer in command.supporting_transfers:
                self.get_logger().info(
                    f'Moving supporting piece "{transfer.model_name}" '
                    f'for {transfer.reason}'
                )
                self.pick_piece(transfer.model_name, transfer.from_pose)
                self._remove_model_from_board(transfer.model_name)
                self.place_piece(transfer.model_name, transfer.to_pose)
                self._place_model_on_square(transfer.model_name, transfer.to_square)
                self._publish_board_collision_scene()

            if not self.move_arm_to_joints(self.INITIAL_JOINTS):
                raise RuntimeError("Failed to return to the initial joint configuration")
        finally:
            self._manipulating = False
            self.pause_pub.publish(self._bool_msg(False))

    def pick_piece(self, model_name: str, source_pose: BoardPose) -> None:
        self._publish_board_collision_scene(active_model=model_name)
        if not self.move_gripper(open=True):
            raise RuntimeError(f"Failed to open gripper for {model_name}")

        pre_grasp = self._board_pose_to_pose(source_pose, z_offset=self.PRE_GRASP_LIFT)
        self.publish_pose_axes(pre_grasp, f"{model_name}_pre_grasp")
        if not self.move_arm_to_pose(
            pre_grasp,
            position_tolerance=self.APPROACH_TOLERANCE,
            orientation_tolerance=0.15,
        ):
            raise RuntimeError(f"Failed pre-grasp approach for {model_name}")

        grasp_pose = self._board_pose_to_pose(source_pose, z_offset=self.GRASP_Z_OFFSET)
        self.publish_pose_axes(grasp_pose, f"{model_name}_grasp")
        if not self.move_arm_to_pose(
            grasp_pose,
            position_tolerance=self.GRASP_TOLERANCE,
            orientation_tolerance=0.12,
        ):
            raise RuntimeError(f"Failed grasp descent for {model_name}")

        if not self.move_gripper(open=False):
            raise RuntimeError(f"Failed to close gripper on {model_name}")

        rclpy.spin_once(self, timeout_sec=self.GRASP_SETTLE_SEC)
        gap = self.current_gripper_gap()
        if gap < self.GRASP_FAIL_GAP:
            self.move_gripper(open=True)
            raise RuntimeError(
                f"Physical grasp failed for {model_name}: gripper closed fully (gap={gap:.4f})"
            )

        micro_lift = copy.deepcopy(grasp_pose)
        micro_lift.position.z += self.MICRO_LIFT
        retreat = copy.deepcopy(pre_grasp)
        transit = self._transit_pose_for_xy(source_pose.x, source_pose.y)
        self._carry_with_reclamp(
            model_name,
            [micro_lift, retreat, transit],
            f"after grasping {model_name}",
        )

    def place_piece(self, model_name: str, target_pose: BoardPose) -> None:
        self._publish_board_collision_scene(active_model=model_name)
        source_transit = self._transit_pose_for_xy(target_pose.x, target_pose.y)
        if not self.move_arm_to_pose(
            source_transit,
            position_tolerance=self.APPROACH_TOLERANCE,
            orientation_tolerance=0.15,
        ):
            raise RuntimeError(f"Failed source transit lift for {model_name}")

        pre_place = self._board_pose_to_pose(target_pose, z=self.TRANSIT_Z)
        self.publish_pose_axes(pre_place, f"{model_name}_pre_place")
        if not self.move_arm_to_pose(
            pre_place,
            position_tolerance=self.APPROACH_TOLERANCE,
            orientation_tolerance=0.15,
        ):
            raise RuntimeError(f"Failed pre-place approach for {model_name}")

        place_pose = self._board_pose_to_pose(target_pose, z_offset=self.PLACE_Z_OFFSET)
        if not self.move_arm_to_pose(
            place_pose,
            position_tolerance=self.GRASP_TOLERANCE,
            orientation_tolerance=0.12,
        ):
            raise RuntimeError(f"Failed placement descent for {model_name}")

        if not self.move_gripper(open=True):
            raise RuntimeError(f"Failed to release {model_name}")

        rclpy.spin_once(self, timeout_sec=0.9)

        retreat = copy.deepcopy(place_pose)
        retreat.position.z += self.RETREAT_LIFT
        if not self.move_arm_to_pose(
            retreat,
            position_tolerance=self.APPROACH_TOLERANCE,
            orientation_tolerance=0.15,
        ):
            raise RuntimeError(f"Failed retreat after placing {model_name}")

    def discard_piece(self, model_name: str) -> None:
        discard_pose = self._discard_pose(self.discard_count)
        self.discard_count += 1
        self.drop_piece_off_table(model_name, discard_pose)

    def drop_piece_off_table(self, model_name: str, discard_pose: BoardPose) -> None:
        source_transit = self._transit_pose_for_xy(discard_pose.x, discard_pose.y)
        if not self.move_arm_to_pose(
            source_transit,
            position_tolerance=self.APPROACH_TOLERANCE,
            orientation_tolerance=0.15,
        ):
            raise RuntimeError(f"Failed discard source transit for {model_name}")

        drop_pose = self._board_pose_to_pose(discard_pose, z=self.TRANSIT_Z - 0.10)
        if not self.move_arm_to_pose(
            drop_pose,
            position_tolerance=self.APPROACH_TOLERANCE,
            orientation_tolerance=0.15,
        ):
            raise RuntimeError(f"Failed discard approach for {model_name}")

        if not self.move_gripper(open=True):
            raise RuntimeError(f"Failed to release captured piece {model_name}")

        rclpy.spin_once(self, timeout_sec=0.9)

        retreat = copy.deepcopy(drop_pose)
        retreat.position.z = self.TRANSIT_Z
        if not self.move_arm_to_pose(
            retreat,
            position_tolerance=self.APPROACH_TOLERANCE,
            orientation_tolerance=0.15,
        ):
            raise RuntimeError(f"Failed discard retreat for {model_name}")

    def destroy_node(self) -> bool:
        self.move_gripper(open=True)
        return super().destroy_node()

    def _discard_pose(self, index: int) -> BoardPose:
        return BoardPose(
            x=0.42 + 0.03 * (index % 4),
            y=0.62,
            z=0.16,
        )

    def _board_pose_to_pose(
        self,
        board_pose: BoardPose,
        z_offset: float = 0.0,
        z: Optional[float] = None,
    ) -> Pose:
        pose = Pose()
        pose.position = Point(
            x=board_pose.x,
            y=board_pose.y,
            z=board_pose.z + z_offset if z is None else z,
        )
        pose.orientation = self.TOP_DOWN_ORIENTATION
        return pose

    def _piece_collision_object(self, square: chess.Square, model_name: str) -> CollisionObject:
        board_pose = square_pose(square)
        collision_height = self.PIECE_COLLISION_DIMS[2]
        base_height = self.PIECE_BASE_DIMS[2]

        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.id = model_name
        obj.operation = CollisionObject.ADD
        obj.primitives.append(
            SolidPrimitive(
                type=SolidPrimitive.BOX,
                dimensions=list(self.PIECE_COLLISION_DIMS),
            )
        )

        pose = Pose()
        pose.position = Point(
            x=board_pose.x,
            y=board_pose.y,
            z=board_pose.z + max(0.0, collision_height - base_height) / 2.0,
        )
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        obj.primitive_poses.append(pose)
        return obj

    def _publish_board_collision_scene(self, active_model: Optional[str] = None) -> None:
        occupied_models = set(self.square_models.values())
        scene = PlanningScene(is_diff=True)

        for model_name in initial_square_models().values():
            if model_name in occupied_models and model_name != active_model:
                continue
            remove = CollisionObject()
            remove.header.frame_id = "world"
            remove.header.stamp = self.get_clock().now().to_msg()
            remove.id = model_name
            remove.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(remove)

        for square, model_name in self.square_models.items():
            if model_name == active_model:
                continue
            scene.world.collision_objects.append(
                self._piece_collision_object(square, model_name)
            )

        self.scene_pub.publish(scene)
        obstacle_count = len(self.square_models) - (1 if active_model in occupied_models else 0)
        if active_model:
            self.get_logger().info(
                f'Published {obstacle_count} chess piece obstacles; excluding "{active_model}"'
            )

    def _remove_model_from_board(self, model_name: str) -> None:
        for square, square_model in list(self.square_models.items()):
            if square_model == model_name:
                self.square_models.pop(square)
                return
        self.get_logger().warning(
            f'Could not remove "{model_name}" from board occupancy; model was not tracked'
        )

    def _place_model_on_square(self, model_name: str, square_name: str) -> None:
        self.square_models[chess.parse_square(square_name)] = model_name

    def _transit_pose_for_xy(self, x: float, y: float) -> Pose:
        pose = Pose()
        pose.position = Point(x=x, y=y, z=self.TRANSIT_Z)
        pose.orientation = self.TOP_DOWN_ORIENTATION
        return pose

    def _carry_with_reclamp(
        self,
        model_name: str,
        waypoints: Sequence[Pose],
        context: str,
    ) -> None:
        for index, waypoint in enumerate(waypoints):
            if not self.move_arm_to_pose(
                waypoint,
                position_tolerance=self.APPROACH_TOLERANCE,
                orientation_tolerance=0.15,
            ):
                self.move_gripper(open=True)
                raise RuntimeError(
                    f"Failed carry waypoint {index + 1} {context}"
                )

            # Re-issue the close command after each lift / carry segment so the
            # effort controller keeps squeezing while the arm is in motion.
            if not self.move_gripper(open=False):
                self.move_gripper(open=True)
                raise RuntimeError(
                    f"Failed to re-clamp gripper at carry waypoint {index + 1} {context}"
                )

            rclpy.spin_once(self, timeout_sec=0.3)
            gap = self.current_gripper_gap()
            if gap < self.GRASP_FAIL_GAP:
                self.move_gripper(open=True)
                raise RuntimeError(
                    f"Lost grasp at carry waypoint {index + 1} {context} "
                    f"(gap={gap:.4f})"
                )

    def current_gripper_gap(self) -> float:
        if self.latest_joint_state is None:
            raise RuntimeError("No joint state available for gripper gap check")

        positions = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        try:
            return float(
                positions[self.GRIPPER_JOINTS[0]] + positions[self.GRIPPER_JOINTS[1]]
            )
        except KeyError as exc:
            raise RuntimeError("Gripper joints missing from latest joint state") from exc

    @staticmethod
    def _bool_msg(value: bool):
        from std_msgs.msg import Bool

        return Bool(data=value)


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
