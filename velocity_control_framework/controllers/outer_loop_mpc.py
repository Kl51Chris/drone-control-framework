"""Constrained linear MPC for CrazySim near-hover tracking."""

from dataclasses import dataclass, field
import math
from time import perf_counter

import numpy as np
from scipy import sparse

from velocity_control_framework.interfaces import BodyRateThrustCommand, DroneState, Reference
from .near_hover_model import NearHoverModelConfig, discrete_model, world_to_fixed_yaw


class MPCSolveError(RuntimeError):
    """The proposed command must not be sent."""


@dataclass(frozen=True, slots=True)
class OuterLoopMPCConfig:
    model: NearHoverModelConfig = field(default_factory=NearHoverModelConfig)
    horizon: int = 25
    q: tuple[float, ...] = (120, 120, 200, 12, 12, 24, 6, 6, 10)
    r: tuple[float, ...] = (2, 2, 1, 12)
    s: tuple[float, ...] = (2, 2, 1, 12)
    bounds: tuple[float, ...] = (0.40, 0.40, 0.45, 0.40, 0.40, 0.60, 0.10, 0.10, 0.30)
    max_rate: float = 0.20
    min_thrust: float = 0.38
    max_thrust: float = 0.56
    solver_time_limit_s: float = 0.012

    def __post_init__(self) -> None:
        if isinstance(self.horizon, bool) or not isinstance(self.horizon, int) or self.horizon < 1:
            raise ValueError("horizon must be a positive number of steps")
        for values, size in ((self.q, 9), (self.r, 4), (self.s, 4), (self.bounds, 9)):
            if len(values) != size or not all(math.isfinite(v) and v > 0 for v in values):
                raise ValueError(f"expected {size} finite positive configuration entries")
        if not 0 < self.max_rate <= 0.20:
            raise ValueError("MPC rate must remain within the CrazySim envelope")
        if not 0.38 <= self.min_thrust < self.model.equilibrium_thrust < self.max_thrust <= 0.56:
            raise ValueError("MPC thrust bounds must contain hover within the CrazySim envelope")
        if not 0 < self.solver_time_limit_s < self.model.sample_period_s:
            raise ValueError("solver limit must be below one controller period")


class OuterLoopMPCController:
    def __init__(self, config: OuterLoopMPCConfig) -> None:
        import osqp

        self.config = config
        ad, bd = discrete_model(config.model)
        n = config.horizon
        self._free = np.vstack([np.linalg.matrix_power(ad, j) for j in range(1, n + 1)])
        self._forced = np.zeros((9 * n, 4 * n))
        for row in range(n):
            for col in range(row + 1):
                self._forced[row * 9:(row + 1) * 9, col * 4:(col + 1) * 4] = (
                    np.linalg.matrix_power(ad, row - col) @ bd
                )
        self._q = np.tile(config.q, n)
        self._r = np.tile(config.r, n)
        self._s = np.tile(config.s, n)
        self._difference = np.eye(4 * n)
        self._difference[4:, :-4] -= np.eye(4 * (n - 1))
        self._low_u = np.tile([-config.max_rate] * 3 + [config.min_thrust - config.model.equilibrium_thrust], n)
        self._high_u = np.tile([config.max_rate] * 3 + [config.max_thrust - config.model.equilibrium_thrust], n)
        self._bounds = np.tile(config.bounds, n)
        slack = sparse.eye(9 * n, format="csc")
        forced = sparse.csc_matrix(self._forced)
        matrix = sparse.vstack((
            sparse.hstack((sparse.eye(4 * n), sparse.csc_matrix((4 * n, 9 * n)))),
            sparse.hstack((forced, -slack)),
            sparse.hstack((forced, slack)),
            sparse.hstack((sparse.csc_matrix((9 * n, 4 * n)), slack)),
        ), format="csc")
        hessian = self._forced.T @ (self._q[:, None] * self._forced) + np.diag(self._r)
        hessian += self._difference.T @ (self._s[:, None] * self._difference)
        p = sparse.block_diag((sparse.csc_matrix(hessian), 0.1 * slack), format="csc")
        self._linear = np.r_[np.zeros(4 * n), np.full(9 * n, 100.)]
        self._inf = np.full(9 * n, np.inf)
        lower, upper = self._constraint_bounds(np.zeros(9 * n))
        self._solver = osqp.OSQP()
        self._solver.setup(P=p, q=self._linear, A=matrix, l=lower, u=upper,
                           verbose=False, time_limit=config.solver_time_limit_s)
        self._rows = matrix.shape[0]
        self.reset()

    def _constraint_bounds(self, free: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return (
            np.r_[self._low_u, -self._inf, -self._bounds - free, np.zeros_like(free)],
            np.r_[self._high_u, self._bounds - free, self._inf, self._inf],
        )

    def reset(self) -> None:
        self._origin: np.ndarray | None = None
        self._yaw = 0.0
        self._solver.warm_start(x=np.zeros(len(self._linear)), y=np.zeros(self._rows))
        self.diagnostics: dict = {}
        self.prediction: dict = {}
        self._last_input = np.zeros(4)

    def update(self, state: DroneState, reference: Reference, dt: float) -> BodyRateThrustCommand:
        if self._origin is None:
            self._origin = state.position.copy()
            self._yaw = state.yaw
        rotation = world_to_fixed_yaw(self._yaw)
        x = np.r_[rotation @ (state.position - self._origin),
                  rotation @ state.velocity, state.roll, state.pitch,
                  math.remainder(state.yaw - self._yaw, 2 * math.pi)]
        goal = np.r_[rotation @ (reference.position - self._origin),
                     rotation @ reference.velocity, 0., 0.,
                     math.remainder(reference.yaw - self._yaw, 2 * math.pi)]
        free = self._free @ x
        target = np.tile(goal, self.config.horizon)
        q = self._linear.copy()
        q[:len(self._r)] = self._forced.T @ (self._q * (free - target))
        q[:4] -= self._s[:4] * self._last_input
        lower, upper = self._constraint_bounds(free)
        start = perf_counter()
        self._solver.update(q=q, l=lower, u=upper)
        result = self._solver.solve(raise_error=False)
        elapsed = perf_counter() - start
        self.diagnostics = {
            "status": result.info.status, "iterations": result.info.iter,
            "primal_residual": result.info.prim_res, "dual_residual": result.info.dual_res,
            "solve_s": result.info.solve_time, "qp_elapsed_s": elapsed,
        }
        self.prediction = {}
        if result.info.status != "solved" or elapsed > self.config.solver_time_limit_s:
            raise MPCSolveError(f"QP status={result.info.status}, elapsed={elapsed:.6f}s")
        if result.x is None or not np.all(np.isfinite(result.x)):
            raise MPCSolveError("QP returned a non-finite solution")
        inputs = result.x[:len(self._r)]
        requested = inputs[:4]
        applied = np.clip(requested, self._low_u[:4], self._high_u[:4])
        if np.max(np.abs(applied - requested)) > 0.002:
            raise MPCSolveError("QP first input violates the command bounds")
        predicted = free + self._forced @ inputs
        slack = result.x[len(self._r):]
        changes = self._difference @ inputs
        changes[:4] -= self._last_input
        self.diagnostics.update(
            tracking_cost=float(self._q @ np.square(predicted - target)),
            effort_cost=float(self._r @ np.square(inputs)),
            increment_cost=float(self._s @ np.square(changes)),
            slack_cost=float(100 * np.sum(np.maximum(slack, 0))),
            max_slack=float(np.max(np.maximum(slack, 0))),
            active_input_count=int(np.count_nonzero(
                (np.abs(inputs - self._low_u) < 0.002) | (np.abs(inputs - self._high_u) < 0.002)
            )),
            soft_boundary_count=int(np.count_nonzero(np.abs(predicted) > self._bounds)),
        )
        self.prediction = {
            "state": np.vstack((x, predicted.reshape(-1, 9))),
            "input": inputs.reshape(-1, 4),
            "slack": slack.reshape(-1, 9),
            "reference": goal,
            "origin": self._origin,
            "yaw": self._yaw,
        }
        self._last_input = applied
        return BodyRateThrustCommand(*applied[:3], applied[3] + self.config.model.equilibrium_thrust)
