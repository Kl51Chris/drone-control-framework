"""Reduced-subspace regularized DeePC in a fixed-yaw, window-local frame."""

from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy import sparse, linalg
from types import SimpleNamespace

from .near_hover_model import world_to_fixed_yaw
from .outer_loop_mpc import MPCSolveError, OuterLoopMPCConfig, OuterLoopMPCController
from ..interfaces import BodyRateThrustCommand, Reference
from ..backends.crazysim_command_adapter import CrazySimCommandAdapter


@dataclass(frozen=True, slots=True)
class OuterLoopDeePCConfig(OuterLoopMPCConfig):
    dataset: str = "log/phase3-v1"
    past: int = 6
    latent: int = 16
    regularization: float = 1e-3


def training_blocks(config):
    root = Path(config.dataset)
    manifest = json.loads((root / "manifest.json").read_text())
    if manifest["version"] != "phase3-v1" or manifest["sample_period_s"] != config.model.sample_period_s:
        raise ValueError("unexpected DeePC dataset version or sample period")
    if not math.isclose(manifest["thrust_equilibrium"], config.model.equilibrium_thrust):
        raise ValueError("DeePC thrust equilibrium differs from dataset")
    t, n = config.past, config.horizon
    if t != 6 or n != 25 or config.latent < 1 or config.regularization <= 0:
        raise ValueError("DeePC prototype is scoped to 6 past and 25 future steps")
    all_u, all_y = [], []
    for item in manifest["episodes"]:
        if item["split"] != "train":
            continue
        with np.load(root / item["processed"]) as record:
            u,y = record["u"],record["y"]
            for k in range(10,len(y)-n):
                all_u.append(u[k-t:k+n])
                output = y[k-t+1:k+n+1].copy()
                output[:,:3] -= y[k,:3]
                all_y.append(output)
    return np.asarray(all_u),np.asarray(all_y)


class OuterLoopDeePCController(OuterLoopMPCController):
    """Inputs are exact; only the output-side nullspace is reduced."""

    def __init__(self, config: OuterLoopDeePCConfig) -> None:
        import osqp

        self.config = config
        t,n = config.past,config.horizon
        u,y = training_blocks(config)
        self._us = np.maximum(u.reshape(-1,4).std(axis=0),1e-5)
        self._ys = np.maximum(y.reshape(-1,9).std(axis=0),1e-4)
        h = (u/self._us).reshape(len(u),-1).T
        v = (y/self._ys).reshape(len(y),-1).T
        a,s,b = np.linalg.svd(h,full_matrices=False)
        if s[-1] < 1e-4*s[0]:
            raise ValueError("poorly conditioned input equality")
        lift = (b.T/s)@a.T
        null = v-(v@b.T)@b
        _,sn,vn = np.linalg.svd(null,full_matrices=False)
        rank = min(config.latent, int(np.count_nonzero(sn>sn[0]*1e-4)))
        directions = vn[:rank].T
        up = 4*t
        self._use_y = np.ones(9*t,dtype=bool)
        self._use_y[-9:-6] = False
        lift_y = v@lift
        latent_y = v@directions
        self._past_map = np.hstack((lift_y[:9*t,up:]/np.tile(self._us,n),latent_y[:9*t]))[self._use_y]
        self._past_offset = lift_y[:9*t,:up][self._use_y]
        future_scale = np.tile(self._ys,n)
        self._forced = future_scale[:,None]*np.hstack((lift_y[9*t:,up:]/np.tile(self._us,n),latent_y[9*t:]))
        self._future_offset = future_scale[:,None]*lift_y[9*t:,:up]
        self._lift_past = lift[:,:up]
        self._lift_free = np.hstack((lift[:,up:]/np.tile(self._us,n),directions))
        self._lambda = (config.regularization*s[0])**2
        self._q = np.tile(config.q,n)
        self._r = np.tile(config.r,n)
        self._s = np.tile(config.s,n)
        self._difference = np.eye(4*n)
        self._difference[4:,:-4] -= np.eye(4*(n-1))
        self._n_input = 4*n
        self._n_decision = 4*n+rank
        self._low_u = np.tile([-config.max_rate]*3+[config.min_thrust-config.model.equilibrium_thrust],n)
        self._high_u = np.tile([config.max_rate]*3+[config.max_thrust-config.model.equilibrium_thrust],n)
        self._bounds = np.tile(config.bounds,n)
        self._inf = np.full(9*n,np.inf)
        slack = sparse.eye(9*n,format="csc")
        forced = sparse.csc_matrix(self._forced)
        select = sparse.eye(self._n_input,self._n_decision,format="csc")
        matrix = sparse.vstack((
            sparse.hstack((select,sparse.csc_matrix((4*n,9*n)))),
            sparse.hstack((forced,-slack)), sparse.hstack((forced,slack)),
            sparse.hstack((sparse.csc_matrix((9*n,self._n_decision)),slack)),
        ),format="csc")
        hessian = (self._forced.T@(self._q[:,None]*self._forced) +
                   self._past_map.T@self._past_map + self._lambda*self._lift_free.T@self._lift_free)
        hessian[:4*n,:4*n] += np.diag(self._r)+self._difference.T@(self._s[:,None]*self._difference)
        p = sparse.block_diag((sparse.csc_matrix(hessian),0.1*slack),format="csc")
        self._factor = linalg.cho_factor(hessian)
        self._box_solver = osqp.OSQP()
        self._box_solver.setup(P=sparse.csc_matrix(hessian),q=np.zeros(self._n_decision),
            A=select,l=self._low_u,u=self._high_u,verbose=False,eps_abs=1e-5,eps_rel=1e-5,
            time_limit=config.solver_time_limit_s)
        self._box_solver.solve(raise_error=False)
        self._linear = np.r_[np.zeros(self._n_decision),np.full(9*n,100.)]
        self._solver = osqp.OSQP()
        lo,hi = self._constraint_bounds(np.zeros(9*n))
        self._solver.setup(P=p,q=self._linear,A=matrix,l=lo,u=hi,verbose=False,
                           time_limit=config.solver_time_limit_s)
        # OSQP includes setup in its first time-limit accounting. Run a zero
        # setup probe on the ground and never use its command for flight.
        self._solver.solve(raise_error=False)
        self._rows = matrix.shape[0]
        self.reset()

    def reset(self):
        super().reset()
        self._box_solver.warm_start(x=np.zeros(self._n_decision),y=np.zeros(self._n_input))
        self._observations = deque(maxlen=self.config.past+1)
        self._issued = deque(maxlen=self.config.past)
        self._bridge = OuterLoopMPCController(self.config) if not hasattr(self,"_bridge") else self._bridge
        self._bridge.reset()
        self._previous_observation = None

    def record_issued_command(self, command):
        super().record_issued_command(command)
        self._bridge.record_issued_command(command)
        actual = self._last_input.copy()
        adapter = CrazySimCommandAdapter
        actual[3] = ((adapter._encode_thrust(command.thrust)-adapter.RAW_THRUST_MIN) /
                     (adapter.RAW_THRUST_MAX-adapter.RAW_THRUST_MIN)-self.config.model.equilibrium_thrust)
        self._last_input = actual
        self._issued.append(actual)

    def _update(self,state,references):
        if self._previous_observation is not None:
            dt=state.timestamp-self._previous_observation
            if not 0 < dt <= .04:
                raise MPCSolveError("DeePC history lost observation continuity")
        self._previous_observation=state.timestamp
        if self._origin is None:
            self._origin=state.position.copy()
            self._yaw=state.yaw
        rotation=world_to_fixed_yaw(self._yaw)
        x=np.r_[rotation@(state.position-self._origin),rotation@state.velocity,state.roll,state.pitch,
                math.remainder(state.yaw-self._yaw,2*math.pi)]
        self._observations.append(x)
        t,n=self.config.past,self.config.horizon
        if len(self._issued)<t or len(self._observations)<t+1:
            command=self._bridge.update(state,references[0],self.config.model.sample_period_s)
            self.diagnostics={**self._bridge.diagnostics,"active_controller":"mpc_history_bridge",
                              "history_samples":len(self._issued)}
            self.prediction={}
            return command
        shift=np.r_[x[:3],np.zeros(6)]
        past=(np.asarray(self._observations)[1:]-shift)/self._ys
        past=past.ravel()[self._use_y]
        u_past=(np.asarray(self._issued)/self._us).ravel()
        target=np.asarray([np.r_[rotation@(ref.position-state.position),rotation@ref.velocity,0.,0.,
                           x[8]+math.remainder(ref.yaw-self._yaw-x[8],2*math.pi)] for ref in references])
        free=self._future_offset@u_past
        # Translate the original fixed takeover box into the moving frame.
        lo,hi=self._constraint_bounds(free+np.tile(shift,n))
        past_residual=self._past_offset@u_past-past
        g0=self._lift_past@u_past
        linear=self._linear.copy()
        linear[:self._n_decision] = (self._forced.T@(self._q*(free-target.ravel()))+
                                    self._past_map.T@past_residual+self._lambda*self._lift_free.T@g0)
        linear[:4] -= self._s[:4]*self._last_input
        start=perf_counter()
        candidate=linalg.cho_solve(self._factor,-linear[:self._n_decision])
        requested=candidate[:self._n_input]
        result=None
        if np.any(requested<self._low_u) or np.any(requested>self._high_u):
            self._box_solver.update(q=linear[:self._n_decision])
            box=self._box_solver.solve(raise_error=False)
            if box.info.status=="solved":
                candidate=box.x
                result=SimpleNamespace(x=np.r_[candidate,np.zeros(9*n)],info=box.info)
        else:
            result=SimpleNamespace(x=np.r_[candidate,np.zeros(9*n)],info=SimpleNamespace(
                status="solved",iter=0,prim_res=0.,dual_res=0.,solve_time=0.))
        if result is None or np.any(abs(self._forced@candidate+free+np.tile(shift,n))>self._bounds):
            self._solver.update(q=linear,l=lo,u=hi)
            result=self._solver.solve(raise_error=False)
        elapsed=perf_counter()-start
        self.diagnostics={"status":result.info.status,"iterations":result.info.iter,
            "primal_residual":result.info.prim_res,"dual_residual":result.info.dual_res,
            "solve_s":result.info.solve_time,"qp_elapsed_s":elapsed,"active_controller":"deepc",
            "history_samples":len(self._issued)}
        self.prediction={}
        if result.info.status!="solved" or elapsed>self.config.solver_time_limit_s or result.x is None or not np.all(np.isfinite(result.x)):
            raise MPCSolveError(f"DeePC QP status={result.info.status}, elapsed={elapsed:.6f}s")
        z=result.x[:self._n_decision]
        u=z[:self._n_input]
        if np.max(np.maximum(self._low_u-u,u-self._high_u))>.002:
            raise MPCSolveError("DeePC input bounds violation")
        slack=np.maximum(result.x[self._n_decision:],0.)
        g=g0+self._lift_free@z
        y=(free+self._forced@z).reshape(n,9)
        residual=self._past_map@z+past_residual
        changes=self._difference@u
        changes[:4]-=self._last_input
        self.diagnostics.update(init_slack_rms=float(np.sqrt(np.mean(residual**2))),
            g_norm=float(np.linalg.norm(g)),regularization_cost=float(.5*self._lambda*(g@g)),
            tracking_cost=float(self._q@np.square((y-target).ravel())),
            effort_cost=float(self._r@np.square(u)),increment_cost=float(self._s@np.square(changes)),
            max_slack=float(np.max(slack)),slack_cost=float(100*np.sum(slack)),
            active_input_count=int(np.count_nonzero((abs(u-self._low_u)<.002)|(abs(u-self._high_u)<.002))),
            soft_boundary_count=int(np.count_nonzero(abs((y+shift).ravel())>self._bounds)))
        self.prediction={"state":np.vstack((x-shift,y)),"input":u.reshape(n,4),
            "slack":slack.reshape(n,9),"reference":target[0],"reference_trajectory":target,
            "origin":state.position.copy(),"yaw":self._yaw}
        command=np.clip(u[:4],self._low_u[:4],self._high_u[:4])
        return BodyRateThrustCommand(*command[:3],command[3]+self.config.model.equilibrium_thrust)
