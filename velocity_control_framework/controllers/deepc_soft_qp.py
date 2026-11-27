"""Deadline-bounded primal-dual solve for the full DeePC soft-box QP."""
from time import perf_counter

import numpy as np
from scipy import linalg


def solve(h, f, low, high, bounds, free, linear, root, deadline):
    """Return a certified solution, iteration count, and residuals.

    The ordering is upper/lower input, upper/lower output, then nonnegative
    slack. Eliminating the diagonal slack block keeps each Newton system at
    the size of the original input and latent variables.
    """
    m, d = f.shape
    p = len(low)
    z = root.copy()
    z[:p] = np.clip(z[:p], low + .01*(high-low), high - .01*(high-low))
    y = f@z+free
    s = np.maximum(np.abs(y)-bounds,0.)+.005
    x = np.concatenate((z,s))
    b = np.concatenate((high,-low,bounds-free,bounds+free,np.zeros(m)))
    e1, e2, e3, e4 = p, 2*p, 2*p+m, 2*p+2*m
    hard_diag = np.arange(p)
    q = np.concatenate((linear,np.full(m,100.)))
    def ax(z,s):
        y = f@z
        return np.concatenate((z[:p],-z[:p],y-s,-y-s,-s))
    def at(v):
        yh, yl = v[e2:e3], v[e3:e4]
        rz = f.T@(yh-yl)
        rz[:p] += v[:e1]-v[e1:e2]
        return np.concatenate((rz,-yh-yl-v[e4:]))
    w = np.maximum(b-ax(z,s),.001)
    lam = np.ones_like(w)
    def boundary(value,direction):
        negative = direction<0.
        return min(1.,float(np.min(-value[negative]/direction[negative],initial=np.inf)))
    for i in range(24):
        if perf_counter()>=deadline:
            return None, i, None
        z,s = x[:d],x[d:]
        rp = ax(z,s)+w-b
        rd = np.concatenate((h@z,.1*s))+q+at(lam)
        mu = w@lam/len(w)
        primal = np.max(np.abs(rp))
        dual = np.max(np.abs(rd))
        if primal < 2e-7 and dual < 2e-5 and mu < 5e-8:
            return x, i, (float(primal), float(dual), float(mu))
        ratios = lam/w
        dh, dl = ratios[:e1], ratios[e1:e2]
        uh, ul = ratios[e2:e3], ratios[e3:e4]
        reg = .1+ratios[e4:]
        ab = uh+ul+reg
        coupling = ul-uh
        effective = (4*uh*ul+(uh+ul)*reg)/ab
        hh = h+f.T@(effective[:,None]*f)
        hh[hard_diag,hard_diag] += dh+dl
        try:
            fact = linalg.cho_factor(hh,check_finite=False)
        except linalg.LinAlgError:
            return None, i, None
        def delta(rc):
            rhs = -rd+at((rc-lam*rp)/w)
            dz = linalg.cho_solve(fact,rhs[:d]-f.T@(coupling*rhs[d:]/ab),check_finite=False)
            ds = (rhs[d:]-coupling*(f@dz))/ab
            dw = -rp-ax(dz,ds)
            dv = (-rc-lam*dw)/w
            return np.concatenate((dz,ds)),dw,dv
        affine = delta(w*lam)
        a = boundary(w,affine[1])
        bstep = boundary(lam,affine[2])
        ratio = ((w+a*affine[1])@(lam+bstep*affine[2]))/(len(w)*mu)
        dx,dw,dv = delta(w*lam+affine[1]*affine[2]-mu*np.clip(ratio,0.,1.)**3)
        a = min(1.,.995*boundary(w,dw))
        bstep = min(1.,.995*boundary(lam,dv))
        x += a*dx
        w += a*dw
        lam += bstep*dv
        if not np.all(np.isfinite(x)) or not np.all(np.isfinite(lam)) or not np.all(np.isfinite(w)):
            return None, i, None
    return None, 24, None
