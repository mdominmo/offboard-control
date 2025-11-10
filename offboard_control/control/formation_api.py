import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp
from mpl_toolkits.mplot3d import Axes3D  # Needed for 3D plotting
import time


# ----------------------- #
#    OPTIMIZATION FUNCTIONS
# ----------------------- #
def approach_trajectory_3d_optimal(
    N_app, dt, V,
    x_in, y_in, z_in, theta_in, phi_in,
    x_final, y_final, z_final, theta_final, phi_final,
    u_theta_max, u_phi_max
):
    # weights for slack penalties
    w_pos = 1e4
    w_ang = 1e4

    # time grid and nominal angle interpolation
    tvec     = np.linspace(0, 1, N_app + 1)
    theta_nom = (1 - tvec) * theta_in + tvec * theta_final
    phi_nom   = (1 - tvec) * phi_in   + tvec * phi_final

    # decision variables
    x       = cp.Variable(N_app + 1)
    y       = cp.Variable(N_app + 1)
    z       = cp.Variable(N_app + 1)
    theta   = cp.Variable(N_app + 1)
    phi     = cp.Variable(N_app + 1)
    u_theta = cp.Variable(N_app)
    u_phi   = cp.Variable(N_app)

    # slack variables for final-state relaxation
    s_pos = cp.Variable(3)  # [dx, dy, dz]
    s_ang = cp.Variable(2)  # [dtheta, dphi]

    # initial-state constraints (hard)
    constraints = [
        x[0] == x_in,
        y[0] == y_in,
        z[0] == z_in,
        theta[0] == theta_in,
        phi[0] == phi_in,
        # relaxed final-state
        x[-1] == x_final + s_pos[0],
        y[-1] == y_final + s_pos[1],
        z[-1] == z_final + s_pos[2],
        theta[-1] == theta_final + s_ang[0],
        phi[-1] == phi_final + s_ang[1],
    ]

    # dynamics + control limits
    for k in range(N_app):
        cosT = np.cos(theta_nom[k])
        sinT = np.sin(theta_nom[k])
        cosP = np.cos(phi_nom[k])
        sinP = np.sin(phi_nom[k])

        constraints += [
            x[k+1] == x[k] + dt * (
                V*cosT*cosP
                - V*cosP*sinT*(theta[k] - theta_nom[k])
                - V*sinP*cosT*(phi[k]   - phi_nom[k])
            ),
            y[k+1] == y[k] + dt * (
                V*sinT*cosP
                + V*cosT*cosP*(theta[k] - theta_nom[k])
                - V*sinT*sinP*(phi[k]   - phi_nom[k])
            ),
            z[k+1] == z[k] + dt * (
                V*sinP + V*cosP*(phi[k] - phi_nom[k])
            ),
            theta[k+1] == theta[k] + dt * u_theta[k],
            phi[k+1]   == phi[k]   + dt * u_phi[k],
            cp.abs(u_theta[k]) <= u_theta_max,
            cp.abs(u_phi[k])   <= u_phi_max,
        ]

    # objective: control effort + slack penalties
    objective = cp.Minimize(
        cp.sum_squares(u_theta) +
        cp.sum_squares(u_phi) +
        w_pos * cp.sum_squares(s_pos) +
        w_ang * cp.sum_squares(s_ang)
    )

    problem = cp.Problem(objective, constraints)
    problem.solve(solver=cp.CLARABEL)

    return (
        x.value,
        y.value,
        z.value,
        theta.value,
        phi.value,
        u_theta.value,
        u_phi.value
    )


def loitering_trajectory_3d(
    N, dt, V,
    x0, y0, z0, theta0, phi0,
    x_cmd, y_cmd, z_cmd, alpha,
    u_theta_max, u_phi_max
):
    # slack penalty weights
    w_end_pos = 1e4
    w_end_ang = 1e4
    w_avg     = 1e4

    # nominal turn‐rate path
    nominal_u_theta = alpha / (N * dt)
    theta_nom = np.zeros(N + 1)
    phi_nom   = np.zeros(N + 1)
    theta_nom[0], phi_nom[0] = theta0, phi0
    for k in range(N):
        theta_nom[k + 1] = theta_nom[k] + dt * nominal_u_theta
        phi_nom[k + 1]   = phi_nom[k]   # nominal_u_phi = 0

    # decision variables
    x       = cp.Variable(N + 1)
    y       = cp.Variable(N + 1)
    z       = cp.Variable(N + 1)
    theta   = cp.Variable(N + 1)
    phi     = cp.Variable(N + 1)
    u_theta = cp.Variable(N)
    u_phi   = cp.Variable(N)

    # slack variables
    s_end_pos = cp.Variable(3)  # dx_end, dy_end, dz_end
    s_end_ang = cp.Variable(2)  # dtheta_end, dphi_end
    s_avg_pos = cp.Variable(3)  # dx_avg, dy_avg, dz_avg

    # constraints
    constraints = [
        # hard initial state
        x[0] == x0, y[0] == y0, z[0] == z0,
        theta[0] == theta0, phi[0] == phi0,
        # relaxed final‐state return
        x[-1] == x0 + s_end_pos[0],
        y[-1] == y0 + s_end_pos[1],
        z[-1] == z0 + s_end_pos[2],
        theta[-1] == theta0 + alpha + s_end_ang[0],
        phi[-1]   == phi0              + s_end_ang[1],
        # relaxed average‐position centering
        cp.sum(x)/(N + 1) == x_cmd + s_avg_pos[0],
        cp.sum(y)/(N + 1) == y_cmd + s_avg_pos[1],
        cp.sum(z)/(N + 1) == z_cmd + s_avg_pos[2],
    ]

    # dynamics + control constraints
    for k in range(N):
        cosT = np.cos(theta_nom[k])
        sinT = np.sin(theta_nom[k])
        cosP = np.cos(phi_nom[k])
        sinP = np.sin(phi_nom[k])
        constraints += [
            x[k+1] == x[k] + dt * (
                V*cosT*cosP
                - V*cosP*sinT*(theta[k] - theta_nom[k])
                - V*sinP*cosT*(phi[k]   - phi_nom[k])
            ),
            y[k+1] == y[k] + dt * (
                V*sinT*cosP
                + V*cosT*cosP*(theta[k] - theta_nom[k])
                - V*sinT*sinP*(phi[k]   - phi_nom[k])
            ),
            z[k+1] == z[k] + dt * (
                V*sinP + V*cosP*(phi[k] - phi_nom[k])
            ),
            theta[k+1] == theta[k] + dt * u_theta[k],
            phi[k+1]   == phi[k]   + dt * u_phi[k],
            cp.abs(u_theta[k]) <= u_theta_max,
            cp.abs(u_phi[k])   <= u_phi_max,
        ]

    # objective: control effort + slack penalties
    objective = cp.Minimize(
        cp.sum_squares(u_theta) +
        cp.sum_squares(u_phi) +
        w_end_pos * cp.sum_squares(s_end_pos) +
        w_end_ang * cp.sum_squares(s_end_ang) +
        w_avg     * cp.sum_squares(s_avg_pos)
    )

    problem = cp.Problem(objective, constraints)
    problem.solve(solver=cp.CLARABEL)

    return (
        x.value,
        y.value,
        z.value,
        theta.value,
        phi.value,
        u_theta.value,
        u_phi.value
    )


# ----------------------- #
#    FIGHTER JET TRAJECTORY PLANNER
# ----------------------- #
def fighter_jet_planner(params):
    # unpack…
    M = params["M"]
    x_cmd, y_cmd, z_cmd = params["x_cmd"], params["y_cmd"], params["z_cmd"]
    inboundMaxRadius = params["inboundMaxRadius"]
    N_app, dt = params["N_app"], params["dt"]
    u_theta_max, u_phi_max = params["u_theta_max"], params["u_phi_max"]
    V_loiter, N_loiter = params["V_loiter"], params["N_loiter"]
    alpha = params["alpha"]
    u_theta_max_loiter = params["u_theta_max_loiter"]
    u_phi_max_loiter = params["u_phi_max_loiter"]

    # 1) random inbound starts (same as before)
    r_in       = inboundMaxRadius * np.sqrt(np.random.rand(M))
    theta_rand = 2*np.pi*np.random.rand(M)
    x_in_all   = x_cmd + r_in * np.cos(theta_rand)
    y_in_all   = y_cmd + r_in * np.sin(theta_rand)
    z_in_all   = np.full(M, z_cmd)
    speed_all  = 8 + (15-8)*np.random.rand(M)
    heading_all= 2*np.pi*np.random.rand(M)

    # 2) loiter‐circle radius (same)
    R_common = (V_loiter * (N_loiter*dt)) / alpha

    # 3) pick your entry angles once, based on M
    if M == 2:
        entry_angles = [0, np.pi/2]
    elif M == 3:
        entry_angles = [0, 2*np.pi/3, 4*np.pi/3]
    else:
        entry_angles = 2*np.pi*np.arange(M)/M

    # 4) build entry‐point arrays
    x_entry_all = x_cmd + R_common * np.cos(entry_angles)
    y_entry_all = y_cmd + R_common * np.sin(entry_angles)
    z_entry_all = np.full(M, z_cmd)
    theta_entry_all = (np.array(entry_angles) + np.pi/2) % (2*np.pi)
    phi_entry_all = np.zeros(M)

    # 5) single loop: one approach + one loiter per candidate
    fullTraj = [None]*M
    start = time.time()
    for i in range(M):
        # inbound state
        x0, y0, z0 = x_in_all[i], y_in_all[i], z_in_all[i]
        th0, ph0   = heading_all[i], 0
        V0         = speed_all[i]

        # entry state
        xe, ye, ze = x_entry_all[i], y_entry_all[i], z_entry_all[i]
        the, phe   = theta_entry_all[i], 0

        # 5a) approach
        app = approach_trajectory_3d_optimal(
            N_app, dt, V0,
            x0, y0, z0, th0, ph0,
            xe, ye, ze, the, phe,
            u_theta_max, u_phi_max
        )
        if app[0] is None or np.any(np.isnan(app[0])):
            continue

        x_app, y_app, z_app, th_app, ph_app, _, _ = app

        # 5b) loiter
        loit = loitering_trajectory_3d(
            N_loiter, dt, V_loiter,
            xe, ye, ze, the, phe,
            x_cmd, y_cmd, z_cmd, alpha,
            u_theta_max_loiter, u_phi_max_loiter
        )
        if loit[0] is None or np.any(np.isnan(loit[0])):
            continue

        x_lo, y_lo, z_lo, th_lo, ph_lo, *_ = loit

        # 5c) stitch & store
        x_full = np.concatenate([x_app, x_lo[1:]])
        y_full = np.concatenate([y_app, y_lo[1:]])
        z_full = np.concatenate([z_app, z_lo[1:]])
        th_full = np.concatenate([th_app, th_lo[1:]])
        ph_full = np.concatenate([ph_app, ph_lo[1:]])
        v_full = np.concatenate([
            V0*np.ones_like(x_app),
            V_loiter*np.ones_like(x_lo[1:])
        ])
        vx = v_full * np.cos(th_full)*np.cos(ph_full)
        vy = v_full * np.sin(th_full)*np.cos(ph_full)
        vz = v_full * np.sin(ph_full)

        fullTraj[i] = {
            'x': x_full, 'y': y_full, 'z': z_full,
            'theta': th_full, 'phi': ph_full,
            'vx': vx, 'vy': vy, 'vz': vz
        }

    duration = time.time() - start
    return fullTraj, duration
