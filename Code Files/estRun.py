def estRun(time, dt, internalStateIn, steeringAngle, pedalSpeed, measurement):
    import numpy as np
    import scipy as sp  # lin‑alg utils

    # unpack filter state
    mean = internalStateIn['mean']
    P = internalStateIn['cov']
    Q_base = internalStateIn['Q_base']
    R = internalStateIn['R']
    Wm = internalStateIn['Wm']
    Wc = internalStateIn['Wc']
    L = internalStateIn['state_dim']

    # sigma‑point spread factor
    c = np.sqrt(1.0 / (2.0 * Wm[1]))

    # Cholesky of P
    try:
        sqrtP = np.linalg.cholesky(P)
    except np.linalg.LinAlgError:
        sqrtP = np.linalg.cholesky(P + 1e-9 * np.eye(L))

    # sigma points
    sigma_points = np.zeros((2 * L + 1, L))
    sigma_points[0] = mean
    for i in range(L):
        offset = c * sqrtP[:, i]
        sigma_points[1 + i] = mean + offset
        sigma_points[1 + L + i] = mean - offset

    # propagate sigma points through motion model
    sigma_pred = np.zeros_like(sigma_points)
    gamma = steeringAngle
    omega = pedalSpeed
    for j in range(2 * L + 1):
        x_j = sigma_points[j]
        v = 5.0 * x_j[4] * omega
        x_pos_new = x_j[0] + v * np.cos(x_j[2]) * dt
        y_pos_new = x_j[1] + v * np.sin(x_j[2]) * dt
        theta_new = x_j[2] + (v / x_j[3]) * np.tan(gamma) * dt
        sigma_pred[j] = [x_pos_new, y_pos_new, theta_new, x_j[3], x_j[4]]

    # predicted mean (handle angle wrap)
    x_pred_mean = np.zeros(L)
    sum_sin = sum_cos = 0.0
    for j in range(2 * L + 1):
        x_pred_mean += Wm[j] * sigma_pred[j]
        sum_sin += Wm[j] * np.sin(sigma_pred[j, 2])
        sum_cos += Wm[j] * np.cos(sigma_pred[j, 2])
    x_pred_mean[2] = np.arctan2(sum_sin, sum_cos)

    # predicted covariance
    P_pred = np.zeros((L, L))
    for j in range(2 * L + 1):
        dx = sigma_pred[j] - x_pred_mean
        dx[2] = np.arctan2(np.sin(dx[2]), np.cos(dx[2]))
        P_pred += Wc[j] * np.outer(dx, dx)
    P_pred += Q_base * dt

    # measurement update (if position provided)
    z_meas = np.array(measurement)
    if not (np.isnan(z_meas[0]) or np.isnan(z_meas[1])):
        m_dim = 2
        Z_sigma = np.zeros((2 * L + 1, m_dim))
        for j in range(2 * L + 1):
            x_state = sigma_pred[j]
            meas_x = x_state[0] + 0.5 * x_state[3] * np.cos(x_state[2])
            meas_y = x_state[1] + 0.5 * x_state[3] * np.sin(x_state[2])
            Z_sigma[j] = [meas_x, meas_y]

        z_pred_mean = np.zeros(m_dim)
        for j in range(2 * L + 1):
            z_pred_mean += Wm[j] * Z_sigma[j]

        S = np.zeros((m_dim, m_dim))
        P_xz = np.zeros((L, m_dim))
        for j in range(2 * L + 1):
            dz = Z_sigma[j] - z_pred_mean
            S += Wc[j] * np.outer(dz, dz)
            dx = sigma_pred[j] - x_pred_mean
            dx[2] = np.arctan2(np.sin(dx[2]), np.cos(dx[2]))
            P_xz += Wc[j] * np.outer(dx, dz)
        S += R

        K = P_xz.dot(np.linalg.inv(S))
        innovation = z_meas - z_pred_mean
        new_mean = x_pred_mean + K.dot(innovation)
        new_P = P_pred - K.dot(S).dot(K.T)
        new_mean[2] = np.arctan2(np.sin(new_mean[2]), np.cos(new_mean[2]))
    else:
        new_mean = x_pred_mean
        new_P = P_pred

    # outputs
    x_est, y_est, theta_est = new_mean[0], new_mean[1], new_mean[2]
    theta_est = (theta_est + np.pi) % (2 * np.pi) - np.pi

    internalStateOut = {
        'mean': new_mean,
        'cov': new_P,
        'Q_base': Q_base,
        'R': R,
        'Wm': Wm,
        'Wc': Wc,
        'state_dim': L
    }

    return x_est, y_est, theta_est, internalStateOut
