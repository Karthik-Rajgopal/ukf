def estInitialize():
    import numpy as np
    import scipy as sp  

    # state = [x, y, θ, B, r]
    x0 = 0.0
    y0 = 0.0
    theta0 = np.pi / 4
    B0 = 0.8
    r0 = 0.425

    state_mean0 = np.array([x0, y0, theta0, B0, r0], dtype=float)

    # initial covariance
    std_x0 = 5.0
    std_y0 = 5.0
    std_theta0 = 0.785
    std_B0 = 0.08
    std_r0 = 0.021

    P0 = np.diag([std_x0**2, std_y0**2, std_theta0**2, std_B0**2, std_r0**2])

    # process noise (per 1 s)
    process_std_x = 0.2
    process_std_y = 0.2
    process_std_theta = 0.1
    process_std_B = 0.0
    process_std_r = 0.0

    Q_base = np.diag(
        [process_std_x**2, process_std_y**2, process_std_theta**2,
         process_std_B**2, process_std_r**2]
    )

    # measurement noise
    meas_std = 1.0
    R = np.diag([meas_std**2, meas_std**2])

    # UKF weights
    L = state_mean0.size
    alpha = 1.0
    beta = 2.0
    kappa = 0.0
    lambd = alpha**2 * (L + kappa) - L

    Wm = np.zeros(2 * L + 1)
    Wc = np.zeros(2 * L + 1)
    Wm[0] = lambd / (L + lambd)
    Wc[0] = lambd / (L + lambd) + (1 - alpha**2 + beta)
    Wm[1:] = 1.0 / (2 * (L + lambd))
    Wc[1:] = 1.0 / (2 * (L + lambd))

    # build internal state to return
    internalState = {
        'mean': state_mean0,
        'cov': P0,
        'Q_base': Q_base,
        'R': R,
        'Wm': Wm,
        'Wc': Wc,
        'state_dim': L
    }

    studentNames = ['Mauricio Vergara', 'Karthik Rajgopal', 'Jannik Heinen']  
    estimatorType = 'UKF'

    return internalState, studentNames, estimatorType
