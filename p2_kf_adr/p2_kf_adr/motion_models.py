import numpy as np

def velocity_motion_model():
    def state_transition_matrix_A():
        # TODO: Define and return the 3x3 identity matrix A
        A=np.eye(3)
        return A

    def control_input_matrix_B(mu, delta_t):
        # TODO: Define B using current theta and timestep delta_t
        # B should apply linear and angular velocity to position and heading
        theta=mu[2]
        B=np.array([
            [np.cos(theta)*delta_t,0],
            [np.sin(theta)*delta_t,0],
            [0, delta_t]])
        return B
    return state_transition_matrix_A, control_input_matrix_B
def velocity_motion_model_2():
    def A(dt=1.0):
        # Define and return the 6x6 constant velocity model transition matrix
        return np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1]
        ])

    def B(mu, dt):
        # TODO: Return 6x2 zero matrix (no control input used in pure KF)
        B=np.array([
            [0,0],
            [0,0],
            [0,0],
            [0,0],
            [0,0],
            [0,0]
        ])
        

    return A, B
