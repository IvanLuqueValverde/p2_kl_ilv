import numpy as np 

from ..motion_models import velocity_motion_model, velocity_motion_model_2
from ..observation_models import odometry_observation_model, odometry_observation_model_2

class KalmanFilter:

    def __init__(self, initial_state, initial_covariance, proc_noise_std = [0.05, 0.05, 0.02], obs_noise_std = [0.01, 0.01, 0.01]):
        self.mu = np.array(initial_state)  # Initial state estimate [x, y, theta]
        self.Sigma = np.array(initial_covariance)  # Initial uncertainty

        # Guardamos las funciones para poder recalcular A y B en cada predict
        self.state_transition_matrix_A, self.control_input_matrix_B = velocity_motion_model()

        # Inicializamos A y B
        self.A = self.state_transition_matrix_A()
        self.B = self.control_input_matrix_B(self.mu, 1.0)

        # Standard deviations for the noise in x, y, and theta (process or action model noise)
        self.proc_noise_std = np.array(proc_noise_std)
        self.R = np.diag(self.proc_noise_std ** 2)  # process noise covariance

        # Observation model (C)
        self.C = odometry_observation_model()  # The observation model to use

        # Standard deviations for the noise in x, y, theta (observation or sensor model noise)
        self.obs_noise_std = np.array(obs_noise_std)
        self.Q = np.diag(self.obs_noise_std ** 2)  # Observation noise covariance
            
    def predict(self, u, dt):
        self.A = self.state_transition_matrix_A()
        self.B = self.control_input_matrix_B(self.mu, dt)

        # Asegura las dimensiones correctas
        self.mu = np.array(self.mu).reshape(-1, 1)  # (3,1)
        u = np.array(u).reshape(-1, 1)              # (2,1)

        # Prediccion del estado
        self.mu = self.A @ self.mu + self.B @ u
        self.Sigma = self.A @ self.Sigma @ self.A.T + self.R

        # Devuelve mu como vector plano para facilitar su uso fuera
        self.mu = self.mu.flatten()
        return self.mu, self.Sigma

    def update(self, z):
        z = np.array(z).reshape(-1, 1)
        mu = self.mu.reshape(-1, 1)
        K = self.Sigma @ self.C.T @ np.linalg.inv(self.C @ self.Sigma @ self.C.T + self.Q)
        mu = mu + K @ (z - self.C @ mu)
        self.Sigma = (np.eye(len(self.mu)) - K @ self.C) @ self.Sigma
        self.mu = mu.flatten()
        return self.mu, self.Sigma

class KalmanFilter_2:
    def __init__(self, initial_state, initial_covariance,
                 proc_noise_std=[0.01, 0.01, 0.01, 0.01, 0.01, 0.001], obs_noise_std=[0.01, 0.01, 0.01, 0.01, 0.01, 0.001]):

        self.mu = initial_state  # Initial state estimate [x, y, theta, vx, vy, omega]
        self.Sigma = initial_covariance  # Initial uncertainty

        self.A, self.B = velocity_motion_model_2()  # Motion model matrices

        self.proc_noise_std = np.array(proc_noise_std)
        self.R = np.diag(self.proc_noise_std ** 2)  # Process noise covariance

        self.C = odometry_observation_model_2()  # Observation matrix
        self.obs_noise_std = np.array(obs_noise_std)
        self.Q = np.diag(self.obs_noise_std ** 2)  # Observation noise covariance

    def predict(self, u=None, dt=1.0):
        
        A = self.A(dt)
        self.mu = np.array(self.mu).reshape(-1, 1)  # (6,1)

        self.mu = A @ self.mu
        self.Sigma = A @ self.Sigma @ A.T + self.R

        self.mu = self.mu.flatten()
        return self.mu, self.Sigma

    def update(self, z):
        z = np.array(z).reshape(-1, 1)
        mu = self.mu.reshape(-1, 1)
        K = self.Sigma @ self.C.T @ np.linalg.inv(self.C @ self.Sigma @ self.C.T + self.Q)
        mu = mu + K @ (z - self.C @ mu)
        self.Sigma = (np.eye(len(self.mu)) - K @ self.C) @ self.Sigma
        self.mu = mu.flatten()
        return self.mu, self.Sigma