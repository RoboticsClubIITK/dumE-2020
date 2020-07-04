import numpy as np
from numpy.linalg import inv, pinv
from numpy.random import randn

class Kalman:
    def __init__(self, x_init, P_init, dt):
        # State and Covariance
        self.x = np.mat(x_init, dtype=np.float32)
        self.P = np.mat(P_init, dtype=np.float32)

        # Transformation Matrices
        self.A = np.mat([[1, 0, 0, dt,  0,  0],
                         [0, 1, 0,  0, dt,  0],
                         [0, 0, 1,  0,  0, dt],
                         [0, 0, 0,  1,  0,  0],
                         [0, 0, 0,  0,  1,  0],
                         [0, 0, 0,  0,  0,  1]],
                        dtype=np.float32)
        self.C = np.mat([[1, 0, 0, 0, 0, 0],
                         [0, 1, 0, 0, 0, 0],
                         [0, 0, 1, 0, 0, 0]],
                        dtype=np.float32)

        # Noise
        self.Q = np.mat(np.diag([0.5*randn(), 0.5*randn(), 0.5*randn(), 2.5*randn(), 2.5*randn(), 2.5*randn()]), dtype=np.float32)
        self.R = np.mat(np.diag([0.25*randn(), 0.25*randn(), 0.25*randn()]), dtype=np.float32)

    def predict(self):
        # Predict with Motion Model
          # State and Covariance
        x = self.x
        P = self.P

          # Transformation Matrices
        A = self.A

          # Motion Noise
        Q = self.Q

          # Predict
        self.x = A @ x
        self.P = A @ P @ A.T + Q

    def update(self, y, area):
        # Update with Measurement Model
          # State and Covariance
        x = self.x
        P = self.P

          # Transformation Matrices
        C = self.C

          # Measurement and Noise
        y = np.mat(y, dtype=np.float32)
        R = self.R/area

          # Update
        K = P @ C.T @ pinv(C @ P @ C.T + R)

        y_check = C @ x
        
        self.x = x + K @ (y - y_check)
        self.P = (np.identity(P.shape[0]) + K @ C) @ P
