import numpy as np
from visualization_msgs.msg import Marker
from tf_transformations import quaternion_from_euler


class ExtendedKalmanFilterUtils():
    def __init__(self, node):
        self.node = node

    ########################################### State Transision Function #######################################################################

    def state_transistion(self, x, u, dt) :
        """
        x : Previous state (3x1) vector
        u : Control input  (2x1) vector
        dt: Time interval  
        """

        x_t_1 , y_t_1, theta_t_1= x[0], x[1], x[2]
        v_t , w_t = u[0], u[1]

        if (abs(w_t) > 1e-6) :
            x_t     = x_t_1 + (v_t / w_t) * ( np.sin(theta_t_1 + w_t*dt) - np.sin(theta_t_1))
            y_t     = y_t_1 + (v_t / w_t) * (-np.cos(theta_t_1 + w_t*dt) + np.cos(theta_t_1))
            theta_t = theta_t_1 + w_t*dt
        else :
            x_t     = x_t_1 + v_t * dt * np.cos(theta_t_1)
            y_t     = y_t_1 + v_t * dt * np.sin(theta_t_1)
            theta_t = theta_t_1

        return np.array([x_t, y_t, theta_t])
        
    ########################################### Jacobian of State Transision Model  ##############################################################

    def jacobian_F(self, x, u, dt):
        """
        x : Previous state (3x1) vector
        u : Control input  (2x1) vector
        dt: Time interval 
        """

        x_t_1 , y_t_1, theta_t_1= x[0], x[1], x[2]
        v_t , w_t = u[0], u[1]
        F = np.eye(3)  
        
        if abs(w_t) > 1e-6:
            F[0, 2] = (v_t / w_t) * (np.cos(theta_t_1 + w_t * dt) - np.cos(theta_t_1))
            F[1, 2] = (v_t / w_t) * (np.sin(theta_t_1 + w_t * dt) - np.sin(theta_t_1))
        else:
            F[0, 2] = -v_t * np.sin(theta_t_1) * dt
            F[1, 2] =  v_t * np.cos(theta_t_1) * dt
        
        return F
    
    ########################################### Process Noise Covariance Matrix ##################################################################

    def process_noice_cov(self, u, dt):
        """
        Compute the process noise covriance using the control input
        """

        v_noise_std = 0.05  
        w_noise_std = 0.05

        sigma_x2     = (v_noise_std * dt) ** 2
        sigma_y2     = (v_noise_std * dt) ** 2 * 0.1 # much smaller than the noise in the x axis
        sigma_theta2 = (w_noise_std * dt) ** 2

        R = np.diag([sigma_x2, sigma_y2, sigma_theta2]) 

        return R
    
    ########################################### Prediction Function in EKF #######################################################################

    def prediction(self, x_prev, cov_prev, u, dt):
        """
        x_prev   : previous state
        cov_prev : Covariance matrix of the previos state
        u        : Control Input
        dt       : Time step
        """

        R      = self.process_noice_cov(u, dt)

        x_t_   = self.state_transistion(x_prev, u, dt)
        G_t_   = self.jacobian_F(x_prev, u, dt)
        cov_t_ = G_t_ @ cov_prev @ G_t_.T + R 

        return x_t_, cov_t_

    ########################################### Measurment Model for the Encoder Readings ########################################################

    def measurment_model(self, x_t_, C):
        """
        x_t_  : Predicted current state (3x1) vector
        C     : Measurment model matrix (3x3) matrix
        """

        h = C @ x_t_
        return h
    
    ########################################### Kalman Gain related to Encoder ####################################################################
        
    def encoder_kalman_gain(self, cov_t_, C):
        """
        cov_t_ : Covariance matrix of prediction state
        C      : Measurement model matrix of Encoders
        """

        x_std     = 0.0002
        y_std     = 0.0002
        theta_std = 0.0002

        sigma_x     = x_std ** 2
        sigma_y     = y_std ** 2
        sigma_theta = theta_std ** 2

        # Measurment noise covariance
        Q = np.diag([sigma_x, sigma_y, sigma_theta])

        innovation = C @ cov_t_ @ C.T + Q
        K = cov_t_ @ C.T @ np.linalg.inv(innovation)

        return K
    
    ########################################### Kalman Gain Related to GPS ########################################################################
        
    def GPS_kalman_gain(self, cov_t_, C):
        """
        cov_t_ : Covariance matrix of prediction state
        C      : Measurement model matrix of GPS
        """

        x_std = 0.0001
        y_std = 0.0001

        sigma_x = x_std ** 2
        sigma_y = y_std ** 2

        # Measurment noise covariance
        Q = np.diag([sigma_x, sigma_y])

        innovation = C @ cov_t_ @ C.T + Q
        K = cov_t_ @ C.T @ np.linalg.inv(innovation)

        return K
    
    ########################################### Correction Step in EKF ############################################################################

    def correction(self, x_t_, z, C, cov_t_, K) :
        """
        x_t_   : Predicted state (3x1) vector
        z      : Sensor reading
        C      : Measurment model matrix
        cov_t_ : covariance matrix from prediction step 
        K      : Kalman gain     
        """

        h = self.measurment_model(x_t_, C)
        I = np.eye(cov_t_.shape[0])

        x_t   = np.asarray(x_t_ + K @ (z - h) )
        cov_t = np.asarray((I - K @ C) @ cov_t_)

        return x_t, cov_t
    
    ###############################################################################################################################################

        




