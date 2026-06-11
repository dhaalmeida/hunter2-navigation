#!/usr/bin/env python3
import numpy as np
from numpy import pi, cos, sin, exp
from numpy.random import randn
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Vector3
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

class HunterNav(Node):

    def __init__(self):
        super().__init__('hunter_nav')
        self.get_logger().info('HunterNav node has been started.')

        # Initiate cmd_vel publisher
        self.cmd_vel_pub_ = self.create_publisher(
            Twist, 'cmd_vel', 10)
        
        # Intiate target position subscriber
        self.target_x_ = None
        self.target_y_ = None
        self.target_pos_sub_ = self.create_subscription(
            Vector3, 'target1_position_topic', self.target_pos_callback, 10)
        
        # Initiate obstacle detection subscriber
        self.sector_dist_obs = None
        self.sector_dist_obs_sub_ = self.create_subscription(
            LaserScan, 'sector_dist_obs', self.sector_obs_callback, 10)

        # Initiate pose subscriber
        self.pose_topic_sub_ = self.create_subscription(
            PoseStamped, 'pose_topic', self.pose_callback, 10)

    # Callback for target position updates
    def target_pos_callback(self, msg: Vector3):
        self.target_x_ = msg.x
        self.target_y_ = msg.y
        #self.get_logger().info('Received target position: x=%.2f, y=%.2f' % (self.target_x_, self.target_y_))

    # Callback for obstacle detection updates
    def sector_obs_callback(self, msg: LaserScan):
        self.sector_dist_obs = msg.ranges # Store the obstacle distances for use in navigation logic
        self.sector_angles = msg.intensities # Store the corresponding angles for use in navigation logic
        #self.get_logger().info('Received obstacle distances: %s' % str(self.sector_dist_obs))

    # Callback for pose updates -> computes navigation logic and publishes velocity command
    def pose_callback(self, pose: PoseStamped):
        self.get_logger().info('Received pose: x=%.2f, y=%.2f' % (pose.pose.position.x, pose.pose.position.y))
        velocity_msg = Twist()

        # Wait for target position to be received before executing navigation logic
        if self.target_x_ is None or self.target_y_ is None:
            self.get_logger().warn('Waiting for target position...')
            return
        
        if self.sector_dist_obs is None:
            self.get_logger().warn('Waiting for obstacle distances...')
            return

        ####################
        # Navigation logic #
        ####################

        # -------------------
        # Target Acquisition
        # -------------------

        # Robot position
        x = pose.pose.position.x
        y = pose.pose.position.y

        # Convert quaternion to yaw (phi)
        # Assumes ROS standard quaternion (x, y, z, w) and planar motion (yaw about Z axis)
        qx = pose.pose.orientation.x
        qy = pose.pose.orientation.y
        qz = pose.pose.orientation.z
        qw = pose.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        phi = np.arctan2(siny_cosp, cosy_cosp)

        # Initialize target position
        target_x = self.target_x_
        target_y = self.target_y_
        psi_tar = np.arctan2(target_y - y, target_x - x)

        # Set parameters
        dt = 0.05  # Time step
        tau_tar = 30 * dt # Time constant for target acquisition
        lambda_tar = 1/tau_tar
        Q = 0.001

        # Compute target direction
        psi_tar = np.arctan2(target_y - y, target_x - x)

        # Compute f_tar
        f_tar = -lambda_tar * sin(phi - psi_tar)

        # Stochastic term
        f_stoch = np.sqrt(Q) * randn()
        f_tar_total = f_tar + f_stoch

        #wrobot = f_tar_total
        #vrobot = 0.5
        
        # -------------------
        # Obstacle Avoidance
        # -------------------

        # 1. Set parameter values
        tau_min = 10 * dt
        beta_1 = 1 / tau_min
        d_tar = np.sqrt((target_x - x)**2 + (target_y - y)**2)  # distance to target in meters
        beta_1_d = beta_1 * np.exp(-0/d_tar)
        beta_2 = 150
        rob_W = 1.50  # Hunter 2 width in meters
        rob_L = 0.98  # Hunter 2 length in meters
        Too_FAR = 5.00  # Threshold distance for obstacle influence in meters
        N = len(self.sector_dist_obs)
        theta_obs = list(self.sector_angles)  # sector angles in radians
        dist = list(self.sector_dist_obs)     # sector distances in meters
        Dtheta = abs(theta_obs[0] - theta_obs[1])

        # 1.2. Create vectors for psi_obs, lambda_obs, sigma, fobs
        psi_obs = np.zeros(N)
        lambda_obs = np.zeros(N)
        sigma = np.zeros(N)
        fobs = np.zeros(N)
        Fobs = 0.0

        # 2. For each sector compute the repulsive forcelet contribution
        for i in range(N):
            d_i = dist[i]  # already in meters
            psi_obs[i] = phi + theta_obs[i]
            lambda_obs[i] = beta_1_d * np.exp(-d_i / beta_2)
            Lx = rob_W / 2 * (np.sin(Dtheta / 2) * np.tan(Dtheta / 2) + np.cos(Dtheta / 2))
            sigma[i] = np.arctan2(np.tan(Dtheta / 2) * (rob_L / 2 + d_i) + Lx / 2, rob_L / 2 + d_i)

            angle_diff = phi - psi_obs[i]
            angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))  # Normalize angle difference to [-pi, pi]
            if d_i < Too_FAR:
                fobs[i] = lambda_obs[i] * (angle_diff) * np.exp(-(angle_diff)**2 / (2 * sigma[i]**2))
            else:
                fobs[i] = 0.0

            Fobs += fobs[i]

        # 3. Add noise
        fstoch = np.sqrt(Q) * randn()

        # 4. Compute total force and velocities
        F = Fobs + f_tar + fstoch
        wrobot = F
        #vrobot = 0.50

        self.get_logger().info(
            f'phi={phi:.3f}, f_tar={float(f_tar):.4f}, Fobs={float(Fobs):.4f}, '
            f'wrobot={float(wrobot):.4f}, d_tar={d_tar:.2f}, beta_1_d={beta_1_d:.6f}'
        )

        # ------------------------
        # Adaptive Linear Velocity
        # ------------------------

        # Compute Lyapunov potential
        U = np.zeros(N)
        k = np.zeros(N)
        Vpot = 0.0
        kr = 0.3
        for i in range(2,N-2):
            k[i] = kr*lambda_obs[i]*sigma[i]**2/np.sqrt(np.exp(1))
            U[i] = lambda_obs[i]*sigma[i]**2*exp(-(phi-psi_obs[i])**2/(2*sigma[i]**2)) - k[i]
            Vpot += k[i] * U[i]

        # Set linear velocity
        v_max = 1.50  # [m/s]
        T2C = 1 * tau_tar  # Time to collision constant [s]
        Dmax = v_max * T2C  # [m]

        if d_tar < 0.25:  # Close to target -> STOP
            vrobot = 0.0
        elif d_tar < Dmax:  # Velocity bound by target distance
            vrobot = d_tar / T2C
        elif min(dist) < Too_FAR:  # Velocity bound by obstacle distance
            beta = 0.5  # ganho ajustável
            vrobot = v_max * exp(-beta * max(Vpot, 0))
        else:
            vrobot = v_max

        # Show velocity
        self.get_logger().info(f'vrobot={vrobot:.4f} m/s')

        ############################
        # Publish velocity command #
        ############################
        velocity_msg.linear.x = vrobot
        velocity_msg.angular.z = wrobot
        self.cmd_vel_pub_.publish(velocity_msg)
        #self.get_logger().info('Published cmd_vel.')

def main(args=None):
    rclpy.init(args=args)

    node = HunterNav()
    rclpy.spin(node)

    rclpy.shutdown()