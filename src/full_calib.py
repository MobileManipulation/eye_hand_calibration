#!/usr/bin/env python
"""
Full-stack calibration script: Round 1

Big idea:
 * We need to calibrate both intrinsics and extrinsics at once
 * Inputs:
    * Four corners of Aruco tag, in pixels
    * Four corners of Aruco tag, in world

 * Process:
    * Project world corner points into image
    * Cost is the distance from the projected point to its measured point

Notes:
 * calibration_target frame is currently defined in the center of the plate,
   not at the front surface
 * As implemented below, we don't allow any more transformations than
   the camera extrinsics. This is a problem; we want to be able to
   minimize over many of the transforms between the arm and the camera.
 * Need a way to specify which transforms to optimize. It's not always joints!
   In fact, for this robot, we only solve for fixed transforms.

Projection:
 * Must be Autodiff-able (so Numpy/Theano only)
 * Shape: Big array of projection transforms * Array of point
 * First cut: Just translate everything into kinect1_link frame beforehand
"""

import math
import autograd.numpy as np
from autograd import grad
from scipy.optimize import minimize
import tf.transformations as transf

def print_func(x):
    print x

class TestDataGenerator(object):
    def __init__(self, grid_size, extrinsics, intrinsics):
        self.extrinsics = extrinsics
        self.intrinsics = intrinsics

        # Build up testing data in a grid
        z = np.linspace(-0.2, 0.2, num=grid_size)
        y = np.linspace(-0.2+0.025, 0.2+0.025, num=grid_size)
        x = np.linspace(0.5, 1.2, num=grid_size)
        g = np.meshgrid(x, y, z)
        kin = np.stack(map(np.ravel, g))
        np.concatenate([kin, np.ones([1, kin.shape[1]])])

        # Compute all truth data in various frames
        self.kinect_truth = kin
        self.optical_truth = opt = toCameraFrame(kin, extrinsics)
        self.pixel_truth = cameraProjection(opt, intrinsics)

    def getCalibratorArgs(self):
        return (
            self.kinect_truth,
            self.pixel_truth,
            self.extrinsics,
            self.intrinsics
        )

class DataReader(object):
    def __init__(self, filename):
        # Steps:
        # 1. Read data in
        data = np.loadtxt(
            filename,
            delimiter=",",
            converters={2: lambda x: 0} # Read timestamps as 0
        )

        #   * 3D points in kinect1 frame
        self.kinect_points = np.concatenate([ # We stored them backwards...
            data[:, 43:46], # Top right
            data[:, 57:60], # Bottom right
            data[:, 50:53], # Bottom left
            data[:, 36:39]  # Top left
        ], axis=0).T        # We want 3 rows by N columns
        self.kinect_points = np.concatenate([
            self.kinect_points,
            np.ones([1, self.kinect_points.shape[1]])
        ])
        print "kinect_points:", self.kinect_points

        #   * Corresponding pixel locations
        self.pixels = np.concatenate([
            data[:, 28:30], # Top right
            data[:, 30:32], # Bottom right
            data[:, 32:34], # Bottom left
            data[:, 34:36]  # Top left
        ], axis=0).T        # We want 2 rows by N columns
        print "pixels:", self.pixels

        intrinsics = np.array([0, 0, 0, 0, 525, 525, 310, 260])
        extrinsics = np.array([0, 0, 0, 0, 0, 0, 1]) # [x, y, z, qx, qy, qz, qw]

        #   * Initial guess for extrinsics (kinect1_link -> kinect1_rgb_frame)
        # self.extrinsics = np.array([0, -0.01, 0, 0, 0, 0, 1]) # Default guess
        self.extrinsics = extrinsics

        #   * Initial guess for intrinsics [k1, k2, p1, p2, fx, fy, cx, cy]
        # self.intrinsics = np.array([0, 0, 0, 0, 525, 525, 320, 240])
        self.intrinsics = intrinsics

        # Filtering
        temp = toCameraFrame(self.kinect_points, extrinsics)
        proj = cameraProjection(temp, intrinsics)
        print "Projected!", proj

        # Check every projected column
        idx = list()
        for i, pix in enumerate(proj.T):
            if pix[0] < 0 or pix[0] > 640:
                idx.append(i)
            elif pix[1] < 0 or pix[1] > 480:
                idx.append(i)
            else:
                continue
            print "Bad pixel!"
            print "pre:", self.kinect_points.T[i]
            print "proj:", pix
        print "Removing bad indicies:", idx
        self.kinect_points = np.delete(self.kinect_points, idx, axis=1)
        self.pixels = np.delete(self.pixels, idx, axis=1)

    def get_data(self):
        return (
            self.kinect_points,
            self.pixels,
            self.extrinsics,
            self.intrinsics
        )

class SystemCalibrator(object):
    def __init__(self, kinect_points, pixels, extrinsics, intrinsics):
        """
        kinect_points: XYZ coordinates of points, expressed in kinect1_link frame
        pixels: pixel coordinates in kinect1/rgb/image_color (unrectified!)
        extrinsics: initial guess of extrinsics
        intrinsics: initial guess of intrinsics
        """
        self.x0 = np.concatenate([extrinsics, intrinsics])
        self.kinect_truth = kinect_points
        self.pixel_truth = pixels

        print "kinect:", self.kinect_truth
        print "pixels:", self.pixel_truth

    def optimize(self, its=10000, acc=1e-12):
        # x0 = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 500, 500,320, 240], dtype=np.float64)
        x0 = self.x0

        spread = np.array([0.03, 0.03, 0.03, 0, 0, 0, 0, 0.3, 0.4,  0.3,  0.3,  100,  100,  100,  100])
        skip_idx = spread==0
        print "skip_idx", skip_idx
        bounds = zip(x0 - spread, x0 + spread)

        # Don't naively bound the quaternion; it is bounded below
        for i in range(3,7):
            bounds[i] = (None, None)
        print bounds

        # Bind quaternion normal
        constraints = [
            # eps_ball(0.3, range(0, 3)),
            quat_norm(range(3, 7))
            # cone_threshold(range(3,7), x0[3:7], 10.0 * math.pi / 180.0)
        ]

        options = {
            "maxiter": its,
            "maxfun": 500000,
            "eps": 1e-9,
            "ftol": 1e-12
        }

        return minimize(
            self.objective,
            x0,
            method="L-BFGS-B",
            bounds=bounds,
            # jac=grad(self.objective), # Autograd unsupported :(
            constraints=constraints,
            callback=print_func,
            options=options
        )

    def objective(self, x):
        # The state in this minimization are the parameters we're solving for
        # This includes extrinsics (trans/quat) and intrinsics (f/c/k/p)

        # Extract state into meaningful variables
        # State = ( t[3], quat[4], k1, k2, p1, p2, fx, fy, cx, cy )
        extrinsics = x[:7]
        intrinsics = x[7:]

        optical_lies = toCameraFrame(self.kinect_truth, extrinsics)
        pixel_lies = cameraProjection(optical_lies, intrinsics)

        # We want to minimize the distance of the projection from ground truth
        return np.linalg.norm(pixel_lies - self.pixel_truth)


def quat_norm(idx):
    i = np.array(idx)
    def c(x):
        return np.linalg.norm(x[i]) - 1.0
    return {
        "type": "eq",
        "fun": c,
        "jac": grad(c)
    }

def cone_threshold(idx, quat, thresh):
    i = np.array(idx)
    def c(x):
        inner = np.inner(x[i], quat)
        print "Inner: ", inner
        return thresh - (1 - 2*(inner**2))
    return {
        "type": "ineq",
        "fun": c,
        "jac": grad(c)
    }

# Not good! Needs regularization since grad(x -> 0) -> infinity
def eps_ball(eps, idx):
    i = np.array(idx)
    def c(x):
        return eps - np.linalg.norm(x[i])
    return {
        "type": "ineq",
        "fun": c
    }

def build_xform_mat(state):
    t, q = (state[:3], state[3:7])
    xform = transf.quaternion_matrix(q)
    xform[:3, 3] = t
    return xform

def toCameraFrame(kinect_points, state):
    # Currently, this assumes all points are in kinect1_link frame
    # TODO: Add support for points in any frame

    # From kinect1_link to kinect1_rgb_frame
    kinect1_link_to_rgb = build_xform_mat(state)
    rgb_points = np.dot(
        np.linalg.inv(kinect1_link_to_rgb),
        # kinect1_link_to_rgb,
        kinect_points
    )

    # From kinect1_rgb_frame to kinect1_rgb_optical_frame
    body_to_camera = transf.euler_matrix(-math.pi/2, 0, -math.pi/2, axes="sxyz")
    optical_points = np.dot(body_to_camera.T, rgb_points)

    return optical_points

def cameraProjection(X, intrinsics):
    # This projection is based on math here:
    # http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    # import pdb; pdb.set_trace()

    # Extract intrinsics
    k1, k2 = intrinsics[0], intrinsics[1]
    p = intrinsics[2:4, None]
    f = intrinsics[4:6, None]
    c = intrinsics[6:8, None]

    # X is a 4xN vector of 3D points (x, y, z)

    # Homogenize the points to 2D by dividing by Z
    # We only want the XY coordinates past here
    X_p = X[:2, :] / X[2, :] # Each point is a column (x', y')'

    # Compute r^2
    r_sq = np.linalg.norm(X_p, axis=0, keepdims=True)**2

    # Big ugly distortion
    X_pp  = X_p*(1+ k1*r_sq + k2*r_sq**2)
    X_pp += p[::-1]*(r_sq + 2*X_p**2)
    X_pp += 2*p*np.prod(X_p,axis=0, keepdims=True)

    # Finally, UV-projection to image space
    pixels = f*X_pp + c
    return pixels

def cameraMats(fcs):
    """
    fcs = [fx fy cx cy]
    """
    K = np.zeros([3,3])
    K[0,0] = fcs[0]
    K[1,1] = fcs[1]
    K[0,2] = fcs[2]
    K[1,2] = fcs[3]
    K[2,2] = 1

    P = np.zeros([3, 4])
    P[0:3, 0:3] = K

    return K, P

def projectPoints(kinect_points, pixels, extrinsics, intrinsics):
    temp = toCameraFrame(kinect_points, extrinsics)
    pixels = cameraProjection(temp, intrinsics)
    return pixels

if __name__=="__main__":
    # Testing values
    # intrinsics = np.array([0.14, -0.23, -0.003, 0.002, 525, 525, 310, 260])
    # extrinsics = np.array([0, -0.025, 0, 0, 0, 0, 1]) # [x, y, z, qx, qy, qz, qw]
    # grid_size = 50
    # data = TestDataGenerator(grid_size, extrinsics, intrinsics).getCalibratorArgs()

    # Read data!
    filename = "/home/momap/momap_data/log_robot/20170809/20170809T163531_full_calib_ir/20170809T163531_full_calib-merged.csv"
    data = DataReader(filename).get_data()
    cal = SystemCalibrator(*data)
    result = cal.optimize()
    print
    print result
    print

    # Perform projection for validation
    pixels = projectPoints(data[0], data[1], result.x[:7], result.x[7:])
    diff = np.abs(pixels - data[1])
    print "Pixel diffs (one axis)"
    print "----------------------"
    print "Min:", np.min(diff)
    print "Max:", np.max(diff)
    print

    dists = np.linalg.norm(diff, axis=0)
    print "Pixel distances (two axis)"
    print "--------------------------"
    print "Min:", np.min(dists)
    print "Max:", np.max(dists)
    print
    print "XYZ:", result.x[:3]
    print "RPY:", np.array(transf.euler_from_quaternion(result.x[3:7]))
    print

    K,P = cameraMats(result.x[-4:])
    print "K:", np.array2string(K.flatten(), separator=", ")
    print "P:", np.array2string(P.flatten(), separator=", ")

    print "D:", np.array2string(result.x[7:11], separator=", ")
