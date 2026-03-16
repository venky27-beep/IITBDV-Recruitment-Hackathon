import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial import distance
import pandas as pd

# ── Load Track from CSV ───────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
df = pd.read_csv(os.path.join(_HERE, "small_track.csv"))

BLUE_CONES   = df[df["tag"] == "blue"      ][["x", "y"]].values.astype(float)
YELLOW_CONES = df[df["tag"] == "yellow"    ][["x", "y"]].values.astype(float)
BIG_ORANGE   = df[df["tag"] == "big_orange"][["x", "y"]].values.astype(float)

_cs               = df[df["tag"] == "car_start"].iloc[0]
CAR_START_POS     = np.array([float(_cs["x"]), float(_cs["y"])])
CAR_START_HEADING = float(_cs["direction"])   # radians (0 = east)

MAP_CONES = np.vstack([BLUE_CONES, YELLOW_CONES])


# ── Build Approximate Centerline ──────────────────────────────────────────────
def _build_centerline():
    """
    Pair each blue cone with its nearest yellow cone, take the midpoint,
    then sort CLOCKWISE around the track centroid so pure-pursuit drives CW.
    """
    center = np.mean(MAP_CONES, axis=0)
    D      = distance.cdist(BLUE_CONES, YELLOW_CONES)
    mids   = np.array(
        [(BLUE_CONES[i] + YELLOW_CONES[np.argmin(D[i])]) / 2.0
         for i in range(len(BLUE_CONES))]
    )
    angles = np.arctan2(mids[:, 1] - center[1], mids[:, 0] - center[0])
    return mids[np.argsort(angles)[::-1]]   # descending angle = clockwise


CENTERLINE = _build_centerline()


# ── Simulation Parameters ─────────────────────────────────────────────────────
SENSOR_RANGE = 12.0   # metres – sensor visibility radius
NOISE_STD    = 0.20   # metres – measurement noise std-dev
WHEELBASE    = 3.0    # metres – bicycle model wheelbase
DT           = 0.1    # seconds – time step
SPEED        = 7.0    # m/s
LOOKAHEAD    = 5.5    # pure-pursuit lookahead distance (m)
N_FRAMES     = 130    # ≈ one full lap


# ── Utility Functions ─────────────────────────────────────────────────────────
def angle_wrap(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


def pure_pursuit(pos: np.ndarray, heading: float, path: np.ndarray) -> float:
    """Compute steering angle (rad) to follow *path* via pure-pursuit."""
    dists   = np.linalg.norm(path - pos, axis=1)
    nearest = int(np.argmin(dists))
    n       = len(path)
    target  = path[(nearest + 5) % n]       # fallback lookahead
    for k in range(nearest, nearest + n):
        pt = path[k % n]
        if np.linalg.norm(pt - pos) >= LOOKAHEAD:
            target = pt
            break
    alpha = angle_wrap(
        np.arctan2(target[1] - pos[1], target[0] - pos[0]) - heading
    )
    steer = np.arctan2(2.0 * WHEELBASE * np.sin(alpha), LOOKAHEAD)
    return float(np.clip(steer, -0.6, 0.6))


def local_to_global(local_pts: np.ndarray,
                    pos: np.ndarray, heading: float) -> np.ndarray:
    """Rotate + translate points from the car's local frame to world frame."""
    c, s = np.cos(heading), np.sin(heading)
    R    = np.array([[c, -s], [s, c]])       # local → world rotation
    return (R @ local_pts.T).T + pos


def get_measurements(pos: np.ndarray, heading: float) -> np.ndarray:
    """
    Simulate a 2-D lidar: return visible cone positions as noisy
    measurements in the car's LOCAL frame (x = forward, y = left).
    """
    dists   = np.linalg.norm(MAP_CONES - pos, axis=1)
    visible = MAP_CONES[dists < SENSOR_RANGE]
    if len(visible) == 0:
        return np.zeros((0, 2))
    c, s = np.cos(heading), np.sin(heading)
    R    = np.array([[c, s], [-s, c]])       # world → local (transpose of above)
    local = (R @ (visible - pos).T).T
    return local + np.random.normal(0, NOISE_STD, local.shape)


def draw_track(ax, alpha_b: float = 0.4, alpha_y: float = 0.4) -> None:
    ax.scatter(BLUE_CONES[:, 0],   BLUE_CONES[:, 1],
               c="royalblue", marker="^", s=65,  alpha=alpha_b,
               zorder=2, label="Blue cones")
    ax.scatter(YELLOW_CONES[:, 0], YELLOW_CONES[:, 1],
               c="gold",      marker="^", s=65,  alpha=alpha_y,
               zorder=2, label="Yellow cones")
    ax.scatter(BIG_ORANGE[:, 0],   BIG_ORANGE[:, 1],
               c="darkorange", marker="s", s=100, alpha=0.7,
               zorder=2, label="Start gate")


def draw_car(ax, pos: np.ndarray, heading: float) -> None:
    ax.scatter(pos[0], pos[1], c="red", s=160, zorder=7, label="Car")
    ax.arrow(pos[0], pos[1],
             2.2 * np.cos(heading), 2.2 * np.sin(heading),
             head_width=0.8, fc="red", ec="red", zorder=8)


def setup_ax(ax, subtitle: str = "") -> None:
    ax.set_xlim(-28, 28)
    ax.set_ylim(-22, 22)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25, linestyle="--")
    if subtitle:
        ax.set_title(subtitle, fontsize=10)


# ── Abstract Base ─────────────────────────────────────────────────────────────
class Bot:
    def __init__(self):
        self.pos     = CAR_START_POS.copy()   # (2,) float64
        self.heading = CAR_START_HEADING      # radians

    def data_association(self, measurements, current_map):
        raise NotImplementedError

    def localization(self, velocity, steering):
        raise NotImplementedError

    def mapping(self, measurements):
        raise NotImplementedError


# ──  Solution ──────────────────────────────────────────────────────────
class Solution(Bot):
    def __init__(self):
        super().__init__()
        self.learned_map  = []                    # list of np.ndarray (2,)
        # Internal state exposed for visualisation
        self._global_meas = np.zeros((0, 2))
        self._assoc       = np.array([], dtype=int)
        self._mu = np.array([self.pos[0], self.pos[1], self.heading])  # (x, y, ψ) state estimate
        self._Sigma = np.diag([0.01,0.01, np.radians(1.0)])**2  # initial uncertainty covariance matrix


    # ------------------------------------------------------------------
    def localization(self, velocity, steering):
        """
        Bicycle kinematic model (dead reckoning):
            ẋ = v·cos(ψ)
            ẏ = v·sin(ψ)
            ψ̇ = (v / L)·tan(δ)
        """
        px,py,psi = self._mu
        self._mu[0]  += velocity * np.cos(psi) * DT
        self._mu[1]  += velocity * np.sin(psi) * DT
        self._mu[2] += (velocity / WHEELBASE) * np.tan(steering) *DT
        '''self.pos[0]  += velocity * np.cos(self.heading) * DT
        self.pos[1]  += velocity * np.sin(self.heading) * DT
        self.heading  = angle_wrap(
            self.heading + (velocity / WHEELBASE) * np.tan(steering) * DT
        )'''
        F = np.array([[1,0, -velocity * np.sin(psi) * DT,],[0,1, velocity * np.cos(psi) * DT],[0,0,1]])# this is the jacobian matrix of the motion of the car wrt to its current state del(xt+1)/del(xt)
        var_x = 0.05**2
        var_y = 0.05**2
        var_yaw = np.radians(0.50)**2

        # Create a 1D array of the diagonal elements
        diagonal_elements = np.array([var_x, var_y, var_yaw])

        # Use np.diag() with the 1D array to create the 2D diagonal matrix
        Q = np.diag(diagonal_elements)
        self._Sigma = F @ self._Sigma @ F.T + Q #this is the covaraince update step of the EKF and we add the noise 
        self._Sigma = (self._Sigma + self._Sigma.T)*0.5#this makes it symmetric for easier calculations

        R = (NOISE_STD**2) * np.eye(2) # this is the measurement noise covariance matri     x which models the uncertainty in the measurements
        meas = get_measurements(self._mu[0:2], self._mu[2]) # this is the measurement we get from the environment in the local frame of the car
        gm = local_to_global(meas, self._mu[0:2], self._mu[2]) # this converts the measurements to the global frame of reference
        D = distance.cdist(gm,MAP_CONES) # this calculates the distance between the measurements and the map cones
        for i, row in enumerate(D):
            j = np.argmin(row) # this finds the index of the closest cone in the map for each measurement
            dist = row[j] # this is the distance to the closest cone
            if dist > 3:
                continue # if the distance is too large we ignore this measurement
            px2,py2,psi2 = self._mu
            c,s = np.cos(psi2), np.sin(psi2)
            z_l = np.array([[c,s],[-s,c]]) @ (MAP_CONES[j] - self._mu[0:2]) # this is the expected measurement in the local frame of the car for the closest cone
            dz = np.array([[-s,-c],[-c,s]]) @z_l #this is the derivative of z_l wrt psi
            H = np.array([[1,0,dz[0]],[0,1,dz[1]]]) # this is the jacobian matrix of the measurement model
            z_hat = np.array([[c, -s], [s, c]]) @ z_l + self._mu[:2]# this is the expected measurement in the global frame of reference
            nu    = gm[i] - z_hat#this is the difference b/w actual measumetemnt and expected measuerement
            S = H @ self._Sigma @ H.T + R# this is the innovation covariance matrix
            if float(nu @ np.linalg.solve(S, nu)) > 5.991:#if this is true then measurement is with outlier for 95% confidence hence its probably a wrong value
                continue
            K = self._Sigma @ H.T @ np.linalg.inv(S)# this tells me hoe much i should trust the measurement wrt to my estimate

            self._mu    = self._mu + K @ nu#this will update the state
            self._mu[2] = angle_wrap(self._mu[2])
            IKH         = np.eye(3) - K @ H# this will update the covariance matrix 
            self._Sigma = IKH @ self._Sigma @ IKH.T + K @ R @ K.T 
            self._Sigma = (self._Sigma + self._Sigma.T) * 0.5
        self.pos     = self._mu[:2].copy() # this will update the position of the car to the estimated position
        self.heading = float(self._mu[2])# this will update the heading of the car to the estimated heading
        



# ── Problem 2 – Localization ───────────────────────────────────────────────────
def make_problem2():
    """
    Visualise dead-reckoning: the magenta trail is the car's estimated
    trajectory built purely from the kinematic model and steering commands.
    """
    sol     = Solution()
    path_x  = [float(sol.pos[0])]
    path_y  = [float(sol.pos[1])]
    fig, ax = plt.subplots(figsize=(10, 7))
    fig.suptitle("Problem 2 – Localization  (Dead Reckoning / Kinematic Model)",
                 fontsize=13, fontweight="bold")

    def update(frame):
        ax.clear()
        steer = pure_pursuit(sol.pos, sol.heading, CENTERLINE)
        sol.localization(SPEED, steer)
        path_x.append(float(sol.pos[0]))
        path_y.append(float(sol.pos[1]))

        draw_track(ax)
        ax.plot(path_x, path_y, color="magenta", lw=2.0,
                alpha=0.85, zorder=4, label="Dead-reckoning path")
        draw_car(ax, sol.pos, sol.heading)
        setup_ax(ax,
            f"Frame {frame+1}/{N_FRAMES}  –  "
            f"pos=({sol.pos[0]:.1f}, {sol.pos[1]:.1f})  "
            f"ψ={np.degrees(sol.heading):.1f}°")
        ax.legend(loc="upper right", fontsize=8, framealpha=0.8)
        fig.tight_layout(rect=[0, 0, 1, 0.95])

    ani = FuncAnimation(fig, update, frames=N_FRAMES, interval=100, repeat=True)
    return fig, ani

 #── Localization Accuracy Test ────────────────────────────────────────────────
def compare_baseline_vs_ekf(n_frames=130):
    """
    Runs both estimators side by side with artificial heading noise added
    to simulate real IMU drift. Prints position error for each.
    """
    def step_kinematic(pos, heading, velocity, steering):
        new_pos = pos.copy()
        new_pos[0] += velocity * np.cos(heading) * DT
        new_pos[1] += velocity * np.sin(heading) * DT
        new_heading = angle_wrap(heading + (velocity / WHEELBASE) * np.tan(steering) * DT)
        return new_pos, new_heading
    np.random.seed(42)
    HEADING_NOISE = np.radians(0.8)   # 0.8 degrees of drift per step

    # --- ground truth: perfect physics, no noise ---
    true_pos  = CAR_START_POS.copy()
    true_head = CAR_START_HEADING

    # --- baseline: dead reckoning with heading drift ---
    dr_pos  = CAR_START_POS.copy()
    dr_head = CAR_START_HEADING

    # --- your EKF ---
    ekf = Solution()

    dr_errors  = []
    ekf_errors = []

    for _ in range(n_frames):
        steer = pure_pursuit(true_pos, true_head, CENTERLINE)

        # advance ground truth perfectly
        true_pos, true_head = step_kinematic(true_pos, true_head, SPEED, steer)

        # advance dead reckoning with a small random heading nudge each step
        dr_pos, dr_head = step_kinematic(dr_pos, dr_head, SPEED, steer)
        dr_head = angle_wrap(dr_head + np.random.normal(0, HEADING_NOISE))

        # advance EKF (it corrects itself using cone observations)
        ekf.localization(SPEED, steer)

        dr_errors.append(np.linalg.norm(dr_pos - true_pos))
        ekf_errors.append(np.linalg.norm(ekf.pos - true_pos))

    print(f"dead reckoning — mean error: {np.mean(dr_errors):.3f} m   "
          f"final error: {dr_errors[-1]:.3f} m")
    print(f"EKF            — mean error: {np.mean(ekf_errors):.3f} m   "
          f"final error: {ekf_errors[-1]:.3f} m")
# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=== Driverless Car Hackathon – SLAM Visualisation ===")
    print(f"  Blue cones   : {len(BLUE_CONES)}")
    print(f"  Yellow cones : {len(YELLOW_CONES)}")
    print(f"  Big orange   : {len(BIG_ORANGE)}")
    print(f"  Car start    : {CAR_START_POS}  "
          f"heading={np.degrees(CAR_START_HEADING):.1f}°")
    print(f"  Centerline   : {len(CENTERLINE)} waypoints (clockwise)")
    print("\nOpening 1 animation window …")

    # Keep references to prevent garbage collection of FuncAnimation objects.
    fig2, ani2 = make_problem2()
    compare_baseline_vs_ekf(n_frames=130)
    plt.show()