import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from matplotlib.widgets import RadioButtons, Button


# ----------------------------------------------------------------------------
# Parameters (toy planet so that the rocket is visible on screen)
# ----------------------------------------------------------------------------
R_PLANET = 1000.0            # m
G0 = 9.81                    # m/s^2 at the surface
GM = G0 * R_PLANET**2

MASS = 100.0                 # kg (constant, unlimited fuel -> TODO: variable mass)
THRUST = 3500.0              # N
A_MAX = THRUST / MASS        # max thrust acceleration (m/s^2)

CD, AREA = 0.5, 0.05         # drag coefficient, reference area (m^2)
RHO0, H_SCALE = 1.2, 80.0    # sea-level density (kg/m^3), scale height (m) -> toy values

# Flight plan
BURN_1D = 3.0                # s, burn duration in 1D mode
H_VERT = 40.0                # m, vertical ascent before starting to tilt (2D / 3D)
H_ORB = 300.0                # m, orbit altitude (2D / 3D)
D_LAND = 500.0               # m, remaining arc to the pad when landing guidance starts
A_BRAKE, V_LAND_MAX, V_TOUCH = 6.0, 80.0, 0.5   # landing speed profile (m/s^2, m/s, m/s)

DT = 0.005                   # s, fixed physics step
STEPS_PER_FRAME = 20         # physics steps per animation frame (speed of the animation)

# pressure at the floor
P_FLOOR = RHO0 * G0 * H_SCALE

# ----------------------------------------------------------------------------
# Physics + simple flight plan
# ----------------------------------------------------------------------------
class RocketSim:
    """Vector (3D) physics. 1D and 2D are the same physics with constraints."""

    def __init__(self, mode="1D"):
        self.mode = mode
        self.t = 0.0
        self.pos = np.array([0.0, 0.0, R_PLANET])   # launch pad on top of the planet
        self.vel = np.zeros(3)
        self.pad = self.pos.copy()
        self.landed = False
        self.phase = "ASCENT"

        # Orbital plane: contains the z axis and the direction e_dir
        self.ez = np.array([0.0, 0.0, 1.0])
        self.e_dir = {"1D": np.array([1.0, 0.0, 0.0]),
                      "2D": np.array([1.0, 0.0, 0.0]),
                      "3D": np.array([0.8, 0.6, 0.0])}[mode]
        self.normal = np.cross(self.ez, self.e_dir)
        self.theta = 0.0                 # angle travelled around the planet (rad)
        self._theta_prev = 0.0

        self.thrust_acc = np.zeros(3)
        self.drag_acc = np.zeros(3)
        self.touchdown_speed = None
        self.landing_error = None

    # --- mode constraints ---
    def constrain(self, v):
        v = v.copy()
        if self.mode == "1D":
            v[0] = v[1] = 0.0          # radial (z) motion only
        elif self.mode == "2D":
            v[1] = 0.0                 # x-z plane only
        return v

    # --- flight phases ---
    def update_phase(self):
        r = np.linalg.norm(self.pos)
        er = self.pos / r
        vr = self.vel @ er # for matrix multiplication
        h = r - R_PLANET

        # Angle travelled around the planet (unwrapped)
        th = np.arctan2(self.pos @ self.e_dir, self.pos @ self.ez)
        self.theta += (th - self._theta_prev + np.pi) % (2 * np.pi) - np.pi
        self._theta_prev = th

        if self.mode == "1D":
            if self.phase == "ASCENT" and self.t >= BURN_1D:
                self.phase = "COAST"
            elif self.phase == "COAST" and vr < 0:
                self.phase = "LANDING"
        else:
            vt = self.vel @ np.cross(self.normal, er)
            vc = np.sqrt(GM / (R_PLANET + H_ORB))
            if self.phase == "ASCENT" and h >= H_VERT:
                self.phase = "INSERTION"
            elif (self.phase == "INSERTION" and abs(h - H_ORB) < 10
                  and abs(vr) < 2 and abs(vt - vc) < 2):
                self.phase = "ORBIT"
            elif self.phase == "ORBIT" and (2 * np.pi - self.theta) * r <= D_LAND:
                self.phase = "LANDING"

    def guidance(self):
        """Thrust acceleration (world frame) commanded by the simulated autopilot."""
        r = np.linalg.norm(self.pos)
        er = self.pos / r

        if self.phase == "ASCENT":
            return A_MAX * er

        if self.phase in ("INSERTION", "ORBIT"):
            et = np.cross(self.normal, er)
            vr, vt = self.vel @ er, self.vel @ et
            h = r - R_PLANET
            vc = np.sqrt(GM / (R_PLANET + H_ORB))
            # Radial: reach and hold the orbit altitude (gravity and centrifugal compensated)
            vr_des = np.clip(0.5 * (H_ORB - h), -40.0, 40.0)
            ar = 2.0 * (vr_des - vr) + GM / r**2 - vt**2 / r
            ar = np.clip(ar, -0.6 * A_MAX, 0.6 * A_MAX)
            # Tangential: reach and hold circular speed
            at_lim = np.sqrt(max(A_MAX**2 - ar**2, 0.0))
            at = np.clip(10.0 * (vc - vt), -at_lim, at_lim)
            return ar * er + at * et

        if self.phase == "LANDING":
            d = self.pad - self.pos
            dist = np.linalg.norm(d)
            v_mag = min(V_LAND_MAX, np.sqrt(2 * A_BRAKE * dist) + V_TOUCH)
            v_des = v_mag * d / dist if dist > 1e-6 else np.zeros(3)
            a = 5.0 * (v_des - self.vel) + GM * self.pos / r**3   # + gravity compensation
            n = np.linalg.norm(a)
            return a if n <= A_MAX else a * (A_MAX / n)

        return np.zeros(3)    # COAST / LANDED

    def acceleration(self):
        r = np.linalg.norm(self.pos)
        gravity = -GM * self.pos / r**3                       # g(h) = GM / r^2

        self.thrust_acc = self.constrain(self.guidance())

        h = r - R_PLANET                                      # drag: 0.5 * rho * Cd * A * v^2
        rho = RHO0 * np.exp(-h / H_SCALE)
        speed = np.linalg.norm(self.vel)
        drag = -0.5 * rho * CD * AREA * speed * self.vel / MASS
        self.drag_acc = self.constrain(drag)

        return self.constrain(gravity) + self.thrust_acc + self.drag_acc

    def step(self, dt=DT):
        if self.landed:
            return
        self.update_phase()
        a = self.acceleration()
        self.vel += a * dt                  # semi-implicit Euler  (TODO: RK4)
        self.pos += self.vel * dt
        self.t += dt
        # if the rocket is landed
        if np.linalg.norm(self.pos) <= R_PLANET and self.t > 0.1:
            # velocity at the moment of touchdown
            self.touchdown_speed = np.linalg.norm(self.vel)
            # position at the moment of touchdown
            self.pos *= R_PLANET / np.linalg.norm(self.pos)
            # landing error is the distance from the pad
            self.landing_error = np.linalg.norm(self.pos - self.pad)
            # stop the rocket
            self.vel[:] = 0.0
            self.landed = True
            self.phase = "LANDED"

    def altitude(self):
        return np.linalg.norm(self.pos) - R_PLANET

    def specific_force(self):
        """What an accelerometer would measure (everything except gravity)."""
        return self.thrust_acc + self.drag_acc


# ----------------------------------------------------------------------------
# Sensors 
# ----------------------------------------------------------------------------
def sensors(sim):
    """Turn the truth into noisy raw measurements.The flight software must ONLY receive this, never the truth.
    In this simulation sensors are accelerometers, gyroscopes, barometers, GPS, magnetometers."""

    '''
    # we retrieve the real datas if the sensors were perfect
    accelerometer = sim.specific_force()
    
    r = np.linalg.norm(self.pos)
    h = r - R_PLANET    
    barometers = sim.altitude() + P_FLOOR * exp(-h / H_SCALE)

    gps = { pos : sim.pos , vel : sim.vel }

    
    gyroscopes = 
    magnetometers = 

    # now we add the noise and the bias to each sensor to simulate real sensors


    return {}

    '''
    
    return None


# ----------------------------------------------------------------------------
# Link to flight_sw (TODO)
# ----------------------------------------------------------------------------
class FlightLink:
    """Placeholder. Later: TCP client sending measurements to the C program
    and returning its estimate. For now it just returns the truth."""

    
    def step(self, sim, measurements):
        # TODO: replace with the real reply from flight_sw
        return {
            "pos": sim.pos.copy(),
            "vel": sim.vel.copy(),
            "alt_uncertainty": 0.0,      # estimated uncertainty (m)
            "vel_uncertainty": 0.0,      # estimated uncertainty (m/s)
        }


# ----------------------------------------------------------------------------
# Visualization
# ----------------------------------------------------------------------------
BG = "#111111"
RECT = [0.2, 0.05, 0.78, 0.88]
PANEL_W = 0.25

class App:

    def __init__(self):
        self.fig = plt.figure(figsize=(9, 7))
        self.fig.patch.set_facecolor(BG)
        self.fig.canvas.manager.set_window_title("Rocket GNC - simulation")
        # align on the left of the window
        self.info = self.fig.text(RECT[0], 0.95, "", color="white", family="monospace")
        lim = R_PLANET * 1.6

        # --- flat view (1D and 2D): planet = a circle ---
        ax2 = self.fig.add_axes(RECT)
        ax2.set_facecolor("black")
        ax2.set_xlim(-lim, lim)
        ax2.set_ylim(-lim, lim)
        ax2.set_aspect("equal")
        ax2.tick_params(colors="gray")
        ax2.set_xlabel("x (m)", color="gray")
        ax2.set_ylabel("z (m)", color="gray")
        ax2.add_patch(Circle((0, 0), R_PLANET, fill=False, ec="#3a6ea5", lw=2))
        ax2.plot([0], [R_PLANET], "s", color="lime", ms=5)          # launch pad
        self.v2 = self.make_lines(ax2, three_d=False)

        # --- 3D view: planet = a sphere ---
        ax3 = self.fig.add_axes(RECT, projection="3d")
        ax3.set_facecolor("black")
        ax3.set_xlim(-lim, lim)
        ax3.set_ylim(-lim, lim)
        ax3.set_zlim(-lim, lim)
        ax3.set_box_aspect((1, 1, 1))
        ax3.tick_params(colors="gray")
        ax3.set_xlabel("x (m)")
        ax3.set_ylabel("y (m)")
        ax3.set_zlabel("z (m)")
        u = np.linspace(0, 2 * np.pi, 40)
        v = np.linspace(0, np.pi, 20)
        ax3.plot_wireframe(R_PLANET * np.outer(np.cos(u), np.sin(v)),
                           R_PLANET * np.outer(np.sin(u), np.sin(v)),
                           R_PLANET * np.outer(np.ones_like(u), np.cos(v)),
                           color="#3a6ea5", linewidth=0.3, alpha=0.5)
        ax3.plot([0], [0], [R_PLANET], "s", color="lime", ms=5)     # launch pad
        self.v3 = self.make_lines(ax3, three_d=True)

        self.info = self.fig.text(0.2, 0.95, "", color="white", family="monospace")

        # --- mode selector ---
        rax = self.fig.add_axes([0.02, 0.75, 0.13, 0.15], facecolor="#222222")
        self.radio = RadioButtons(rax, ("1D", "2D", "3D"), active=0)
        for lbl in self.radio.labels:
            lbl.set_color("white")
        self.radio.on_clicked(self.reset)

        # add the connection to the flight software
        self.link = FlightLink()

        # add pause button
        self.paused = False
        bax = self.fig.add_axes([0.88, 0.93, 0.10, 0.05])
        self.btn_pause = Button(bax, "Pause", color="#333333", hovercolor="#555555")
        self.btn_pause.label.set_color("white")
        self.btn_pause.label.set_fontsize(11)
        self.btn_pause.on_clicked(self.toggle_pause) # call our function toggle_pause

        self.reset("1D")
        self.anim = FuncAnimation(self.fig, self.update, interval=30, blit=False,cache_frame_data=False)

        # add panel for truth, sensors and estimate datas
        self.txt_truth    = self.make_panel([0.01, 0.46, PANEL_W, 0.26], "TRUTH (simulator)", "white")
        self.txt_sensors  = self.make_panel([0.01, 0.25, PANEL_W, 0.19], "SENSORS -> flight_sw", "orange")
        self.txt_estimate = self.make_panel([0.01, 0.04, PANEL_W, 0.19], "flight_sw -> ESTIMATE", "red")

    @staticmethod
    def make_lines(ax, three_d):
        """Trails and dots: white = truth, red = estimate."""
        empty = ([], [], []) if three_d else ([], [])
        (tt,) = ax.plot(*empty, color="white", lw=1, alpha=0.6)
        (te,) = ax.plot(*empty, color="red", lw=1, alpha=0.6)
        (dt_,) = ax.plot(*empty, "o", color="white", ms=6)
        (de,) = ax.plot(*empty, "o", color="red", ms=4)
        return dict(ax=ax, three_d=three_d, trail_true=tt, trail_estimate=te,
                    dot_true=dt_, dot_estimate=de)

    def reset(self, mode):
        # turn off the pause if it was on
        self.paused = False
        self.btn_pause.label.set_text("Pause")

        # reset the simulation
        self.sim = RocketSim(mode)
        self.true_pts, self.estimate_pts = [], []
        self.done = False
        flat = mode != "3D"
        self.view = self.v2 if flat else self.v3
        self.v2["ax"].set_visible(flat)
        self.v3["ax"].set_visible(not flat)
        for v in (self.v2, self.v3):
            for k in ("trail_true", "trail_estimate", "dot_true", "dot_estimate"):
                v[k].set_data([], [])
                if v["three_d"]:
                    v[k].set_3d_properties([])

    def draw_points(self, view, pts, trail, dot):
        arr = np.array(pts)
        if view["three_d"]:
            view[trail].set_data(arr[:, 0], arr[:, 1])
            view[trail].set_3d_properties(arr[:, 2])
            view[dot].set_data([arr[-1, 0]], [arr[-1, 1]])
            view[dot].set_3d_properties([arr[-1, 2]])
        else:
            view[trail].set_data(arr[:, 0], arr[:, 2])      # x-z plane
            view[dot].set_data([arr[-1, 0]], [arr[-1, 2]])

    def make_panel(self, rect, title, color):
        """Rectangle with a title and a text area. Returns the text object to update."""
        ax = self.fig.add_axes(rect, facecolor="#1a1a1a")
        ax.set_xticks([])
        ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_color(color)
        ax.text(0.04, 0.96, title, color=color, fontsize=11, fontweight="bold",va="top", family="monospace", transform=ax.transAxes)
        return ax.text(0.04, 0.80, "", color="white", fontsize=10, va="top",family="monospace", transform=ax.transAxes)
    
    def toggle_pause(self, _event):
        self.paused = not self.paused
        self.btn_pause.label.set_text("Resume" if self.paused else "Pause")
        self.fig.canvas.draw_idle()
    
    def format_estimate(self, s, est, vr_true):
        """Text of panel 3, depending on the mode. 'error' is computed here:
        flight_sw never knows the truth."""
        pos, vel = est["pos"], est["vel"]
        r_est = np.linalg.norm(pos)
        alt_est = r_est - R_PLANET
        vr_est = vel @ (pos / r_est)

        if s.mode == "1D":
            return (
                f"altitude  {alt_est:8.1f} m\n"
                f"velocity  {vr_est:8.1f} m/s\n"
                f"altitude_uncertainty {est['alt_uncertainty']:8.2f} m\n"
                f"velocity_uncertainty {est['vel_uncertainty']:8.2f} m/s\n"
                f"error_altitude   {alt_est - s.altitude():8.2f} m\n"
                f"error_velocity   {vr_est - vr_true:8.2f} m/s"
            )

        err_pos = np.linalg.norm(pos - s.pos)

        if s.mode == "2D":
            return (
                f"position_x     {pos[0]:8.1f} m\n"
                f"position_z     {pos[2]:8.1f} m\n"
                f"velocity_x     {vel[0]:8.1f} m/s\n"
                f"velocity_z     {vel[2]:8.1f} m/s\n"
                f"altitude  {alt_est:8.1f} m\n"
                f"error_position   {err_pos:8.2f} m"
            )

        return (  # 3D
            f"position_x     {pos[0]:8.1f} m\n"
            f"position_y     {pos[1]:8.1f} m\n"
            f"position_z     {pos[2]:8.1f} m\n"
            f"velocity_x     {vel[0]:8.1f} m/s\n"
            f"velocity_y     {vel[1]:8.1f} m/s\n"
            f"velocity_z     {vel[2]:8.1f} m/s\n"
            f"altitude  {alt_est:8.1f} m\n"
            f"error_position   {err_pos:8.2f} m"
        )
    
    def format_truth(self, s, vr_true):
        """Text of panel 1 (true state). Same labels and same order as the
        estimate panel, so that both can be compared line by line."""
        pos, vel = s.pos, s.vel

        if s.mode == "1D":
            return (
                f"altitude  {s.altitude():8.1f} m\n"
                f"velocity  {vr_true:8.1f} m/s"
            )

        if s.mode == "2D":
            return (
                f"position_x     {pos[0]:8.1f} m\n"
                f"position_z     {pos[2]:8.1f} m\n"
                f"velocity_x     {vel[0]:8.1f} m/s\n"
                f"velocity_z     {vel[2]:8.1f} m/s\n"
                f"altitude  {s.altitude():8.1f} m"
            )

        return (  # 3D
            f"position_x     {pos[0]:8.1f} m\n"
            f"position_y     {pos[1]:8.1f} m\n"
            f"position_z     {pos[2]:8.1f} m\n"
            f"velocity_x     {vel[0]:8.1f} m/s\n"
            f"velocity_y     {vel[1]:8.1f} m/s\n"
            f"velocity_z     {vel[2]:8.1f} m/s\n"
            f"altitude  {s.altitude():8.1f} m"
        )
    
    def update(self, _frame):
        # if the animation is over or in pause do nothing
        if self.done or self.paused:
            return
        # else for each frame
        for _ in range(STEPS_PER_FRAME):
            # do the physics step, get the measurements and the estimate
            self.sim.step()
            # make the measurements from the sensors
            measurements = sensors(self.sim)
            # make the estimate from the flight software
            estimate = self.link.step(self.sim, measurements)
            
        
        # update the visualization (difference between the truth and the estimate)
        self.true_pts.append(self.sim.pos.copy())
        self.estimate_pts.append(estimate["pos"].copy())
        self.draw_points(self.view, self.true_pts, "trail_true", "dot_true")
        self.draw_points(self.view, self.estimate_pts, "trail_estimate", "dot_estimate")

        s = self.sim
        r = np.linalg.norm(s.pos)
        # vr is the radial velocity (positive = away from the planet)
        vr = s.vel @ (s.pos / r)

        # Top text
        text = (f"mode {s.mode} | {s.phase:9s} | t = {s.t:6.1f} s | alt = {s.altitude():7.1f} m"f" | lap = {s.theta / (2 * np.pi) * 100:5.1f} %")

        # if the rocket has landed
        if s.landed:
            # modify the main text and set the done flag to True
            text += f"\nLANDED: {s.landing_error:.1f} m from pad, touchdown {s.touchdown_speed:.1f} m/s"
            self.done = True

        # update the main text
        self.info.set_text(text)

        # panel 1: true physical state (depends on the mode)
        self.txt_truth.set_text(self.format_truth(s, vr))

        # panel 2: raw sensor data sent to flight_sw
        if measurements:
            self.txt_sensors.set_text("\n".join(f"{k:<8s} {v:10.3f}" for k, v in measurements.items()))
        else:
            self.txt_sensors.set_text("(no sensor model yet)")

        # panel 3: values returned by flight_sw (depends on the mode)
        self.txt_estimate.set_text(self.format_estimate(s, estimate, vr))

    
if __name__ == "__main__":
    app = App()
    plt.show()
   