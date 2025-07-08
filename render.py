#!/usr/bin/python

# The goal of this project is to make a python renderer that can simulate drawing operations.
# This is for practice only.
# We will aim to simulate how lines, squares, and pixels are drawn on a grid.
# (the renderer can toggle between passthrough mode, where it uses the original drawing methods,
# and simulated mode, where it updates an RGB buffer instead)

import pygame
import math
import time
import sys
import ctypes
import numpy as np
# Make windows not scale this window (pixels do have to be perfect)
if sys.platform == "win32":
    ctypes.windll.user32.SetProcessDPIAware()

PASSTHROUGH = False # Toggle between passthrough and simulated draw
DRAW_PIXEL_BORDER = True  # Toggle to draw a border around pixels
PIXEL_BORDER_SIZE = 1  # Size of the pixel border

angle = 0

mouse_x = 0
mouse_y = 0

draw_z_buffer = False
draw_faces = True
draw_lines = False

from typing import List, Tuple
COLOR_BLACK = (0, 0, 0)
COLOR_RED = (255, 0, 0)
COLOR_GREEN = (0, 255, 0)
COLOR_BLUE = (0, 0, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_DARK_GRAY = (50, 50, 50)
COLOR_PINK = (255, 105, 180)

FRAME_LOG_INTERVAL = 60  # log once per 60 frames
frame_count = 0  # count frames rendered so far
_profile_timers = {}  # keep track of our named profilers

OBJ_PATH = "./models/teapot.obj"
CONUTER_CLOCKWISE_TRIANGLES = False
START_DISTANCE = 100.0

def profile_start(name: str, n=60):
    global frame_count
    if frame_count % n == 0:
        _profile_timers[name] = time.perf_counter()

def profile_end(name: str, n=60):
    global frame_count
    if frame_count % n == 0:
        if name in _profile_timers:
            elapsed = (time.perf_counter() - _profile_timers.pop(name)) * 1000
            print(f"{name}: {elapsed:.3f}ms")
        else:
            print(f"Warning: profile_end called for '{name}' without matching profile_start")

def timed(name="", n=60):
    def wrapper(fn):
        def inner(*args, **kwargs):
            global frame_count
            result = fn(*args, **kwargs)
            if frame_count % n == 0:
                start = time.perf_counter()
                fn(*args, **kwargs)
                print(f"{name or fn.__name__}: {(time.perf_counter() - start) * 1000:.3f}ms")
            return result
        return inner
    return wrapper

def load_obj(filepath):
    vertices = []
    triangles = []

    with open(filepath) as file:
        for line in file:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.strip().split()
            if parts[0] == 'v':
                vertices.append(tuple(map(float, parts[1:4])))
            elif parts[0] == 'f':
                face = [int(p.split('/')[0]) - 1 for p in parts[1:4]]
                triangles.append(tuple(face))

    return vertices, triangles

def normalize(v):
    return v / (np.linalg.norm(v) + 1e-16)

def get_projection_matrix(fov, aspect, near, far):
    f = 1 / np.tan(fov / 2)
    proj = np.zeros((4, 4))
    proj[0,0] = f / aspect
    proj[1,1] = f
    proj[2,2] = (far + near) / (near - far)
    proj[2,3] = (2 * far * near) / (near - far)
    proj[3,2] = -1
    return proj

def rotation_matrix_x(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1, 0, 0],
                    [0, c, -s],
                    [0, s,  c]])

def rotation_matrix_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[ c, 0, s],
                    [ 0, 1, 0],
                    [-s, 0, c]])
def rotation_matrix_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0],
                    [s,  c, 0],
                    [0,  0, 1]])

class Renderer:
    def __init__(self, width: int = 800, height: int = 800, grid_size: int = 200) -> None:
        pygame.init()
        self.width, self.height = width, height
        self.grid_size = grid_size
        self.cell_size = width // grid_size
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Renderer")
        self.clock = pygame.time.Clock()
        self.running = True

        # Initialize 2D RGB buffer
        self.rgb_buffer = np.zeros((self.grid_size, self.grid_size, 3), dtype=np.uint8)
        self.z_buffer = np.full((self.grid_size, self.grid_size), np.inf, dtype=np.float32)
        # Initialize 2D polygon buffer
        self.triangle_buffer = [
            [(12, 14), (18, 22), (15, 30)],
            [(20, 20), (30, 25), (25, 35)],
            [(40, 15), (45, 28), (35, 33)],
            [(60, 40), (70, 45), (65, 55)],
            [(50, 60), (55, 65), (45, 70)],
            [(30, 50), (35, 60), (25, 65)],
            [(80, 20), (85, 30), (75, 35)],
            [(15, 75), (20, 85), (10, 90)],
            [(70, 70), (80, 75), (75, 85)],
            [(90, 10), (95, 20), (85, 25)],
            [(65, 25), (70, 30), (60, 35)],
            [(22, 40), (28, 45), (24, 50)],
        ]
        
        utah_teapot = load_obj(OBJ_PATH)
        self.object = utah_teapot
        
        self.camera_pos = [0.0,0.0,-float(START_DISTANCE)]
        self.dragging = False
        self.last_mouse_pos = (0, 0)
        self.camera_rot = [0.0,0.0,0.0]
        self.camera_speed = 1.0
        self.projection_matrix = get_projection_matrix(fov=np.radians(90),aspect=1,near=0.1,far=1000)

        
    def _is_bounded(self, position: Tuple[int, int]) -> bool:
        x, y = position
        return 0 <= x < self.grid_size and 0 <= y < self.grid_size

    def draw_line(self, start: Tuple[int, int], end: Tuple[int, int], color: Tuple[int, int, int] = COLOR_RED, width: int = 1) -> None:
        if PASSTHROUGH:
            pygame.draw.line(self.screen, color, start, end, width)
            return

        self.bresenhams_algorithm_draw_line(start, end, color)

    # https://medium.com/geekculture/bresenhams-line-drawing-algorithm-2e0e953901b3
    def bresenhams_algorithm_draw_line(self, start, end, color):
        is_x_flipped = False  # for handling slope < 0
        is_x_dominant_axis = False  # for handling slope > 1
        
        x1 = start[0]
        x2 = end[0]
        y1 = start[1]
        y2 = end[1]
        
        # Handle x1 > x2 edge case (swap (x2, y2) and (x1, y1))
        if x1 > x2:
            # flip start and end
            tx, ty = x1, y1
            x1, y1 = x2, y2
            x2, y2 = tx, ty
        
        # Handle slope < 0
        if y2 - y1 < 0:
            # change (x1, y1) to (x1, -y1) and (x2, y2) to (x2, -y2)
            y1 *= -1
            y2 *= -1
            is_x_flipped = True
        
        # Handle slope > 1
        if y2 - y1 > x2 - x1:
            # exchange x and y values. So x1 becomes y1
            t2, t1 = x2, x1
            x2, x1 = y2, y1
            y2, y1 = t2, t1
            is_x_dominant_axis = True
        
        dx = (x2 - x1)
        dy = (y2 - y1)
        p = 2 * dy - dx
        x = x1
        y = y1
        
        while x <= x2:
            out_x = x
            out_y = y
            # undo "slope > 1" edge case
            if is_x_dominant_axis:
                tmp = out_x
                out_x = out_y
                out_y = tmp
            # undo "slope < 0" edge case
            out_y = out_y if not is_x_flipped else out_y * -1

            self.draw_pixel((out_x, out_y), color)
            x += 1
            if p < 0:
                p += 2 * dy
            else:
                p += 2 * (dy - dx)
                y += 1

    def draw_square(self, top_left: Tuple[int, int], size: int, color: Tuple[int, int, int] = COLOR_RED) -> None:
        if PASSTHROUGH:
            pygame.draw.rect(self.screen, color, (*top_left, size, size))
            return
        for y in range(top_left[1], top_left[1] + size):
            for x in range(top_left[0], top_left[0] + size):
                if self._is_bounded((x, y)):
                    self.rgb_buffer[y][x] = color

    def draw_pixel(self, position: Tuple[int, int], color: Tuple[int, int, int] = COLOR_RED) -> None:
        if PASSTHROUGH:
            self.screen.set_at(position, color)
            return
        x, y = position
        if self._is_bounded((x,y)):
            self.rgb_buffer[y][x] = color

    def draw_circle(self, center: Tuple[int, int], radius: int, color: Tuple[int, int, int] = COLOR_RED) -> None:
        if PASSTHROUGH:
            pygame.draw.circle(self.screen, color, center, radius)
            return

        sqrt_limit = radius**2
        for y in range(center[1] - radius, center[1] + radius):
            for x in range(center[0] - radius, center[0] + radius):
                if self._is_bounded((x, y)) and (y - center[1])**2 + (x - center[0])**2 < sqrt_limit:
                    self.rgb_buffer[y][x] = color

    # 500x500 triangles cost 0.8ms to draw (not great)
    def fill_triangle(self, p1: Tuple[float,float,float], p2: Tuple[float,float,float], p3: Tuple[float,float,float], color: Tuple[int, int, int] = COLOR_RED):
        if PASSTHROUGH:
            pygame.draw.polygon(self.screen, color, [p1, p2, p3])

        min_x = int(max(min(p1[0], p2[0], p3[0]), 0))
        max_x = int(min(max(p1[0], p2[0], p3[0]), self.grid_size - 1))
        min_y = int(max(min(p1[1], p2[1], p3[1]), 0))
        max_y = int(min(max(p1[1], p2[1], p3[1]), self.grid_size - 1))
        
        # Compute edge coefficients for: E(x, y) = A*x + B*y + C
        def edge_coeffs(p1, p2):
            A = p1[1] - p2[1]
            B = p2[0] - p1[0]
            C = p1[0]*p2[1] - p1[1]*p2[0]
            return A, B, C

        A0, B0, C0 = edge_coeffs(p2, p3)
        A1, B1, C1 = edge_coeffs(p3, p1)
        A2, B2, C2 = edge_coeffs(p1, p2)

        for y in range(min_y, max_y + 1):
            # Start x from min_x
            w0 = A0 * min_x + B0 * y + C0
            w1 = A1 * min_x + B1 * y + C1
            w2 = A2 * min_x + B2 * y + C2

            # Precompute deltas
            dw0_dx = A0
            dw1_dx = A1
            dw2_dx = A2

            def edge(a, b, c):
                # Returns twice the signed area of triangle abc
                return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0])
            area = edge(p1, p2, p3)  # Precompute this outside the loop
            if area == 0:
                return  # skip degenerate triangle
            
            for x in range(min_x, max_x + 1):
                # Accept both types of faces. Can be optimized if only one is supported
                if (w0 >= 0 and w1 >= 0 and w2 >= 0) or (w0 <= 0 and w1 <= 0 and w2 <= 0):
                    # Normalize barycentric coordinates
                    alpha = w0 / area
                    beta = w1 / area
                    gamma = w2 / area

                    # Interpolate Z
                    z = alpha * p1[2] + beta * p2[2] + gamma * p3[2]
                    if z < self.z_buffer[y, x]:
                        self.rgb_buffer[y, x] = color
                        # Do a dumb interpolation (won't work for different perspectives)
                        self.z_buffer[y, x] = z
                w0 += dw0_dx
                w1 += dw1_dx
                w2 += dw2_dx


    def draw_triangle(self, p1: Tuple[int,int], p2: Tuple[int, int], p3: Tuple[int, int], color: Tuple[int, int, int] = COLOR_WHITE):
        if PASSTHROUGH:
            pygame.draw.lines(self.screen, color, True, [p1, p2, p3])
            return
        self.draw_line(p1, p2, color)
        self.draw_line(p2, p3, color)
        self.draw_line(p3, p1, color)
            
    @timed()
    def draw_polygons(self, scale=1, offset=(0, 0)):
        # === Setup ===
        light = np.array([0, -1, 0])
        camera_direction = np.array([0, 0, 1])  # assuming camera looks along +Z

        global angle
        Rx = rotation_matrix_x(angle)
        Ry = rotation_matrix_y(angle)
        R = Ry @ Rx
        angle += 0.01
        
        pitch, yaw, roll = self.camera_rot
        Rx = rotation_matrix_x(pitch)
        Ry = rotation_matrix_y(yaw)
        Rz = rotation_matrix_z(roll)
        R_cam = Rz @ Ry @ Rx  # camera rotation
        R_view = R_cam.T  # inverse of rotation matrix is transpose

        for face_index in self.object[1]:
            # === World-space triangle ===
            tri_world = [np.array(self.object[0][i]) for i in face_index]

            # === Rotate object ===
            tri_rotated = [R @ v for v in tri_world]
            
            # === View transform: apply inverse camera rotation and translation ===
            tri_camera = [R_view @ (v - self.camera_pos) for v in tri_rotated]

            # === Translate by camera position (view transform: inverse translation) ===
            # tri_camera = [v - self.camera_pos for v in tri_rotated]

            # === Compute face normal and backface culling ===
            a = tri_camera[1] - tri_camera[0]
            b = tri_camera[2] - tri_camera[0]
            normal = normalize(np.cross(a, b))

            if np.dot(normal, camera_direction) > 0 != CONUTER_CLOCKWISE_TRIANGLES:
                continue  # Cull

            # === Lighting ===
            brightness = max(0, (np.dot(normal, light) + 1) / 2)
            color = tuple(int(brightness * c) for c in COLOR_WHITE)

            # === Project ===
            tri_homogeneous = [np.append(v, 1) for v in tri_camera]
            tri_projected = [self.projection_matrix @ v for v in tri_homogeneous]
            tri_ndc = [v[:3] / v[3] for v in tri_projected]  # NDC space
            
            # === Frustum culling ===
            if all(v[2] <= 0 for v in tri_camera):
                continue  # All behind camera

            # === Convert to screen space ===
            tri_screen = [
                (
                    int((v[0] + 1) * 0.5 * self.grid_size * scale + offset[0]),
                    int((1 - (v[1] + 1) * 0.5) * self.grid_size * scale + offset[1]),
                    v[2]  # keep depth
                )
                for v in tri_ndc
            ]
            if draw_faces or draw_z_buffer:
                self.fill_triangle(tri_screen[0], tri_screen[1], tri_screen[2], color) # type: ignore
            if draw_lines:
                self.draw_triangle(tri_screen[0][0:2], tri_screen[1][0:2], tri_screen[2][0:2], COLOR_GREEN)

            

    @timed("render_buffer")
    def render_buffer(self):
        global draw_z_buffer  # Toggle this to enable/disable Z buffer debug view
        # DEBUG_Z_MIN = -100  # how close we can see
        # DEBUG_Z_MAX = 100 # how far we can see
        if draw_z_buffer:
            # Normalize Z buffer to [0, 255] for display
            finite_z = self.z_buffer[np.isfinite(self.z_buffer)]
            if finite_z.size == 0:
                return  # Nothing to render

            z_min, z_max = finite_z.min(), finite_z.max()
            if z_min == z_max:
                z_max += 1e-5  # Prevent divide by zero

            z_norm = ((z_max - self.z_buffer) / (z_max - z_min) * 205 + 50).astype(np.uint8)
            z_norm[~np.isfinite(self.z_buffer)] = 0  # Set infs to black
            z_gray = np.stack([z_norm] * 3, axis=-1)
            surface = pygame.surfarray.make_surface(z_gray.swapaxes(0, 1))
        else:
            surface = pygame.surfarray.make_surface(self.rgb_buffer.swapaxes(0, 1))
        surface = pygame.transform.scale(surface, (self.width, self.height))
        self.screen.blit(surface, (0, 0))

        if DRAW_PIXEL_BORDER:
            pixel_border_color = COLOR_DARK_GRAY
            for x in range(self.grid_size):
                pygame.draw.line(self.screen, pixel_border_color, (x * self.cell_size, 0), (x * self.cell_size, self.height), PIXEL_BORDER_SIZE)
            for y in range(self.grid_size):
                pygame.draw.line(self.screen, pixel_border_color, (0, y * self.cell_size), (self.width, y * self.cell_size), PIXEL_BORDER_SIZE)

    def run(self):
        
        while self.running:
            global frame_count
            frame_count += 1  # start of the next frame
            self.screen.fill((0, 0, 0))

            # Drawing demo here
            if True:
                # self.draw_line((0, 0), (self.width, self.height), (255, 0, 0))
                # self.draw_square((50, 50), 100, (0, 255, 0))
                # self.draw_pixel((0, 0), COLOR_RED)
                # self.draw_pixel((1, 1), COLOR_BLUE)
                # self.draw_pixel((1, 0), COLOR_GREEN)
                # self.draw_line((6, 6), (11, 8), COLOR_GREEN)
                # self.draw_line((2, 2), (5, 5), COLOR_WHITE)
                # self.draw_square((10,10), 5, COLOR_WHITE)
                # self.draw_circle((20, 8), 8, COLOR_GREEN)
                # self.fill_triangle((1*2, 12*2), (8*2, 9*2), (18*2, 15*2), COLOR_RED)
                # self.draw_polygons_2d()
                self.draw_polygons()
                
                # Spinning line (10px long from center)
                # cx, cy = self.grid_size // 2, self.grid_size // 2
                # length = 20
                # global angle
                # x2 = int(cx + length * math.cos(angle))
                # y2 = int(cy + length * math.sin(angle))
                # self.draw_line((cx, cy), (x2, y2), COLOR_WHITE)
                
                # angle += 0.01
                
                # Polygon follows mouse, left click cycles which point is moved
                if not hasattr(self, "poly_points"):
                    cx, cy = self.grid_size // 2, self.grid_size // 2
                    self.poly_points = [
                        [cx - 10, cy - 5],
                        [cx + 10, cy - 5],
                        [cx, cy + 10]
                    ]
                    self.active_point = 0

                mx, my = pygame.mouse.get_pos()
                
                mx //= self.cell_size
                my //= self.cell_size

                global mouse_x
                mouse_x = mx
                global mouse_y
                mouse_y = my
                
                # Move the active point to mouse position
                # self.poly_points[self.active_point][0] = mx
                # self.poly_points[self.active_point][1] = my

                # # Draw the polygon
                # self.fill_triangle(
                #     (self.poly_points[0][0], self.poly_points[0][1]),
                #     (self.poly_points[1][0], self.poly_points[1][1]),
                #     (self.poly_points[2][0], self.poly_points[2][1]),
                #     COLOR_WHITE
                # )

                # # Draw points for visual feedback
                # for idx, pt in enumerate(self.poly_points):
                #     color = COLOR_RED if idx == self.active_point else COLOR_BLUE
                #     self.draw_square((pt[0], pt[1]), 2, color)

                # Handle mouse click to cycle active point
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        if event.button == 1:  # Left click
                            # self.active_point = (self.active_point + 1) % 3
                            print(self.poly_points)
                    # ====== Camera rotation stuff ======
                    if event.type == pygame.MOUSEBUTTONDOWN:
                            if event.button == 1:  # Left click
                                self.dragging = True
                                self.last_mouse_pos = pygame.mouse.get_pos()
                    elif event.type == pygame.MOUSEBUTTONUP:
                        if event.button == 1:
                            self.dragging = False
                            print(f"Camera rotated with new vector ({self.camera_rot[0]}, {self.camera_rot[1]}, {self.camera_rot[2]})")
                    elif event.type == pygame.MOUSEMOTION and self.dragging:
                        x, y = pygame.mouse.get_pos()
                        dx = x - self.last_mouse_pos[0]
                        dy = y - self.last_mouse_pos[1]
                        self.last_mouse_pos = (x, y)

                        self.camera_rot[1] -= dx * 0.005  # yaw (Y axis)
                        self.camera_rot[0] -= dy * 0.005  # pitch (X axis)
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_z:
                        global draw_z_buffer
                        draw_z_buffer = not draw_z_buffer
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_v:
                        global draw_lines
                        draw_lines = not draw_lines
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_f:
                        global draw_faces
                        draw_faces = not draw_faces
                keys = pygame.key.get_pressed()
                move_dir = np.array([0.0, 0.0, 0.0])
                if keys[pygame.K_LSHIFT]: 
                    move_boost = 5
                else: 
                    move_boost = 1
                if keys[pygame.K_w]: move_dir[2] += 1
                if keys[pygame.K_s]: move_dir[2] -= 1
                if keys[pygame.K_a]: move_dir[0] += 1
                if keys[pygame.K_d]: move_dir[0] -= 1
                if keys[pygame.K_SPACE]: move_dir[1] -= 1
                if keys[pygame.K_c]: move_dir[1] += 1

                if np.linalg.norm(move_dir) > 0:
                    # move_dir = move_dir / np.linalg.norm(move_dir) * self.camera_speed
                    move_dir = move_dir / np.linalg.norm(move_dir) * self.camera_speed * move_boost

                    # Camera rotation to world space
                    pitch, yaw, roll = self.camera_rot
                    Rx = rotation_matrix_x(pitch)
                    Ry = rotation_matrix_y(yaw)
                    Rz = rotation_matrix_z(roll)
                    R_cam = Rz @ Ry @ Rx
                    move_world = R_cam @ move_dir
                    self.camera_pos += move_world

            if not PASSTHROUGH:
                self.render_buffer()
            
            # Clear the RGB buffer for the next frame
            self.rgb_buffer.fill(0)
            self.z_buffer.fill(np.inf)

            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()

if __name__ == "__main__":
    renderer = Renderer()
    renderer.run()

# NOTE ChatGPT (4o for general templates) and Copilot (internally using GPT-4.1) was used in this project.
