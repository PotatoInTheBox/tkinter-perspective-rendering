
""" File: ivans-graphics.py
    Author: Ivan Gusachenko
    Date: 2/1/2022
    Purpose: Renders a 3d object. A few example settings can be configured
        below in the constant variables area.
"""


from math import sin, cos, tan, pi, sqrt
import graphics
import time
import os

# Mostly follows https://www.youtube.com/watch?v=ih20l3pJoeU

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# There are four variable constants below named
# RENDER_FILE, DISTANCE_TO_OBJECT, CLOCKWISE_POLYGONS, and DEBUG_TRIANGLES
# for easy modification.

# Opens and renders the specified file in the same directory
# the obj file should only have v (verticies with 3 floats)
# and f (faces with 3 floats).
# The faces must have 3 indecies seperated by spaces (eg. 0 1 2).
# 3 indecies because only triangles are supported (no quads).
# Default options are
# "ship.obj",
# "name.obj", and
# "teapot.obj".
RENDER_FILE = "./models/teapot.obj"

# DISTANCE_TO_OBJECT modifies how far the object is from the "camera".
# "Camera" in quotations because it isn't fully implemented.
# There is no clipping when triangles are behind the camera.
# Therefore when an object gets too close it gets drawn very incorrectly.
# Defaults are...
# ship.obj ->     6 distance
# name.obj ->     6 distance
# teapot.obj -> 100 distance (https://people.sc.fsu.edu/~jburkardt/)
DISTANCE_TO_OBJECT = 100

# Triangles faces are determined by whether they are clockwise or not.
# By default clockwise triangles are rendered while counterclockwise
# triangles get skipped. Clockwise/counterclockwise triangles can be
# flipped by alternating the CLOCKWISE_POLYGONS variable.
# ship.obj ->   True (clockwise)
# name.obj ->   True (clockwise)
# teapot.obj -> False (counterclockwise)
CLOCKWISE_POLYGONS = False

# It is also possible to show the triangle wireframe.
DEBUG_TRIANGLES = False


def main():
    # set directory for easier .obj file selection
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    screen_w = 800
    screen_h = 600
    win = graphics.graphics(screen_w, screen_h, "Rendering shape")

    r_scene = SceneRender()
    r_scene.screen_width = screen_w
    r_scene.screen_height = screen_h

    # 0 prioritizes previous fps, 1 prioritizes next fps
    avarage_fps = 0
    avarage_fps_weight = 0.1

    while not win.is_destroyed():
        start_time = time.time()
        win.clear()

        draw_object(win, r_scene)
        win.canvas.create_text(10, 10, text=int(avarage_fps))
        win.update()  # render as many frames as you can!

        r_scene.calculate_rotation_matrix()
        tmp = (time.time() - start_time)
        if tmp != 0:
            avarage_fps += (avarage_fps_weight *
                            ((1.0 / tmp) - avarage_fps))


# Really inefficient in Python (should be a construct) but for the sake of
# code clarity, classes are needed.
class Matrix4:
    """A 4 by 4 matrix (4 lists containing 4 numbers each)"""

    def __init__(self):
        self.matrix_array = [[0]*4 for i in range(4)]

    def __getitem__(self, index):
        return self.matrix_array[index]

    def zero_array(self):
        """Clear itself to all 0's"""
        self.matrix_array = [[0]*4 for i in range(4)]

    def dot_vector(self, vector):
        """Perform the multiply_matrix_vector() with self and another
        vector. Return a new multiplied vector (dot product).
        """
        return multiply_matrix_vector(vector, self.matrix_array)


class SceneRender():
    """Scene Renderer holding the necessary rendering data."""
    def __init__(self):
        self.screen_height = 600
        self.screen_width = 800

        self.my_obj = None
        self.matProj = None

        self.matRotX = None
        self.matRotZ = None
        self.f_theta = 0

        self.is_counter_clockwise = not CLOCKWISE_POLYGONS
        self.distance = DISTANCE_TO_OBJECT
        self.v_camera = [0.0, 0.0, 0.0]
        self.light_direction = [0.0, 0.0, -1.0]
        # also normalize the light
        l = sqrt(self.light_direction[0]**2 + self.light_direction[1]**2 +
                 self.light_direction[2]**2)
        if l != 0:
            self.light_direction[0] /= l
            self.light_direction[1] /= l
            self.light_direction[2] /= l

        # begin loading
        self.load_obj()
        self.calculate_projection_matrix()
        self.calculate_rotation_matrix()

    def load_obj(self):
        """Load .obj file into an array of triangles in my_obj."""
        load_location = RENDER_FILE
        vertecies = []
        triangles = []

        obj_file = open(load_location)
        obj_lines = obj_file.readlines()

        for line in obj_lines:
            if line.startswith('v '):
                vertecies.append([float(n) for n in line[1:].split()])
            if line.startswith('f '):
                triangles.append(
                    [tuple(vertecies[int(i)-1]) for i in line[1:].split()])

        obj_file.close()
        self.my_obj = tuple(triangles)  # tuple is a little faster than list


    def calculate_projection_matrix(self):
        """Generate the matrix for mapping cordinates to a 2d screen."""
        # Projection Data
        f_near = 0.1
        f_far = 1000
        f_fov = 90
        f_aspect_ratio = self.screen_height / self.screen_width
        f_fov_rad = 1 / tan(f_fov * 0.5 / 180 * pi)

        # Projection Matrix
        matProj = Matrix4()
        matProj[0][0] = f_aspect_ratio * f_fov_rad
        matProj[1][1] = f_fov_rad
        matProj[2][2] = f_far / (f_far - f_near)
        matProj[3][2] = (-f_far * f_near) / (f_far - f_near)
        matProj[2][3] = 1
        matProj[3][3] = 0

        self.matProj = matProj

    def calculate_rotation_matrix(self):
        """Generate the matrix for rotating triangles."""
        self.f_theta += 0.02

        # Rotation Z
        matRotZ = Matrix4()
        matRotZ[0][0] = cos(self.f_theta)
        matRotZ[0][1] = sin(self.f_theta)
        matRotZ[1][0] = -sin(self.f_theta)
        matRotZ[1][1] = cos(self.f_theta)
        matRotZ[2][2] = 1
        matRotZ[3][3] = 1

        # Rotation X
        matRotX = Matrix4()
        matRotX[0][0] = 1
        matRotX[1][1] = cos(self.f_theta * 0.5)
        matRotX[1][2] = sin(self.f_theta * 0.5)
        matRotX[2][1] = -sin(self.f_theta * 0.5)
        matRotX[2][2] = cos(self.f_theta * 0.5)
        matRotX[3][3] = 1

        self.matRotZ = matRotZ
        self.matRotX = matRotX


# NOTE As much as I would like to split this into more functions I can't.
# The function calls would decrease performance too much.
# Some functionallity has been split into functions due to their complexity
# but I can't afford to do the same for all of them.
def draw_object(win, r_scene):
    """Draw the loaded object to the screen using graphics.py."""
    projected_triangles = []
    for triangle in r_scene.my_obj:  # Caution: called 1000s of times per frame
        screen_width = r_scene.screen_width
        screen_height = r_scene.screen_height

        # ============== APPLY MATRIX CALCULATIONS ============================
        projected_triangle  = [[0]*3 for i in range(3)]
        translated_triangle = [[0]*3 for i in range(3)]
        rotatedZ_triangle   = [[0]*3 for i in range(3)]
        rotatedZX_triangle  = [[0]*3 for i in range(3)]

        for i in range(3):
            # Rotate in Z-Axis
            rotatedZ_triangle[i] = \
                r_scene.matRotZ.dot_vector(triangle[i])
            # Rotate in X-Axis
            rotatedZX_triangle[i] = \
                r_scene.matRotX.dot_vector(rotatedZ_triangle[i])
            # Duplicate vectors
            translated_triangle[i] = \
                rotatedZX_triangle[i].copy()
            # Offset into the screen
            translated_triangle[i][2] = \
                rotatedZX_triangle[i][2] + r_scene.distance

        # ==================== CALCULATE MIDPOINT =============================
        normal = [0.0]*3
        line1 = [0.0]*3
        line2 = [0.0]*3

        line1[0] = translated_triangle[1][0] - translated_triangle[0][0]
        line1[1] = translated_triangle[1][1] - translated_triangle[0][1]
        line1[2] = translated_triangle[1][2] - translated_triangle[0][2]

        line2[0] = translated_triangle[2][0] - translated_triangle[0][0]
        line2[1] = translated_triangle[2][1] - translated_triangle[0][1]
        line2[2] = translated_triangle[2][2] - translated_triangle[0][2]

        normal[0] = line1[1] * line2[2] - line1[2] * line2[1]
        normal[1] = line1[2] * line2[0] - line1[0] * line2[2]
        normal[2] = line1[0] * line2[1] - line1[1] * line2[0]

        l = sqrt(normal[0]**2 + normal[1]**2 + normal[2]**2)
        if l != 0:
            normal[0] /= l
            normal[1] /= l
            normal[2] /= l

        # ==================== TRIANGLE CULLING (using midpoint)===============
        # only render what can be seen
        if ((normal[0] * translated_triangle[0][0] - r_scene.v_camera[0] +
            normal[1] * translated_triangle[0][1] - r_scene.v_camera[1] +
            normal[2] * translated_triangle[0][2] - r_scene.v_camera[2] < 0.0)
                ^ r_scene.is_counter_clockwise):  # xor clockwise case

            # =========== ILLUMINATION ============
            dp = (normal[0] * r_scene.light_direction[0] +
                  normal[1] * r_scene.light_direction[1] +
                  normal[2] * r_scene.light_direction[2])
            dp = int(abs(dp)*200)
            color = '#%02x%02x%02x' % (dp, dp, dp)

            # Project triangles from 3D -> 2D =====
            for i in range(3):
                projected_triangle[i] = r_scene.matProj.dot_vector(translated_triangle[i])

            # Scale into view
            for i in range(3):
                for o in range(3):
                    projected_triangle[i][o] += 1
            projected_triangle[0][0] *= 0.5 * screen_width
            projected_triangle[0][1] *= 0.5 * screen_height
            projected_triangle[1][0] *= 0.5 * screen_width
            projected_triangle[1][1] *= 0.5 * screen_height
            projected_triangle[2][0] *= 0.5 * screen_width
            projected_triangle[2][1] *= 0.5 * screen_height

            # store our projected triangles for a different loop
            projected_triangles.append((projected_triangle, color))

    # (outside of for loop)
    # sort triangles from back to front
    projected_triangles.sort(key=
        lambda x: (x[0][0][2] + x[0][1][2] + x[0][2][2]) / 3.0,
        reverse=True)

    # render with tkinter
    for data in projected_triangles:
        p_tri, color = data
        win.canvas.create_polygon(  # shaded
            p_tri[0][0], p_tri[0][1],
            p_tri[1][0], p_tri[1][1],
            p_tri[2][0], p_tri[2][1],
            fill=color, outline=''
        )
        if DEBUG_TRIANGLES:
            win.canvas.create_polygon(  # wireframe
                p_tri[0][0], p_tri[0][1],
                p_tri[1][0], p_tri[1][1],
                p_tri[2][0], p_tri[2][1],
                fill='', outline='black'
            )


def multiply_matrix_vector(i, m):
    """
    Dot product of two parameters
    (`i` being the vector and `m` being the matrix).

    A new vector is returned.
    """
    o = [0.0, 0.0, 0.0]

    o[0] = i[0] * m[0][0] + i[1] * m[1][0] + i[2] * m[2][0] + m[3][0]
    o[1] = i[0] * m[0][1] + i[1] * m[1][1] + i[2] * m[2][1] + m[3][1]
    o[2] = i[0] * m[0][2] + i[1] * m[1][2] + i[2] * m[2][2] + m[3][2]
    w = i[0] * m[0][3] + i[1] * m[1][3] + i[2] * m[2][3] + m[3][3]

    if w != 0:
        o[0] /= w
        o[1] /= w
        o[2] /= w

    return o


if __name__ == "__main__":
    main()
