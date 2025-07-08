# Tkinter Perspective Renderer

This is a simple 3D object renderer using Python and `tkinter`. It supports `.obj` files composed of triangles and renders them with basic projection, rotation, and lighting.

## Features

- Renders `.obj` models in real time
- Basic perspective projection
- Simple directional lighting and backface culling
- Rotating animation
- Easy configuration of model file, object distance, and render direction

## Preview

![Rendered Name Model](./preview.png)

## Usage

1. Install requirements (only `tkinter` and standard libraries are needed).
2. Place a valid `.obj` file (triangle faces only) in the `models/` directory.
3. Edit `RENDER_FILE` in `ivans-graphics.py` to point to your desired model (and edit whether it is CLOCKWISE or COUNTERCLOCKWISE).
4. Run the script:

```bash
python ivans-graphics.py