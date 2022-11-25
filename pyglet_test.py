import pyglet
from pyglet import shapes
import numpy as np
from pyglet.gl import *



win_width, win_height = 640, 480
window = pyglet.window.Window(win_width, win_height, resizable=True)

batch = pyglet.graphics.Batch()

car = shapes.BorderedRectangle(0, 0, 50, 20, border=3, border_color=(0, 0, 0))
car.opacity = 128
car.anchor_position = 25, 10
lmargin = 100   

gc_x = 0
gc_y = 0
gc_psi = -0.0
@window.event
def on_draw():
    global gc_psi
    car.x = lmargin + gc_x
    car.y = lmargin + gc_y
    car.rotation = gc_psi * 180 / np.pi
    car.draw()
    gc_psi += 0.001
   

pyglet.app.run()