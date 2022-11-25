import pyglet
from pyglet import shapes
import numpy as np
from pyglet.gl import *

wheel_base = 3.0
car_length = 4.8
car_width = 1.8

lf = 1.5
lr = 1.5
wheel_y = 0.8
wheel_length = 0.7
wheel_width = 0.23


meter2pixel = 20

wheel_base *= meter2pixel
car_length *= meter2pixel
car_width *= meter2pixel
lf *= meter2pixel
lr *= meter2pixel
wheel_y *= meter2pixel
wheel_length *= meter2pixel
wheel_width *= meter2pixel

lmargin = 100
def pydraw(X, U, X0, XN, total_steps):


    win_width, win_height = 1280, 720
    window = pyglet.window.Window(win_width, win_height, resizable=True)

    batch = pyglet.graphics.Batch()

    car = shapes.BorderedRectangle(0, 0, car_length, car_width, border=2, border_color=(0, 0, 0))
    car.opacity = 128
    car.anchor_position = car_length / 2, car_width / 2
    wheel1 = shapes.BorderedRectangle(0, 0, wheel_length, wheel_width, border=2, border_color=(0, 0, 0))
    wheel1.opacity = 128
    wheel1.anchor_position = wheel_length / 2, wheel_width / 2

    wheel2 = shapes.BorderedRectangle(0, 0, wheel_length, wheel_width, border=2, border_color=(0, 0, 0))
    wheel2.opacity = 128
    wheel2.anchor_position = wheel_length / 2, wheel_width / 2

    wheel3 = shapes.BorderedRectangle(0, 0, wheel_length, wheel_width, border=2, border_color=(0, 0, 0))
    wheel3.opacity = 128
    wheel3.anchor_position = wheel_length / 2, wheel_width / 2

    wheel4 = shapes.BorderedRectangle(0, 0, wheel_length, wheel_width, border=2, border_color=(0, 0, 0))
    wheel4.opacity = 128
    wheel4.anchor_position = wheel_length / 2, wheel_width / 2

    bg_color = shapes.Rectangle(0, 0, win_width, win_height, color=(255, 255, 255), batch=batch)
    bg_lines = []
    for i in list(range(win_width//32)):
        bg_lines.append(shapes.Line((i+1)*32, 0, (i+1)*32, win_height, width=1, color=(180, 180, 180), batch=batch))
    for i in list(range(win_height//32)):
        bg_lines.append(shapes.Line(0, (i+1)*32, win_width, (i+1)*32, width=1, color=(180, 180, 180), batch=batch))
    

    start_point = shapes.Circle(lmargin + X0[0] * meter2pixel, lmargin + X0[1] * meter2pixel, 3, color=(200, 40, 40), batch=batch)
    start_point_line = shapes.Line(lmargin + X0[0] * meter2pixel, lmargin + X0[1] * meter2pixel, 
                                   lmargin + X0[0] * meter2pixel + 40 * np.cos(X0[2]), lmargin + X0[1] * meter2pixel + 40 * np.sin(X0[2]),
                                   1, color=(200, 40, 40), batch=batch)
    end_point   = shapes.Circle(lmargin + XN[0] * meter2pixel, lmargin + XN[1] * meter2pixel, 3, color=(200, 40, 40), batch=batch)
    end_point_line = shapes.Line(lmargin + XN[0] * meter2pixel, lmargin + XN[1] * meter2pixel, 
                                 lmargin + XN[0] * meter2pixel + 40 * np.cos(XN[2]), lmargin + XN[1] * meter2pixel + 40 * np.sin(XN[2]),
                                 1, color=(200, 40, 40), batch=batch)
    obs0_point = shapes.Circle(lmargin + 2 * meter2pixel, lmargin + 2 * meter2pixel, 3, color=(200, 40, 40), batch=batch)

    labelX = pyglet.text.Label('EgoX:',
                            font_name='Times New Roman',
                            font_size=10,
                            bold=True,
                            color=(0, 0, 0, 255),
                            x=300, y=100, batch=batch)
    labelY = pyglet.text.Label('EgoY:',
                            font_name='Times New Roman',
                            font_size=10,
                            bold=True,
                            color=(0, 0, 0, 255),
                            x=300, y=80, batch=batch)
    labelTheta = pyglet.text.Label('Theta:',
                            font_name='Times New Roman',
                            font_size=10,
                            bold=True,
                            color=(0, 0, 0, 255),
                            x=300, y=60, batch=batch)
    labelV = pyglet.text.Label('V:',
                            font_name='Times New Roman',
                            font_size=10,
                            bold=True,
                            color=(0, 0, 0, 255),
                            x=300, y=40, batch=batch)
    labelF = pyglet.text.Label('F:',
                            font_name='Times New Roman',
                            font_size=10,
                            bold=True,
                            color=(0, 0, 0, 255),
                            x=400, y=100, batch=batch)
    labelT = pyglet.text.Label('T:',
                            font_name='Times New Roman',
                            font_size=10,
                            bold=True,
                            color=(0, 0, 0, 255),
                            x=400, y=80, batch=batch)
    labelStep = pyglet.text.Label('step:',
                            font_name='Times New Roman',
                            font_size=10,
                            bold=True,
                            color=(0, 0, 0, 255),
                            x=400, y=60, batch=batch)
    idx = 0
    frame = 0
    traj = []
    @window.event
    def on_draw():
        nonlocal idx
        nonlocal frame
        nonlocal traj
        # glScalef(2.0, 2.0, 2.0)
        # glClearColor(1, 1, 1, 1)

        frame += 1
        if frame % 6 == 0:
            idx = (idx + 1) % total_steps
            if idx == 0:
                traj = []
        window.clear()
        labelX.text = f"EgoX:  {X[idx, 0]:+.2f}"
        labelY.text = f"EgoY:  {X[idx, 1]:+.2f}"
        labelTheta.text = f"Psi:  {X[idx, 2]:+.2f}"
        labelV.text = f"V:  {X[idx, 3]:+.2f}"
        labelF.text = f"a:  {U[idx, 0]:+.2f}"
        labelT.text = f"delta_dot:  {U[idx, 1]:+.2f}"
        labelStep.text = f"step:  {idx}"
        batch.draw()

        if len(traj) > 1:
            batch1 = pyglet.graphics.Batch()
            tmp = []
            for x,y in traj:
                tmp.append(shapes.Circle(x, y, 1, color=(80, 180, 0), batch=batch1))
            batch1.draw()
                

        gc_x = X[idx, 0] * meter2pixel
        gc_y = X[idx, 1] * meter2pixel
        gc_psi = -X[idx, 2]
        fw_delta = -X[idx, 4]
        # fw_delta = -U[idx, 1]
        
        w_pos_theta = -np.pi + np.arctan(wheel_y / lr) + gc_psi
        w_pos_r = np.linalg.norm([wheel_y, lr])
        wheel2.x = lmargin + gc_x + w_pos_r * np.cos(w_pos_theta)
        wheel2.y = lmargin + gc_y - w_pos_r * np.sin(w_pos_theta)
        wheel2.rotation = gc_psi * 180 / np.pi

        w_pos_theta = -np.pi - np.arctan(wheel_y / lr) + gc_psi
        w_pos_r = np.linalg.norm([wheel_y, lr])
        wheel3.x = lmargin + gc_x + w_pos_r * np.cos(w_pos_theta)
        wheel3.y = lmargin + gc_y - w_pos_r * np.sin(w_pos_theta)
        wheel3.rotation = gc_psi * 180 / np.pi

        w_pos_theta = -np.arctan(wheel_y / lf) + gc_psi
        w_pos_r = np.linalg.norm([wheel_y, lf])
        wheel1.x = lmargin + gc_x + w_pos_r * np.cos(w_pos_theta)
        wheel1.y = lmargin + gc_y - w_pos_r * np.sin(w_pos_theta)
        wheel1.rotation = (gc_psi + fw_delta)* 180 / np.pi

        w_pos_theta = np.arctan(wheel_y / lf) + gc_psi
        w_pos_r = np.linalg.norm([wheel_y, lf])
        wheel4.x = lmargin + gc_x + w_pos_r * np.cos(w_pos_theta)
        wheel4.y = lmargin + gc_y - w_pos_r * np.sin(w_pos_theta)
        wheel4.rotation = (gc_psi + fw_delta) * 180 / np.pi

        car.x = lmargin + gc_x
        car.y = lmargin + gc_y
        car.rotation = gc_psi * 180 / np.pi
        wheel1.draw()
        wheel2.draw()
        wheel3.draw()
        wheel4.draw()
        car.draw()
        car_center = shapes.Circle(lmargin + gc_x, lmargin + gc_y, 2, color=(255, 0, 255))
        car_center.draw()
        if len(traj) == 0 or traj[-1] != (car.x, car.y):
            traj.append((car.x, car.y))
        

    @window.event
    def on_resize(width, height):
        print("on_resize", width, height)
        glViewport(0, 0, *window.get_framebuffer_size())
        window.projection = pyglet.math.Mat4.orthogonal_projection(0, width, 0, height, -255, 255)

        # glViewport(0, 0, *window.get_framebuffer_size())
        # window.projection = pyglet.math.Mat4.perspective_projection(window.aspect_ratio, z_near=0.1, z_far=255)
        return pyglet.event.EVENT_HANDLED

    pyglet.app.run()