import pyglet
from pyglet import shapes
import numpy as np
from pyglet.gl import *

wheel_base = 2.8
car_length = 4.6
car_width = 1.8

park_width = car_width + 0.6
park_length = 2 * car_length + 0.5
road_width = 1.5 * car_length
bot = 2

lf = 1.4
lr = 1.4
wheel_y = 0.8
wheel_length = 0.7
wheel_width = 0.23


meter2pixel = 30

wheel_base *= meter2pixel
car_length *= meter2pixel
car_width *= meter2pixel
lf *= meter2pixel
lr *= meter2pixel
wheel_y *= meter2pixel
wheel_length *= meter2pixel
wheel_width *= meter2pixel

lmargin = 100
def pydraw(X, U, X_des):

    # print("FPS: ", pyglet.clock.get_frequency())
    win_width, win_height = 1280, 720
    window = pyglet.window.Window(win_width, win_height, resizable=True)

    batch = pyglet.graphics.Batch()

    arrow_image = pyglet.image.load('arrow.png')
    arrow_image.anchor_x = 0
    arrow_image.anchor_y = 16
    arrow = pyglet.sprite.Sprite(arrow_image, x=50, y=50)
    arrow.scale = 0.25
    arrow.rotation = -90


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
    grid_size = 64
    for i in list(range(win_width//grid_size)):
        bg_lines.append(shapes.Line((i+1)*grid_size, 0, (i+1)*grid_size, win_height, width=1, color=(180, 180, 180), batch=batch))
    for i in list(range(win_height//grid_size)):
        bg_lines.append(shapes.Line(0, (i+1)*grid_size, win_width, (i+1)*grid_size, width=1, color=(180, 180, 180), batch=batch))
    
    bg_lines.append(shapes.Line(win_width//2 - park_width * meter2pixel, bot * meter2pixel, \
                                win_width//2 + park_width * meter2pixel, bot * meter2pixel, \
                                width=2, color=(20, 20, 20), batch=batch))
    bg_lines.append(shapes.Line(win_width//2 - park_width * meter2pixel, bot * meter2pixel, \
                                win_width//2 - park_width * meter2pixel, (bot + park_length) * meter2pixel, \
                                width=2, color=(20, 20, 20), batch=batch))
    bg_lines.append(shapes.Line(win_width//2, bot * meter2pixel, \
                                win_width//2, (bot + park_length) * meter2pixel, \
                                width=1, color=(100, 100, 100), batch=batch))
    bg_lines.append(shapes.Line(win_width//2 + park_width * meter2pixel, bot * meter2pixel, \
                                win_width//2 + park_width * meter2pixel, (bot + park_length) * meter2pixel, \
                                width=2, color=(20, 20, 20), batch=batch))
    bg_lines.append(shapes.Line(0, (bot + park_length) * meter2pixel, \
                                win_width//2 - park_width * meter2pixel, (bot + park_length) * meter2pixel, \
                                width=2, color=(20, 20, 20), batch=batch))
    bg_lines.append(shapes.Line(win_width//2 + park_width * meter2pixel, (bot + park_length) * meter2pixel, \
                                win_width, (bot + park_length) * meter2pixel, \
                                width=2, color=(20, 20, 20), batch=batch))
    bg_lines.append(shapes.Line(0, (bot + park_length + road_width) * meter2pixel, \
                                win_width, (bot + park_length + road_width) * meter2pixel, \
                                width=2, color=(20, 20, 20), batch=batch))
    bg_lines.append(shapes.Circle(win_width // 2 - park_width * meter2pixel, (bot + park_length) * meter2pixel,\
                                  4, color=(255, 0, 0), batch=batch))
    bg_lines.append(shapes.Circle(win_width // 2 + park_width * meter2pixel, (bot + park_length) * meter2pixel,\
                                  4, color=(255, 0, 0), batch=batch))
    bg_lines.append(shapes.Circle(win_width // 2, (bot + park_length) * meter2pixel,\
                                  4, color=(255, 0, 0), batch=batch))
    bg_lines.append(shapes.Circle(win_width // 2 - park_width * meter2pixel, bot * meter2pixel,\
                                  4, color=(255, 0, 0), batch=batch))
    bg_lines.append(shapes.Circle(win_width // 2 + park_width * meter2pixel, bot * meter2pixel,\
                                  4, color=(255, 0, 0), batch=batch))
    bg_lines.append(shapes.Circle(win_width // 2, bot * meter2pixel,\
                                  4, color=(255, 0, 0), batch=batch))                                 

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
    batch1 = pyglet.graphics.Batch()
    @window.event
    def on_draw():
        nonlocal idx
        nonlocal frame
        nonlocal traj
        # glScalef(2.0, 2.0, 2.0)
        # glClearColor(1, 1, 1, 1)

        frame += 1
        if frame % 2 == 0:
            idx = (idx + 1) % len(U)
            if idx == 0:
                traj = []
        window.clear()
        # print(idx)
        # print(X[idx])
        labelX.text = f"X:  {X[idx, 0]:+.2f}"
        labelY.text = f"Y:  {X[idx, 1]:+.2f}"
        labelTheta.text = f"Psi:  {X[idx, 2]:+.2f}"
        labelV.text = f"V:  {X[idx, 3]:+.2f}"
        labelF.text = f"a:  {U[idx, 0]:+.2f}"
        labelT.text = f"delta_dot:  {U[idx, 1]:+.2f}"
        labelStep.text = f"step:  {idx}"
        batch.draw()

        for X_step in X_des:
            arrow.x = lmargin + X_step[0] * meter2pixel
            arrow.y = lmargin + X_step[1] * meter2pixel
            arrow.rotation = -X_step[2] / np.pi * 180
            arrow.draw()

        # if len(traj) > 1:
        #     batch1 = pyglet.graphics.Batch()
        #     tmp = []
        #     for x,y in traj:
        #         tmp.append(shapes.Circle(x, y, 1, color=(80, 180, 0), batch=batch1))
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
        car_center = shapes.Circle(lmargin + gc_x, lmargin + gc_y, 2, color=(255, 0, 0))
        car_center.draw()
        if len(traj) == 0 or traj[-1][:2] != (car.x, car.y):
            traj.append((car.x, car.y, shapes.Circle(car.x, car.y, 1, color=(80, 180, 0), batch=batch1)))
            # tmp.append(
        pyglet.image.get_buffer_manager().get_color_buffer().save(f'imgs/screenshot{idx:04}.png')
        

    @window.event
    def on_resize(width, height):
        print("on_resize", width, height)
        glViewport(0, 0, *window.get_framebuffer_size())
        window.projection = pyglet.math.Mat4.orthogonal_projection(0, width, 0, height, -255, 255)

        # glViewport(0, 0, *window.get_framebuffer_size())
        # window.projection = pyglet.math.Mat4.perspective_projection(window.aspect_ratio, z_near=0.1, z_far=255)
        return pyglet.event.EVENT_HANDLED

    pyglet.app.run()