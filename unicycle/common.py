import pyglet
from pyglet import shapes
import numpy as np

def pydraw(X, U, X0, XN):


    win_width, win_height = 640, 480
    window = pyglet.window.Window(win_width, win_height)
    batch = pyglet.graphics.Batch()

    car_image = pyglet.image.load('car.png')
    car_image.anchor_x = 32
    car_image.anchor_y = 32
    car = pyglet.sprite.Sprite(car_image, x=50, y=50)
    car.rotation = 90

    bg_color = shapes.Rectangle(0, 0, 960, 540, color=(255, 255, 255), batch=batch)
    bg_lines = []
    for i in list(range(win_width//32)):
        bg_lines.append(shapes.Line((i+1)*32, 0, (i+1)*32, win_height, width=1, color=(180, 180, 180), batch=batch))
    for i in list(range(win_height//32)):
        bg_lines.append(shapes.Line(0, (i+1)*32, win_width, (i+1)*32, width=1, color=(180, 180, 180), batch=batch))
    

    start_point = shapes.Circle(100 + X0[0] * 100, 100 + X0[1] * 100, 3, color=(200, 40, 40), batch=batch)
    end_point   = shapes.Circle(100 + XN[0] * 100, 100 + XN[1] * 100, 3, color=(200, 40, 40), batch=batch)

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
        frame += 1
        if frame % 6 == 0:
            idx = (idx + 1) % len(U)
            if idx == 0:
                traj = []
        window.clear()
        labelX.text = f"EgoX:  {X[idx, 0]:.2f}"
        labelY.text = f"EgoY:  {X[idx, 1]:.2f}"
        labelTheta.text = f"Theta:  {X[idx, 3]:.2f}"
        labelF.text = f"F:  {U[idx, 0]:.2f}"
        labelT.text = f"T:  {U[idx, 1]:.2f}"
        labelStep.text = f"step:  {idx}"
        batch.draw()

        if len(traj) > 1:
            batch1 = pyglet.graphics.Batch()
            tmp = []
            for x,y in traj:
                tmp.append(shapes.Circle(x, y, 1, color=(160, 40, 40), batch=batch1))
            batch1.draw()
                


        car.x = 100 + X[idx ,0] * 100
        car.y = 100 + X[idx ,1] * 100
        car.rotation = 90 - X[idx ,3] * 180 / np.pi
        car.draw()
        if len(traj) == 0 or traj[-1] != (car.x, car.y):
            traj.append((car.x, car.y))
        



    pyglet.app.run()