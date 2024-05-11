import sys

from vispy import scene
from vispy.color import Color
canvas = scene.SceneCanvas(keys='interactive', size=(800, 600), show=True)

# Set up a viewbox to display the cube with interactive arcball
view = canvas.central_widget.add_view()
view.bgcolor = '#efefef'
view.camera = 'turntable'
view.padding = 100

# color = Color("#3f51b5")

color1 = (0,0,1,0.2)
color2 = (1,0,0,0.2)

cube = scene.visuals.Box(1, 1, 1, color=color1, edge_color="black",
                         parent=view.scene)

cube2 = scene.visuals.Box(0.2, 0.2, 0.4, color=color2, edge_color="black",
                         parent=view.scene)

if __name__ == '__main__' and sys.flags.interactive == 0:
    canvas.app.run()