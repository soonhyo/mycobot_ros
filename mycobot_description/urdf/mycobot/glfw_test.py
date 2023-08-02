import glfw
from OpenGL.GL import *

print('GLFW version:', glfw.get_version())

if not glfw.init():
    raise RuntimeError('Could not initialize GLFW3')

window = glfw.create_window(300, 300, 'prog1 5.1', None, None)
if not window:
    glfw.terminate()
    raise RuntimeError('Could not create an window')

glfw.make_context_current(window)

print('Vendor :', glGetString(GL_VENDOR))
print('GPU :', glGetString(GL_RENDERER))
print('OpenGL version :', glGetString(GL_VERSION))

glfw.terminate()
