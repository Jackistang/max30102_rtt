import os
from building import *

cwd = GetCurrentDir()

src = Split("""
    src/max30102.c
    src/CircularBuffer.c
    port/port.c
""")

if GetDepend(['MAX30102_USING_EXAMPLE']):
    src += ['examples/example.c']


# define max30102 group
objs = DefineGroup('max30102', src, depend = ['PKG_USING_MAX30102'])

Return('objs')