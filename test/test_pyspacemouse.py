import pyspacemouse
import time

success = pyspacemouse.open(dof_callback=pyspacemouse.print_state, button_callback=pyspacemouse.print_buttons)
if success:
    while 1:
        state = pyspacemouse.read()
        # print(state)
        # print(type(state))
        # print(dir(state))
        # # 你可以尝试直接访问常见属性
        # print(state.x, state.y, state.z)
        # print(state.buttons)
        time.sleep(0.01)