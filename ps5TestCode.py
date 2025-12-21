import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import time
import pygame

THRESH = 0.08     # ignore tiny noise
HZ     = 10       # update rate (prints 10 times/sec)

def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick detected. Plug in the controller and run again.")
        return

    js = pygame.joystick.Joystick(0)
    js.init()

    na = js.get_numaxes()
    nb = js.get_numbuttons()
    nh = js.get_numhats()

    print(f"Controller: {js.get_name()}")
    print(f"axes={na}, buttons={nb}, hats={nh}")
    print("Move sticks / press triggers. Ctrl+C to quit.\n")

    dt = 1.0 / HZ

    try:
        while True:
            pygame.event.pump()

            axes = [js.get_axis(i) for i in range(na)]
            hats = [js.get_hat(i) for i in range(nh)] if nh else []
            btns = [js.get_button(i) for i in range(nb)]

            # Clear screen (works in most terminals)
            print("\033[2J\033[H", end="")

            print(f"Controller: {js.get_name()}")
            print(f"axes={na}, buttons={nb}, hats={nh}\n")

            print("AXES:")
            for i, v in enumerate(axes):
                mark = "*" if abs(v) > THRESH else " "
                print(f"  {mark} axis[{i:02d}] = {v:+.3f}")

            if hats:
                print("\nHATS:")
                for i, h in enumerate(hats):
                    print(f"  hat[{i}] = {h}")

            # Show only pressed buttons to reduce spam
            pressed = [i for i, b in enumerate(btns) if b]
            print("\nBUTTONS pressed:", pressed if pressed else "none")

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nBye.")
    finally:
        js.quit()
        pygame.quit()

if __name__ == "__main__":
    main()
