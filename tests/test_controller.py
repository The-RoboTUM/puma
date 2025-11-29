import pygame
import sys
import time

def main():
    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    print(f"Number of joysticks: {joystick_count}")

    if joystick_count == 0:
        print("No joystick found.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Joystick name: {joystick.get_name()}")
    print(f"Number of axes: {joystick.get_numaxes()}")
    print(f"Number of buttons: {joystick.get_numbuttons()}")
    print(f"Number of hats: {joystick.get_numhats()}")

    print("\nListening for events... Press Ctrl+C to exit.")

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    print(f"Axis {event.axis} value: {event.value:>6.3f}")
                elif event.type == pygame.JOYBUTTONDOWN:
                    print(f"Button {event.button} down")
                elif event.type == pygame.JOYBUTTONUP:
                    print(f"Button {event.button} up")
                elif event.type == pygame.JOYHATMOTION:
                    print(f"Hat {event.hat} value: {event.value}")
            
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()
