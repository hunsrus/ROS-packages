#include "joystick.h"

const char* joystick::getDevice()
{
    return this->device;
}

void joystick::setDevice(const char* device)
{
    this->device = device;
}

/**
 * Lee un evento del joystick
 * Devuelve 0 si resulta exitoso, si no devuelve -1
 */
int joystick::read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

/**
 * Devuelve el número de ejes del control o 0 si ocurre un error.
 */
size_t joystick::get_axis_count(int fd)
{
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

/**
 * Devuelve el número de botones del control o 0 si ocurre un error.
 */
size_t joystick::get_button_count(int fd)
{
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}

/**
 * Seguimiento del estado actual del eje.
 *
 * NOTA: Asume que los ejes se numeran empezando del 0, que el eje X es
 * un número par y que el eje Y es un número impar.
 *
 * Devuelve el eje que indica el evento.
 */
size_t joystick::get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}