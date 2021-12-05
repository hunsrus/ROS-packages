#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>

struct axis_state {
    short x, y;
};

class joystick{
    private:
        const char *device;
    public:
        const char* getDevice();
        void setDevice(const char*);
        int read_event(int fd, struct js_event *event);
        size_t get_axis_count(int fd);
        size_t get_button_count(int fd);
        size_t get_axis_state(struct js_event *event, struct axis_state axes[3]);
};

#endif // JOYSTICK_H