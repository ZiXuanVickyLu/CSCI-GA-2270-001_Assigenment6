#pragma once

#include <Eigen/Core>
#include <iostream>
#include <SDL.h>
#include <SDL_timer.h>
#include <functional>

/*
 * Modifiers:
 *
 *
KMOD_NONE 0 (no modifier is applicable)
KMOD_LSHIFT the left Shift key is down
KMOD_RSHIFT the right Shift key is down
KMOD_LCTRL the left Ctrl (Control) key is down
KMOD_RCTRL the right Ctrl (Control) key is down
KMOD_LALT the left Alt key is down
KMOD_RALT the right Alt key is down
KMOD_LGUI the left GUI key (often the Windows key) is down
KMOD_RGUI the right GUI key (often the Windows key) is down
KMOD_NUM the Num Lock key (may be located on an extended keypad) is down
KMOD_CAPS the Caps Lock key is down
KMOD_MODE the AltGr key is down
KMOD_CTRL (KMOD_LCTRL|KMOD_RCTRL)
KMOD_SHIFT (KMOD_LSHIFT|KMOD_RSHIFT)
KMOD_ALT (KMOD_LALT|KMOD_RALT)
KMOD_GUI (KMOD_LGUI|KMOD_RGUI)
 *
 */
typedef enum{
    NORM = 0, //nothing strange or pressed 'space' will go to normal.
    HOLD_ON, //translating a triangle
    INSERT_ON, // 'i' pressed and click will then add vertex, and mouse motion active
    INSERT1, // have add 1 vertex
    INSERT2, // have add 2 vertex
    DELETE, // p pressed and select will delete
    TRANSLATION, //
    COLOR, //color mode ,key 1-9 are active and the click will change to a neighbor vertex
    KEYFRAME_START,//record keyframe 1
    KEYFRAME_TRANSLATION, // record and want translation
    KEYFRAME_HOLD_ON ,//
    KEYFRAME_OVER, // record keyframe 2

}Program_Mode;

class SDLViewer
{
public:
    SDLViewer();


    bool init(const std::string &window_name, const int w, const int h);

    void resize(const int w, const int h);

    bool draw_image(
        const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> &R,
        const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> &G,
        const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> &B,
        const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> &A);

    void launch(const int redraw_interval = 30);
    ~SDLViewer();

    // x, y, x_rel, y_rel
    std::function<void(int, int, int, int)> mouse_move;

    // x, y, is_pressed, button, n_clicks
    std::function<void(int, int, bool, int, int)> mouse_pressed;

    //dx, dy, is_direction_normal
    std::function<void(int, int, bool)> mouse_wheel;

    // key, is_pressed, modifier, repeat
    std::function<void(char, bool, int, int)> key_pressed;

    std::function<void(SDLViewer &)> redraw;

    std::function<void()> clean;

    Program_Mode getMode(){
        return Event_Mode;
    }

    void update();

    bool redraw_next;

    void set_Mode(Program_Mode M){
        this -> Event_Mode = M;
        this -> dump();
    }
    void set_active(int i){
        active_triangle = i;
        std::cout<< "ACTIVE: "<< i << std::endl;
    }
    void set_inactive(){
        active_triangle = -1;
        active_vertex = -1;
        std::cout<< "NONE ACTIVE"<<std::endl;
    }

    int get_active(){
        return active_triangle;
    }

    int get_vertex(){
        return active_vertex;
    }

    void set_vertex(int i){
        active_vertex = i;
    }

    void dump();
private:
    // The window we'll be rendering to
    SDL_Window *window;

    //The surface contained by the window
    SDL_Surface *window_surface;
    int active_triangle = -1;
    int active_vertex = -1;
    Program_Mode Event_Mode = NORM;
};