#include "SDLViewer.h"

#include <vector>
#include <iostream>

namespace
{

    Uint32 redraw_callback(Uint32 interval, void *param)
    {
        SDL_Event event;
        SDL_UserEvent userevent;

        userevent.type = SDL_USEREVENT;
        userevent.code = 0;
        userevent.data1 = NULL;
        userevent.data2 = NULL;

        event.type = SDL_USEREVENT;
        event.user = userevent;

        SDL_PushEvent(&event);
        return (interval);
    }
} // namespace

SDLViewer::SDLViewer()
    : window(nullptr), window_surface(nullptr), redraw_next(false)
{

}
void SDLViewer::resize(const int w, const int h)
{
    if(!window)
        return;

    SDL_SetWindowSize(window, w, h);
    window_surface = SDL_GetWindowSurface(window);
    update();
}

void SDLViewer::update()
{
    if(!redraw_next)
        return;
    redraw_next = false;

    if (redraw != nullptr)
        redraw(*this);
}

bool SDLViewer::init(const std::string &window_name, const int w, const int h)
{
    //Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0)
    {
        std::cout << "error initializing SDL: " << SDL_GetError() << std::endl;
        return false;
    }
    else
    {
        //Create window
        window = SDL_CreateWindow(window_name.c_str(),
                                  SDL_WINDOWPOS_CENTERED,
                                  SDL_WINDOWPOS_CENTERED, w, h,
                                  SDL_WINDOW_SHOWN);
        if (window == NULL)
        {
            // In the case that the window could not be made...
            std::cout << "Could not create window: " << SDL_GetError() << std::endl;
            return false;
        }

        //Get window surface
        window_surface = SDL_GetWindowSurface(window);
    }

    return true;
}

bool SDLViewer::draw_image(
    const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> &R,
    const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> &G,
    const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> &B,
    const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> &A)
{
    std::vector<uint8_t> data(R.size() * 4);
    for (int i = 0; i < R.size(); ++i)
    {
        data[4 * i + 0] = R(i);
        data[4 * i + 1] = G(i);
        data[4 * i + 2] = B(i);
        data[4 * i + 3] = A(i);
    }

    // 4 bytes per pixel * pixels per row
    const int depth = 32;
    const int pitch = 4 * R.rows();

    static const Uint32 rmask = 0x000000ff;
    static const Uint32 gmask = 0x0000ff00;
    static const Uint32 bmask = 0x00ff0000;
    static const Uint32 amask = 0xff000000;

    //The image we will load and show on the screen
    SDL_Surface *surface;
    surface = SDL_CreateRGBSurfaceFrom((void *)&data[0], R.rows(), R.cols(),
                                       depth, pitch,
                                       rmask, gmask, bmask, amask);
    if (surface == nullptr)
    {
        std::cout << "Unable to load image SDL Error: " << SDL_GetError() << std::endl;
        return false;
    }

    SDL_SetSurfaceBlendMode(surface,SDL_BLENDMODE_NONE);
    SDL_BlitSurface(surface, NULL, window_surface, NULL);
    SDL_UpdateWindowSurface(window);
    SDL_FreeSurface(surface);

    return true;
}

void SDLViewer::launch(const int redraw_interval)
{
    redraw(*this);
    bool is_quit = false;

    Uint32 delay = (10 / 10) * redraw_interval;
    SDL_TimerID my_timer_id = SDL_AddTimer(delay, redraw_callback, nullptr);

    SDL_Event event;
    while (!is_quit)
    {
        if (SDL_WaitEvent(&event))
        {
            switch (event.type)
            {
            case SDL_QUIT:
                is_quit = true;
                break;
            case SDL_MOUSEMOTION:
                if (mouse_move != nullptr)
                    mouse_move(event.motion.x, event.motion.y, event.motion.xrel, event.motion.yrel);
                break;

            case SDL_KEYDOWN:
            case SDL_KEYUP:
                if (key_pressed != nullptr)
                    key_pressed(event.key.keysym.sym, event.key.state == SDL_PRESSED, event.key.keysym.mod, event.key.repeat);
                break;

            case SDL_MOUSEBUTTONDOWN:
            case SDL_MOUSEBUTTONUP:
                if (mouse_pressed != nullptr)
                    mouse_pressed(event.button.x, event.button.y, event.button.state == SDL_PRESSED, event.button.button, event.button.clicks);
                break;

            case SDL_MOUSEWHEEL:
                if (mouse_wheel != nullptr)
                    mouse_wheel(event.wheel.x, event.wheel.y, event.wheel.direction == SDL_MOUSEWHEEL_NORMAL);
                break;

            case SDL_USEREVENT:
                if(event.user.code == 0)
                    update();
                break;
            }

        }
    }
}

SDLViewer::~SDLViewer()
{
    //Destroy window
    SDL_DestroyWindow(window);
    window = nullptr;

    //Quit SDL subsystems
    SDL_Quit();
}

void SDLViewer::dump() {
    /*
            NORM = 0, //nothing strange or pressed 'space' will go to normal.
            HOLD_ON,
            INSERT_ON, // 'i' pressed and click will then add vertex, and mouse motion active
            INSERT1, // have add 1 vertex
            INSERT2, // have add 2 vertex
            DELETE, // p pressed and select will delete
            TRANSLATION, //
            COLOR, //color mode ,key 1-9 are active and the click will change to a neighbor vertex
            KEYFRAME_START,//record keyframe 1
            KEYFRAME_TRANSLATION, // record and want translation
            KEYFRAME_HOLD_ON,
            KEYFRAME_OVER, // record keyframe 2
    */
    std::cout<< "CURRENT STATE: ";
    switch(this->Event_Mode){
        case INSERT_ON:
            std::cout<< "INSERT_ON" << std::endl;
            break;
        case INSERT1:
            std::cout<< "INSERT1" << std::endl;
            break;
        case INSERT2:
            std::cout<< "INSERT2" << std::endl;
            break;
        case DELETE:
            std::cout<< "DELETE" << std::endl;
            break;
        case TRANSLATION:
            std::cout<< "TRANSLATION" << std::endl;
            break;
        case COLOR:
            std::cout<< "COLOR" << std::endl;
            break;
        case KEYFRAME_START:
            std::cout<< "KEYFRAME_START" << std::endl;
            break;
        case KEYFRAME_TRANSLATION:
            std::cout<< "KEYFRAME_TRANSLATION" << std::endl;
            break;
        case KEYFRAME_OVER:
            std::cout<< "KEYFRAME_OVER" << std::endl;
            break;
        case HOLD_ON:
            std::cout<< "HOLD_ON" << std::endl;
            break;
        case KEYFRAME_HOLD_ON:
            std::cout<< "KEYFRAME_HOLD_ON" << std::endl;
            break;
        default:
            std::cout<< "NORM" << std::endl;
    }
}
