#ifndef SCREEN1_VIEW_HPP
#define SCREEN1_VIEW_HPP

#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>
#include <mvp/View.hpp>
#include <Graph.hpp>

using namespace touchgfx;

class Screen1View : public Screen1ViewBase
{
public:
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent();
    virtual void LED1_button_clicked();
protected:
private:
    int tickCounter;

    Image background;
    Graph graph;
};

#endif // SCREEN1_VIEW_HPP
