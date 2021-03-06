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
    virtual void updateVal(uint16_t *);
protected:
private:
    int tickCounter;

    Graph graph;
    Graph spectrum;
};

#endif // SCREEN1_VIEW_HPP
