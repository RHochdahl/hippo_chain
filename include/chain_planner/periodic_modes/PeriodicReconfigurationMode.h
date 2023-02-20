#ifndef PERIODICRECONFIGURATIONMODE_H
#define PERIODICRECONFIGURATIONMODE_H


#include "PeriodicMode.h"


class PeriodicReconfigurationMode : public PeriodicMode
{
private:
    double form;
    boost::function<void(double)> setForm;

    void callback()
    {
        form *= -1;
        setForm(form);
    }


public:
    PeriodicReconfigurationMode(const double period, const double delay, boost::function<void(double)> _setForm)
    : PeriodicMode(0.5*period, delay, boost::bind(&PeriodicReconfigurationMode::callback, this))
    , form(1.0)
    , setForm(_setForm)
    {}

    ~PeriodicReconfigurationMode() = default;
};

#endif  // PERIODICRECONFIGURATIONMODE_H