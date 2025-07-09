#pragma once

class IMotor {
    public:
    virtual ~IMotor() = default;
    virtual bool initialize() = 0;
    //Input might be PWM but leave empty for now
    virtual void buzz()const = 0;
    virtual const char* getName() const = 0;
};