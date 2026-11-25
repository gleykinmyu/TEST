#pragma once
#include <stdint.h>
namespace joystick {
// Exponential moving average over ADC counts.
// alpha is thousandths of 1: 250 is 0.25, 1000 passes the sample through.
class Ema {
    static constexpr int32_t unit=256;
    int32_t state_=0;
    uint16_t alpha_=250;
    bool primed_=false;
public:
    void alpha(uint16_t thousandths){alpha_=thousandths;}
    void reset(){primed_=false;}
    void reset(uint16_t sample){state_=int32_t(sample)*unit;primed_=true;}
    bool primed() const{return primed_;}
    uint16_t value() const{return primed_?uint16_t((state_+unit/2)/unit):0;}
    uint16_t update(uint16_t sample){
        int32_t x=int32_t(sample)*unit;
        if(!primed_||alpha_>=1000){state_=x;primed_=true;return sample;}
        int64_t step=int64_t(x-state_)*alpha_;
        state_+=int32_t(step>=0?(step+500)/1000:(step-500)/1000);
        return value();
    }
};
}
