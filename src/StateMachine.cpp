#include <Wire.h>
#include <cstdint>

class StateMachine{
    private:
        uint8_t currState;

    public:
        void Idle(){
            return;
        }
        void Ascent(){
            Serial.print("ascent state");
        }
        void Descent(){
            Serial.print("descent state");
        }
        void Recovery(){
            return;
        }

};