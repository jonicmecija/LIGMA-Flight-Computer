#include <cstdint>
enum States{
  IDLE = 0,
  ASCENT = 1,
  DESCENT = 2,
  RECOVERY = 3
};

class StateMachine{
    private:
        uint8_t currState;
        States states;
    public:

    StateMachine()
        {
            currState = 0;
        }
    
    void Idle();
    void Ascent();
    void Descent();
    void Recovery();
};