#ifndef BOOT_H_
#define BOOT_H_

class Boot {
    public:

        Boot();
        void init();
        void main();
        void setState(int newState);
        int getState();

    private:
                // Receiver _receiver;
        Controller _controller;
        
        Comm _comm;

        VehicleState _state;

        const String START_FLAG = "DATA_START";
        const String STOP_FLAG = "DATA_STOP";

        
        enum VehicleState {
            BOOT,
            INITIALIZING,
            CALIBRATING,
            STANDBY_EMPTY,
            STANDBY_LOADED,
            ACTIVE,
            ABORT_HOT,
            COOLDOWN
        };
};

#endif  //  BOOT_H_