#ifndef SRC_DOCK_STATE_HPP
#define SRC_DOCK_STATE_HPP


// the current robot states
struct RobotState {
    enum State {
        IDLE,
        SCAN,
        FIND_DOCK,
        GET_PARALLEL,
        POSITION_ALIGN,
        ANGLE_ALIGN,
        DOCKING,
        TURN_AROUND, //尾部对接需要
        LAST_DOCK,   //尾部对接需要
        DOCKED_IN,
    };
};

#endif //SRC_DOCK_STATE_HPP
