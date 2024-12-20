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
        DOCKED_IN,
    };
};

#endif //SRC_DOCK_STATE_HPP
