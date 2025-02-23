#ifndef SRC_DOCK_STATE_HPP
#define SRC_DOCK_STATE_HPP


// the current robot states
struct RobotState {
    enum State {
        IDLE,
        SCAN,
        FIND_WALL,//垂直于墙壁方向，并保证离墙距离范围0.8~1.1m
        SCAN2,//开始找DOCK
        FIND_DOCK,
        GET_PARALLEL,
        MOVE_ALIGN,
        POSITION_ALIGN, //未被使用
        POSITION_ALIGN_EXTENSION, //未被使用
        ANGLE_ALIGN,
        DOCKING,
        TURN_AROUND, //尾部对接需要 //未被使用
        LAST_DOCK,   //尾部对接需要 //未被使用
        DOCKED_IN,
    };
};

#endif //SRC_DOCK_STATE_HPP
