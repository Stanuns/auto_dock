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

        SCAN_IR,
        FIND_IR,
        GET_IR,
        GET_IR_RETURN,
        SCAN_TO_ALIGN_IR,
        ALIGNED_IR,
        DOCKING_IR,
        DOCKED_IN_IR,
    };
};

struct DockStationIRState {
    enum State {
        INVISIBLE=0, //没有接收到信号
        LEFT=1,   //接收到dock的左边信号
        CENTER=2, //接收到dock的中间信号
        RIGHT=3,  //接收到dock的右边信号
    };
};

#endif //SRC_DOCK_STATE_HPP
