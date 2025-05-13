#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <uORB/uORB.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <math.h> 
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/vehicle_status.h>  

#include "../../modules/commander/px4_custom_mode.h"

class MyCustomApp : public ModuleBase<MyCustomApp> {
public:
    static int print_usage(const char *reason = nullptr);
    static int custom_command(int argc, char *argv[]);
    static int task_spawn(int argc, char *argv[]);

private:
    void hover();
    void move_backward();
    void move_left();
    void move_right();
    void enter_offboard_mode();
};


int MyCustomApp::print_usage(const char *reason) {
    if (reason) {
        PX4_WARN("%s", reason);
    }

    PRINT_MODULE_USAGE_NAME("my_custom_app", "example");
    PRINT_MODULE_USAGE_COMMAND_DESCR("hover", "Hover in place");
    PRINT_MODULE_USAGE_COMMAND_DESCR("backward", "Move backward");
    PRINT_MODULE_USAGE_COMMAND_DESCR("left", "Move left");
    PRINT_MODULE_USAGE_COMMAND_DESCR("right", "Move right");
    return 0;
}

int MyCustomApp::custom_command(int argc, char *argv[]) {
    if (argc < 1) return print_usage("No command given");

    MyCustomApp app;

    if (!strcmp(argv[0], "hover")) {
        app.hover(); return 0;
    } else if (!strcmp(argv[0], "backward")) {
        app.move_backward(); return 0;
    } else if (!strcmp(argv[0], "left")) {
        app.move_left(); return 0;
    } else if (!strcmp(argv[0], "right")) {
        app.move_right(); return 0;
    }

    return print_usage("Unknown command");
}

int MyCustomApp::task_spawn(int argc, char *argv[]) {
    return 0;  // 필요 x
}

void MyCustomApp::enter_offboard_mode() {
    PX4_INFO("Entering offboard mode.");

    orb_advert_t offboard_mode_pub = nullptr;
    orb_advert_t traj_sp_pub = nullptr;

    offboard_control_mode_s offboard_mode{};
    offboard_mode.timestamp = hrt_absolute_time();
    offboard_mode.position = true; 
    offboard_mode.velocity = false;
    offboard_mode.acceleration = false;
    offboard_mode.attitude = false;
    offboard_mode.body_rate = false;

    trajectory_setpoint_s traj_sp{};
    traj_sp.timestamp = hrt_absolute_time();
    traj_sp.position[0] = 0.0f;  // x 
    traj_sp.position[1] = 0.0f;  // y 
    traj_sp.position[2] = -2.0f; // z 
    traj_sp.yaw = NAN; // 방향 그대로


    for (int i = 0; i < 10; i++) {
        offboard_mode.timestamp = hrt_absolute_time();
        traj_sp.timestamp = hrt_absolute_time();

        // offboard_mode publish
        if (offboard_mode_pub == nullptr) {
            offboard_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_mode);
        } else {
            orb_publish(ORB_ID(offboard_control_mode), offboard_mode_pub, &offboard_mode);
        }

        // trajectory_setpoint publish
        if (traj_sp_pub == nullptr) {
            traj_sp_pub = orb_advertise(ORB_ID(trajectory_setpoint), &traj_sp);
        } else {
            orb_publish(ORB_ID(trajectory_setpoint), traj_sp_pub, &traj_sp);
        }

        // 100ms 대기 후 계속 publish
        px4_usleep(100000); // 100ms
    }

    // 오프보드 모드 진입 
    vehicle_command_s cmd{};
    cmd.timestamp = hrt_absolute_time();
    cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
    cmd.param1 = 1.0f;  // Set mode to custom
    cmd.param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD; // offboard 모드
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;

    orb_advert_t vehicle_cmd_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);

    for (int i = 0; i < 3; i++) {
        orb_publish(ORB_ID(vehicle_command), vehicle_cmd_pub, &cmd);
        PX4_INFO("Offboard mode command sent (%d)", i + 1);
        px4_usleep(1000000); // 1초 대기
    }

    // 오프보드 모드 활성화 확인
    bool offboard_activated = false;
    int retries = 0;

    // 구독: offboard_control_mode
    int mode_sub = orb_subscribe(ORB_ID(offboard_control_mode));


    while (!offboard_activated && retries < 10) {
        retries++;
        px4_usleep(500000); // 0.5초 

        // offboard_control_mode reas
        offboard_control_mode_s mode_status{};
        bool updated = false;
        orb_check(mode_sub, &updated);
        if (updated) {
            orb_copy(ORB_ID(offboard_control_mode), mode_sub, &mode_status);
            if (mode_status.position == true) {
                offboard_activated = true;
                PX4_INFO("Offboard mode successfully activated.");
            }
        } else {
            PX4_WARN("Offboard mode not yet activated. Retrying... (%d/10)", retries);
        }
    }

    if (!offboard_activated) {
        PX4_ERR("Failed to activate offboard mode after 10 attempts.");
    }

    // 구독 해제
    orb_unsubscribe(mode_sub);
}

void MyCustomApp::hover() {
    PX4_INFO("Command: Hovering in place.");


    enter_offboard_mode();

    
    static orb_advert_t offboard_mode_pub = nullptr;
    static orb_advert_t traj_sp_pub = nullptr;

    // 오프보드 모드 설정
    offboard_control_mode_s offboard_mode{};
    offboard_mode.timestamp = hrt_absolute_time();
    offboard_mode.position = true;
    offboard_mode.velocity = false;
    offboard_mode.acceleration = false;
    offboard_mode.attitude = false;
    offboard_mode.body_rate = false;

    // 궤적 설정 (현재 위치 기준)
    trajectory_setpoint_s traj_sp{};
    traj_sp.timestamp = hrt_absolute_time();
    traj_sp.position[0] = 0.0f;  // x 좌표
    traj_sp.position[1] = 0.0f;  // y 좌표
    traj_sp.position[2] = -8.0f; // z 고도
    traj_sp.yaw = NAN; // 방향 그대로

    // 1초 동안 publish 유지
    for (int i = 0; i < 10; i++) {
        offboard_mode.timestamp = hrt_absolute_time();
        traj_sp.timestamp = hrt_absolute_time();

        // 최초 한 번만 퍼블리셔 핸들 초기화
        if (offboard_mode_pub == nullptr) {
            offboard_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_mode);
        } else {
            orb_publish(ORB_ID(offboard_control_mode), offboard_mode_pub, &offboard_mode);
        }

        if (traj_sp_pub == nullptr) {
            traj_sp_pub = orb_advertise(ORB_ID(trajectory_setpoint), &traj_sp);
        } else {
            orb_publish(ORB_ID(trajectory_setpoint), traj_sp_pub, &traj_sp);
        }

        px4_usleep(100000); 
    }

    PX4_INFO("Hovering complete.");
}
void MyCustomApp::move_backward() {
    PX4_INFO("Command: Moving backward.");


    enter_offboard_mode();

    // 퍼블리셔 핸들 생성
    orb_advert_t offboard_mode_pub = nullptr;
    orb_advert_t traj_sp_pub = nullptr;

    // 오프보드 모드 설정
    offboard_control_mode_s offboard_mode{};
    offboard_mode.timestamp = hrt_absolute_time();
    offboard_mode.position = true;
    offboard_mode.velocity = false;
    offboard_mode.acceleration = false;
    offboard_mode.attitude = false;
    offboard_mode.body_rate = false;

    // 궤적 설정 (뒤로 이동)
    trajectory_setpoint_s traj_sp{};
    traj_sp.timestamp = hrt_absolute_time();
    traj_sp.position[0] = -4.0f; 
    traj_sp.position[1] = 7.0f;
    traj_sp.position[2] = -8.0f; 
    traj_sp.yaw = 0.0f; 
    // 1초 동안 publish 유지
    for (int i = 0; i < 10; i++) {
        offboard_mode.timestamp = hrt_absolute_time();
        traj_sp.timestamp = hrt_absolute_time();

        // 최초 한 번만 퍼블리셔 핸들 초기화
        if (offboard_mode_pub == nullptr) {
            offboard_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_mode);
        } else {
            orb_publish(ORB_ID(offboard_control_mode), offboard_mode_pub, &offboard_mode);
        }

        if (traj_sp_pub == nullptr) {
            traj_sp_pub = orb_advertise(ORB_ID(trajectory_setpoint), &traj_sp);
        } else {
            orb_publish(ORB_ID(trajectory_setpoint), traj_sp_pub, &traj_sp);
        }

        px4_usleep(100000); 
    }

    PX4_INFO("Moving backward complete.");
}

void MyCustomApp::move_left() {
    PX4_INFO("Command: Moving left.");


    enter_offboard_mode();
    orb_advert_t offboard_mode_pub = nullptr;
    orb_advert_t traj_sp_pub = nullptr;
    offboard_control_mode_s offboard_mode{};
    offboard_mode.timestamp = hrt_absolute_time();
    offboard_mode.position = true;
    offboard_mode.velocity = false;
    offboard_mode.acceleration = false;
    offboard_mode.attitude = false;
    offboard_mode.body_rate = false;

    // 궤적 설정 (왼쪽으로 이동)
    trajectory_setpoint_s traj_sp{};
    traj_sp.timestamp = hrt_absolute_time();
    traj_sp.position[0] = 4.0f; 
    traj_sp.position[1] = 7.0f;
    traj_sp.position[2] = -8.0f; 
    traj_sp.yaw = M_PI / 2; 


    for (int i = 0; i < 10; i++) {
        offboard_mode.timestamp = hrt_absolute_time();
        traj_sp.timestamp = hrt_absolute_time();

        if (offboard_mode_pub == nullptr) {
            offboard_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_mode);
        } else {
            orb_publish(ORB_ID(offboard_control_mode), offboard_mode_pub, &offboard_mode);
        }

        if (traj_sp_pub == nullptr) {
            traj_sp_pub = orb_advertise(ORB_ID(trajectory_setpoint), &traj_sp);
        } else {
            orb_publish(ORB_ID(trajectory_setpoint), traj_sp_pub, &traj_sp);
        }

        px4_usleep(100000); 
    }

    PX4_INFO("Moving left complete.");
}

void MyCustomApp::move_right() {
    PX4_INFO("Command: Moving right.");


    enter_offboard_mode();

    orb_advert_t offboard_mode_pub = nullptr;
    orb_advert_t traj_sp_pub = nullptr;


    offboard_control_mode_s offboard_mode{};
    offboard_mode.timestamp = hrt_absolute_time();
    offboard_mode.position = true;
    offboard_mode.velocity = false;
    offboard_mode.acceleration = false;
    offboard_mode.attitude = false;
    offboard_mode.body_rate = false;

    // 궤적 설정 (오른쪽으로 이동)
    trajectory_setpoint_s traj_sp{};
    traj_sp.timestamp = hrt_absolute_time();
    traj_sp.position[0] = -1.0f; 
    traj_sp.position[1] = 0.0f;
    traj_sp.position[2] = -2.0f; 
    traj_sp.yaw = -M_PI / 2; 


    for (int i = 0; i < 10; i++) {
        offboard_mode.timestamp = hrt_absolute_time();
        traj_sp.timestamp = hrt_absolute_time();

        if (offboard_mode_pub == nullptr) {
            offboard_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_mode);
        } else {
            orb_publish(ORB_ID(offboard_control_mode), offboard_mode_pub, &offboard_mode);
        }

        if (traj_sp_pub == nullptr) {
            traj_sp_pub = orb_advertise(ORB_ID(trajectory_setpoint), &traj_sp);
        } else {
            orb_publish(ORB_ID(trajectory_setpoint), traj_sp_pub, &traj_sp);
        }

        px4_usleep(100000); 
    }

    PX4_INFO("Moving right complete.");
}

extern "C" __EXPORT int my_custom_app_main(int argc, char *argv[]) {
    return MyCustomApp::main(argc, argv);
}
