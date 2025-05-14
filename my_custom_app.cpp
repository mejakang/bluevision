#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>

#include <uORB/uORB.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>

#include "../../modules/commander/px4_custom_mode.h"
#include <math.h>

class MyCustomApp : public ModuleBase<MyCustomApp>
{
public:
    static int print_usage(const char *reason = nullptr);
    static int custom_command(int argc, char *argv[]);
    static int task_spawn(int argc, char *argv[]);

private:
    void enter_offboard_mode();
    void send_setpoint(float x, float y, float z, float yaw);

    void hover();
    void move_backward();
    void move_left();
    void move_right();
};

int MyCustomApp::print_usage(const char *reason)
{
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

int MyCustomApp::custom_command(int argc, char *argv[])
{
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

    return print_usage("Unknown Command !!");
}

int MyCustomApp::task_spawn(int argc, char *argv[])
{
    return 0; 
}

void MyCustomApp::enter_offboard_mode()
{
    PX4_INFO("Preparing to enter OFFBOARD mode");

    // STEP 1. Pre-flight setpoint stream
    offboard_control_mode_s offboard_mode{};
    trajectory_setpoint_s traj_sp{};

    offboard_mode.position = true;
    offboard_mode.timestamp = hrt_absolute_time();

    traj_sp.position[0] = 0.0f;
    traj_sp.position[1] = 0.0f;
    traj_sp.position[2] = -2.0f;
    traj_sp.yaw = NAN;
    traj_sp.timestamp = hrt_absolute_time();

    orb_advert_t offboard_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_mode);
    orb_advert_t traj_sp_pub = orb_advertise(ORB_ID(trajectory_setpoint), &traj_sp);

    for (int i = 0; i < 20; i++) {
        offboard_mode.timestamp = hrt_absolute_time();
        traj_sp.timestamp = hrt_absolute_time();

        orb_publish(ORB_ID(offboard_control_mode), offboard_mode_pub, &offboard_mode);
        orb_publish(ORB_ID(trajectory_setpoint), traj_sp_pub, &traj_sp);
        px4_usleep(100000);
    }

    // STEP 2. Arm
    vehicle_command_s arm_cmd{};
    arm_cmd.timestamp = hrt_absolute_time();
    arm_cmd.param1 = 1.0f; // ARM
    arm_cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    arm_cmd.target_system = 1;
    arm_cmd.target_component = 1;
    arm_cmd.source_system = 1;
    arm_cmd.source_component = 1;
    arm_cmd.from_external = true;

    orb_advert_t cmd_pub = orb_advertise(ORB_ID(vehicle_command), &arm_cmd);
    orb_publish(ORB_ID(vehicle_command), cmd_pub, &arm_cmd);
    PX4_INFO("Arm command sent.");

    px4_usleep(1000000);

    // STEP 3. Set OFFBOARD mode
    vehicle_command_s mode_cmd{};
    mode_cmd.timestamp = hrt_absolute_time();
    mode_cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
    mode_cmd.param1 = 1; // MAV_MODE_FLAG_CUSTOM_MODE 플래그 세우기
    mode_cmd.param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
    mode_cmd.target_system = 1;
    mode_cmd.target_component = 1;
    mode_cmd.source_system = 1;
    mode_cmd.source_component = 1;
    mode_cmd.from_external = true;

    for (int i = 0; i < 3; i++) {
        orb_publish(ORB_ID(vehicle_command), cmd_pub, &mode_cmd);
        PX4_INFO("Offboard Mode command sent (%d)", i + 1);
        px4_usleep(1000000);
    }

    PX4_INFO("OFFBOARD Complete.");
}

void MyCustomApp::send_setpoint(float x, float y, float z, float yaw)
{

    PX4_INFO("Sending Setpoint: X=%.2f Y=%.2f Z=%.2f Yaw=%.2f", (double)x, (double)y, (double)z, (double)yaw);

    offboard_control_mode_s offboard_mode{};
    trajectory_setpoint_s traj_sp{};

    offboard_mode.position = true;

    traj_sp.position[0] = x;
    traj_sp.position[1] = y;
    traj_sp.position[2] = z;
    traj_sp.yaw = yaw;

    orb_advert_t offboard_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_mode);
    orb_advert_t traj_sp_pub = orb_advertise(ORB_ID(trajectory_setpoint), &traj_sp);

    for (int i = 0; i < 20; i++) {
        offboard_mode.timestamp = hrt_absolute_time();
        traj_sp.timestamp = hrt_absolute_time();

        orb_publish(ORB_ID(offboard_control_mode), offboard_mode_pub, &offboard_mode);
        orb_publish(ORB_ID(trajectory_setpoint), traj_sp_pub, &traj_sp);

        px4_usleep(100000);
    }

    PX4_INFO("Setpoint Transmission Complete.");
}

void MyCustomApp::hover()
{
    PX4_INFO("Command: Hover");
    enter_offboard_mode();
    send_setpoint(0.0f, 0.0f, -8.0f, NAN);
}

void MyCustomApp::move_backward()
{
    PX4_INFO("Command: Move Backward");
    enter_offboard_mode();
    send_setpoint(-4.0f, 7.0f, -8.0f, 0.0f);
}

void MyCustomApp::move_left()
{
    PX4_INFO("Command: Move Left");
    enter_offboard_mode();
    send_setpoint(4.0f, 7.0f, -8.0f, M_PI / 2);
    //send_setpoint(0.5f, 0.0f, -2.0f, NAN);
}

void MyCustomApp::move_right()
{
    PX4_INFO("Command: Move Right");
    enter_offboard_mode();
    send_setpoint(-1.0f, 0.0f, -2.0f, -M_PI / 2);
}
extern "C" __EXPORT int my_custom_app_main(int argc, char *argv[]) {
    return MyCustomApp::main(argc, argv);
}