# todo: in readme place this script in the scripts file, or change the path here
#TASK3_ENV_LAUNCH_FILE_PATH = os.path.dirname(__file__) + "/../launch/task3_env.launch"
TASK3_ENV_LAUNCH_FILE_PATH = "/root/catkin_ws/src/task3_env/launch/task3_env.launch"


def launch():
    # Launch the simulation

    # Bad bc it blocks the rest of the code
    os.system("roslaunch task3_env task3_env.launch")

    # Bad bc there is no way to stop it
    os.system("roslaunch task3_env task3_env.launch &")

    # Doesn't work
    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    # launch = roslaunch.scriptapi.ROSLaunch()
    # launch.parent = roslaunch.parent.ROSLaunchParent(uuid, TASK3_ENV_LAUNCH_FILE_PATH)
    # launch.start()

    # Doesn't work
    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    # cli_args = ['task3_env', 'task3_env.launch']
    # roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
    # launch_files = [roslaunch_file]
    # launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    # launch.start()

    # return launch

