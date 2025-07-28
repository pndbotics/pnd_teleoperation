#! /usr/bin/zsh

SCRIPT_PATH=$(dirname $0)
SCRIPT_FULL_PATH=$(readlink -f $0)
ABSOLUTE_PATH=$(dirname $SCRIPT_FULL_PATH)
WORKSPACE_PATH=$(dirname $ABSOLUTE_PATH)

WORKSPACE="/pnd_workspace"



# Function to display the usage of the script
function display_usage() {
    echo "Helper script to manage the Docker container for the ISU project"
    echo "Usage: ${ABSOLUTE_PATH}/dc [option]"
    echo "Options:"
    echo "  run              Create and Run a command inside the Docker container"
    echo "  start            Start the Debug Docker container"
    echo "  stop             Stop the Debug Docker container"
    echo "  exec             Execute a command inside the Debug Docker container"
    echo "  build_image      Build the Docker image"
    echo "  catkin_make      Run catkin_make command"
    echo "  catkin_build     Run catkin_build command"
    echo "  init_home        Initialize the home directory for current user"
    echo "  help             Display this help message"
}



DOCKER_NAME="pnd_retarget"

DEBUG_CONTAINER_NAME="pnd_retarget_debug"

CURRENT_USER="$(id -u)"

ROOT_USER="-u 0:0  "

DOCKER_ARGS=(
    --rm
    --net=host
    --privileged
    -v /dev:/dev
    -v /tmp/.X11-unix:/tmp/.X11-unix
    -e DISPLAY=$DISPLAY
    -v $WORKSPACE_PATH/docker/entrypoint:/entrypoint
    -v $WORKSPACE_PATH:$WORKSPACE
    -v $WORKSPACE_PATH/home:/home/$USER
    -v /etc/group:/etc/group:ro
    -v /etc/passwd:/etc/passwd:ro)

INIT_ENTRYPOINT="/entrypoint/init"

# check if nvidia gpu aviailable
if [ -x "$(command -v nvidia-smi)" ]; then
GPU_ARGS=(--runtime=nvidia
           --gpus all)
else
GPU_ARGS=()
fi


# Sub-function to run the program
function run() {

    if [ $# -eq 0 ]; then
        CMD=("bash" "-i")
    else
        CMD=("bash" "-i" "-c" "$*")
    fi
    echo "Run ${CMD[@]} in docker"
    check_docker_image
    docker run -it "${GPU_ARGS[@]}" "${DOCKER_ARGS[@]}" -u ${CURRENT_USER}   ${DOCKER_NAME} "${CMD[@]}"
}

function init_home() {
    BASHRC_PATH=${WORKSPACE_PATH}/home/.bashrc
    if [ ! -f $BASHRC_PATH ]; then
        echo "Run Init script in docker ..."
        run $INIT_ENTRYPOINT
    else
        echo "Home File already exists"
    fi

}

function sudo() {
    CMD="$@"
    if [ -z "$CMD" ]; then
        echo "No command provided, starting bash shell..."
        CMD=("bash" "-i")
    else
        echo "Executing the command: ${CMD}"
        CMD=(bash -i -c ${CMD})
    fi

    check_docker_image
    docker exec -it -u 0 ${DEBUG_CONTAINER_NAME}  ${CMD}
}

function start() {
    echo "Starting the Docker container..."
    check_docker_image
    docker run \
        -d \
        "${GPU_ARGS[@]}" -u ${CURRENT_USER} \
        --name ${DEBUG_CONTAINER_NAME} \
        "${DOCKER_ARGS[@]}" \
        ${DOCKER_NAME} \
        tail -f /dev/null

}
function stop() {
    echo "Stopping the Docker container..."
    docker stop ${DEBUG_CONTAINER_NAME}
}

function exec() {
    CMD="$@"
    if [ -z "$CMD" ]; then
        echo "No command provided, starting bash shell..."
        CMD=("bash" "-i")
    else
        echo "Executing the command: ${CMD}"
        CMD=(bash -i -c ${CMD})
    fi

    check_docker_image
    docker exec -it -u ${CURRENT_USER} ${DEBUG_CONTAINER_NAME}  ${CMD}
}

# Sub-function to build the Docker image
function build_image() {
    echo "Building the Docker image..."

    # find docker directory
    DOCKER_PATH="${WORKSPACE_PATH}/docker/Dockerfile"
    docker build -t ${DOCKER_NAME} -f ${DOCKER_PATH} .

}

function check_docker_image() {
    if [[ "$(docker images -q ${DOCKER_NAME} 2>/dev/null)" == "" ]]; then
        echo "Docker image ${DOCKER_NAME} does not exist. Please run the build_image command."
    fi
}

# Sub-function to run the catkin_make command
function dk_catkin_make() {
    echo "Running catkin_make command..."
    check_docker_image

    # Add your code here
    run "cd ros_ws&&pwd&&catkin_make"
}

# Sub-function to run the catkin_build command
function dk_catkin_build() {
    check_docker_image

    echo "Running catkin_build command..."
    run cd ros_ws&&pwd&&catkin_make
}



# check if home is initialized
DOCKER_HOME="$WORKSPACE_PATH/home"
if [ ! -d "$DOCKER_HOME" ]; then
    echo "Home directory not initialized. "
    echo "Initializing home directory..."
    mkdir -p $DOCKER_HOME
    init_home
fi

# Check if an argument is provided
if [ $# -eq 0 ]; then
    display_usage
    exit 1
fi



# Check the provided argument and call the corresponding function
case $1 in
run)
    run "${@:2}"
    ;;
sudo)
    sudo "${@:2}"
    ;;
build_image)
    build_image
    ;;
catkin_make)
    dk_catkin_make
    ;;
catkin_build)
    dk_catkin_build
    ;;
start)
    start
    ;;
stop)
    stop
    ;;
exec)
    exec "${@:2}"
    ;;
init_home)
    init_home
    ;;
*)
    echo "Invalid option: $1"
    display_usage
    exit 1
    ;;
esac
