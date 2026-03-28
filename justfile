justfile_path := `realpath justfile`
tbai_build_dir := "/tmp/tbai_build_123"
zenoh_c_build_dir := "/tmp/zenoh_c_build"
zenoh_cpp_build_dir := "/tmp/zenoh_cpp_build"
tbai_sdk_build_dir := "/tmp/tbai_sdk_build"
tbai_mujoco_build_dir := "/tmp/tbai_mujoco_build"
current_dir := `realpath justfile`

# Show available commands
help:
    #!/usr/bin/env bash
    just -l

# Fresh install go2 environment
[group("1. fresh")]
fresh-install-go2: clean clone-tbai build-tbai ros-build-go2 install-tbai-cbf-mppi
    #!/usr/bin/env bash
    catkin build elevation_mapping elevation_mapping_cupy
    echo "All good 🤗"

# Fresh install go2-gpu-free environment
[group("1. fresh")]
fresh-install-go2-gpu-free: clean clone-tbai build-tbai ros-build-go2 install-tbai-cbf-mppi
    #!/usr/bin/env bash
    catkin build elevation_mapping
    echo "All good 🤗"

# Fresh install all environment
[group("1. fresh")]
fresh-install-all: clean clone-tbai build-tbai ros-build-all install-tbai-cbf-mppi    #!/usr/bin/env bash
    catkin build elevation_mapping elevation_mapping_cupy
    echo "All good 🤗"

# Fresh install all-gpu-free environment
[group("1. fresh")]
fresh-install-all-gpu-free: clean clone-tbai build-tbai ros-build-all install-tbai-cbf-mppi    #!/usr/bin/env bash
    catkin build elevation_mapping
    echo "All good 🤗"

#******************************************************************************************************************#
#******************************************************************************************************************#
#******************************************************************************************************************#

# Go2 MPC in Mujoco
[group("2. demos")]
go2_mpc_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_go2_mpc_mujoco deploy_go2_mpc.launch run_go2_joystick:=false run_virtual_joystick:=true unitree_channel:=1 network_interface:=lo mujoco_simulation:=true run_rviz:=true

# Go2 NP3O in Mujoco
[group("2. demos")]
go2_np3o_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_go2_rl_mujoco deploy_go2_np3o.launch run_go2_joystick:=false run_virtual_joystick:=true unitree_channel:=1 network_interface:=lo mujoco_simulation:=true run_rviz:=true publish_pointcloud:=false


# G1 humanoid in MuJoCo
[group("2. demos")]
g1_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_g1_mujoco g1_mujoco.launch mujoco_simulation:=true run_rviz:=true

# ANYmal B blind MPC in MuJoCo
[group("2. demos")]
anymal_b_mpc_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_anymal_b_mujoco anymal_b_mpc_mujoco.launch mujoco_simulation:=true run_rviz:=true

# ANYmal C blind MPC in MuJoCo
[group("2. demos")]
anymal_c_mpc_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_anymal_c_mujoco anymal_c_mpc_mujoco.launch mujoco_simulation:=true run_rviz:=true

# ANYmal D blind MPC in MuJoCo
[group("2. demos")]
anymal_d_mpc_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_anymal_d_mujoco anymal_d_mpc_mujoco.launch mujoco_simulation:=true run_rviz:=true

# ANYmal D blind Bob in MuJoCo
[group("2. demos")]
anymal_d_bob_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_anymal_d_mujoco anymal_d_bob_mujoco.launch mujoco_simulation:=true run_rviz:=true

# ANYmal D perceptive Bob in MuJoCo
[group("2. demos")]
anymal_d_bob_perceptive_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_anymal_d_mujoco anymal_d_bob_perceptive_mujoco.launch mujoco_simulation:=true run_rviz:=true

# ANYmal D blind DTC in MuJoCo
[group("2. demos")]
anymal_d_dtc_blind_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_anymal_d_mujoco anymal_d_dtc_blind_mujoco.launch mujoco_simulation:=true run_rviz:=true

# ANYmal D perceptive DTC in MuJoCo
[group("2. demos")]
anymal_d_dtc_perceptive_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_anymal_d_mujoco anymal_d_dtc_perceptive_mujoco.launch mujoco_simulation:=true run_rviz:=true

# ANYmal D JOE in MuJoCo
[group("2. demos")]
anymal_d_joe_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_anymal_d_mujoco anymal_d_joe_mujoco.launch mujoco_simulation:=true run_rviz:=true

# Spot in MuJoCo (with MPC + RViz)
[group("2. demos")]
spot_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_spot_mujoco spot_mujoco.launch mujoco_simulation:=true run_rviz:=true

# Spot Arm in MuJoCo (with MPC + RViz)
[group("2. demos")]
spot_arm_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_spot_mujoco spot_arm_mujoco.launch mujoco_simulation:=true run_rviz:=true

# Franka Panda in MuJoCo (with MPC + RViz)
[group("2. demos")]
franka_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_franka_mujoco franka_mujoco.launch mujoco_simulation:=true run_rviz:=true enable_mpc:=true

# Go2W (wheeled) drive in MuJoCo
[group("2. demos")]
go2w_drive_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_go2w_mujoco go2w_mujoco_drive.launch run_rviz:=true

# Go2W (wheeled) handstand in MuJoCo
[group("2. demos")]
go2w_handstand_mujoco:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_go2w_mujoco go2w_mujoco_handstand.launch run_rviz:=false

# Spot with arm, MPC, dummy
[group("2. demos")]
spot_arm_mpc_dummy:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_mpc spot_arm_blind.launch

# Franka arm, MPC, dummy
[group("2. demos")]
franka_arm_mpc_dummy:
    #!/usr/bin/env bash
    source $(catkin locate)/devel/setup.bash
    roslaunch tbai_ros_mpc franka.launch

#******************************************************************************************************************#
#******************************************************************************************************************#
#******************************************************************************************************************#

#******************************************************************************************************************#
#******************************************************************************************************************#
#******************************************************************************************************************#

# Format C++ code using clang-format, disabled for 'dependencies' folder
[group("3. development")]
format:
    #!/usr/bin/env bash
    folders=$(ls -d */ | grep -v thirdparty)
    for folder in $folders; do
        for file in $(find $folder -name "*.hpp" -o -name "*.cpp"); do
            echo "[TBAI] Formatting $file"
            clang-format -i -style=file $file
        done
    done

# Run cpplint on all folders, disabled for 'dependencies' folder
[group("3. development")]
lint:
    #!/usr/bin/env bash
    folders=$(ls -d */ | grep -v dependencies)
    for folder in $folders; do
        cpplint --recursive $folder
    done

# Build ROS packages (only those related to tbai_ros)
[group("3. development")]
build:
    #!/usr/bin/env bash
    ros_packages=""
    search_dirs=(./ tbai_ros_examples/ tbai_ros_controllers/ tbai_ros_examples/tbai_ros_go2/)
    for search_dir in ${search_dirs[@]}; do
        for folder in "$search_dir"*/; do
            folder=${folder%/}
            if [[ -f "$folder/CMakeLists.txt" ]] && [[ -f "$folder/package.xml" ]]; then
                package=$(basename "$folder")
                ros_packages+=" $package"
            fi
        done
    done
    echo "[TBAI] Building ROS packages:$ros_packages"
    catkin build $ros_packages

# List all pixi environments\
[group("3. development")]
pixi-list-envs:
    #!/usr/bin/env bash
    pixi workspace environment list | grep -E '^- ' | cut -d':' -f1 | sed 's/^- //'

# Generate conda environments for all pixi environments
[group("3. development")]
pixi-generate-conda-envs:
    #!/usr/bin/env bash
    set -euo pipefail
    all_envs=$(pixi workspace environment list | grep -E '^- ' | cut -d':' -f1 | sed 's/^- //')
    for env in $all_envs; do
        pixi workspace export conda-environment -e $env > .conda/$env.yaml
    done

# Clean ROS workspace and remove tbai
[group("3. development")]
clean:
    #!/usr/bin/env bash
    CURRENT_DIR=$(pwd)
    cd ../..
    catkin init
    catkin config --cmake-args -Wno-dev -DCMAKE_BUILD_TYPE=Release
    echo "Cleaning ROS workspace: $CURRENT_DIR"
    cd $CURRENT_DIR
    catkin clean -y
    rm -rf ${tbai_build_dir}

# Build and install zenoh-c (requires Rust)
[group("4.0 zenoh")]
build-zenoh-c:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ -f "$CONDA_PREFIX/lib/libzenohc.so" ]; then
        echo "[TBAI] zenoh-c already installed, skipping."
        exit 0
    fi
    echo "[TBAI] Building zenoh-c (requires Rust)..."
    [ -d "thirdparty/zenoh-c" ] || \
        git clone --depth 1 https://github.com/eclipse-zenoh/zenoh-c.git thirdparty/zenoh-c
    cmake -S thirdparty/zenoh-c -B {{zenoh_c_build_dir}} \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" \
        -DZENOHC_BUILD_WITH_UNSTABLE_API=ON \
        -DZENOHC_BUILD_WITH_SHARED_MEMORY=ON
    cmake --build {{zenoh_c_build_dir}} --config Release -j"$(nproc)" --target install
    echo "[TBAI] zenoh-c installed."

# Build and install zenoh-cpp headers
[group("4.0 zenoh")]
build-zenoh-cpp: build-zenoh-c
    #!/usr/bin/env bash
    set -euo pipefail
    if [ -f "$CONDA_PREFIX/lib/cmake/zenohcxx/zenohcxxConfig.cmake" ]; then
        echo "[TBAI] zenoh-cpp already installed, skipping."
        exit 0
    fi
    echo "[TBAI] Installing zenoh-cpp headers..."
    [ -d "thirdparty/zenoh-cpp" ] || \
        git clone --depth 1 https://github.com/eclipse-zenoh/zenoh-cpp.git thirdparty/zenoh-cpp
    cmake -S thirdparty/zenoh-cpp -B {{zenoh_cpp_build_dir}} \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" \
        -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
        -DZENOHCXX_ZENOHC=ON -DZENOHCXX_ZENOHPICO=OFF
    cmake --build {{zenoh_cpp_build_dir}} --config Release -j"$(nproc)" --target install
    echo "[TBAI] zenoh-cpp installed."

# Clone tbai_sdk (skips if already exists)
[group("4.1 tbai_sdk")]
clone-tbai-sdk:
    #!/usr/bin/env bash
    set -euo pipefail
    TBAI_SDK_DIR="thirdparty/tbai_sdk"
    if [[ ! -d "$TBAI_SDK_DIR" ]]; then
        echo "[TBAI] Cloning tbai_sdk..."
        git clone --depth 1 git@github.com:tbai-lab/tbai_sdk.git "$TBAI_SDK_DIR"
    else
        echo "[TBAI] tbai_sdk already exists at $TBAI_SDK_DIR"
        if [[ -d "$TBAI_SDK_DIR/.git" ]]; then
            echo "[TBAI] Pulling latest changes..."
            git -C "$TBAI_SDK_DIR" pull
        fi
    fi

# Build and install tbai_sdk
[group("4.1 tbai_sdk")]
build-tbai-sdk: build-zenoh-cpp clone-tbai-sdk
    #!/usr/bin/env bash
    set -euo pipefail
    TBAI_SDK_DIR="thirdparty/tbai_sdk"
    if [ -f "$CONDA_PREFIX/lib/cmake/tbai_sdk/tbai_sdkConfig.cmake" ]; then
        echo "[TBAI] tbai_sdk already installed, skipping."
        exit 0
    fi
    echo "[TBAI] Building and installing tbai_sdk..."
    cmake -S "$TBAI_SDK_DIR" -B {{tbai_sdk_build_dir}} \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX"
    cmake --build {{tbai_sdk_build_dir}} --parallel "$(nproc)" --target install
    echo "[TBAI] tbai_sdk installed."

# Clone tbai_mujoco (skips if already exists)
[group("4.15 tbai_mujoco")]
clone-tbai-mujoco:
    #!/usr/bin/env bash
    set -euo pipefail
    if [[ ! -d "thirdparty/tbai_mujoco" ]]; then
        echo "[TBAI] Cloning tbai_mujoco..."
        git clone --depth 1 git@github.com:tbai-lab/tbai_mujoco.git thirdparty/tbai_mujoco
    else
        echo "[TBAI] tbai_mujoco already exists"
        if [[ -d "thirdparty/tbai_mujoco/.git" ]]; then
            echo "[TBAI] Pulling latest changes..."
            git -C thirdparty/tbai_mujoco pull
        fi
    fi

# Build tbai_mujoco
[group("4.15 tbai_mujoco")]
build-tbai-mujoco: build-tbai-sdk clone-tbai-mujoco
    #!/usr/bin/env bash
    set -euo pipefail
    echo "[TBAI] Building tbai_mujoco..."
    cmake -S thirdparty/tbai_mujoco -B {{tbai_mujoco_build_dir}} \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX"
    cmake --build {{tbai_mujoco_build_dir}} --parallel "$(nproc)" --target install
    echo "[TBAI] tbai_mujoco installed."

# Clone tbai repository (skips if already exists)
[group("4.2 tbai")]
clone-tbai:
    #!/usr/bin/env bash
    if [[ ! -d thirdparty/tbai ]]; then
        git clone https://github.com/tbai-lab/tbai.git --single-branch --branch=develop thirdparty/tbai
    else
        echo "[TBAI] thirdparty/tbai already exists, skipping clone"
        if [[ -d thirdparty/tbai/.git ]]; then
            echo "[TBAI] thirdparty/tbai exists, pulling latest changes"
            git -C thirdparty/tbai pull
        fi
    fi

# Build tbai library
[group("4.2 tbai")]
build-tbai: build-tbai-mujoco
    #!/usr/bin/env bash
    cmake -B{{tbai_build_dir}} -Sthirdparty/tbai -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX
    cmake --build {{tbai_build_dir}} --parallel "$(nproc)" --target install

# Remove tbai dependencies
[group("4.2 tbai")]
remove-tbai:
    #!/usr/bin/env bash
    rm -rf thirdparty/tbai && rm -rf ${tbai_build_dir}

[group("4.2 tbai")]
install-tbai-cbf-mppi: clone-tbai
    #!/usr/bin/env bash
    pip3 install git+https://github.com/tbai-lab/tbai_cbf_mppi.git

# Build all ROS packages
[group("3. development")]
ros-build-all: build

# Build go2 ROS packages
[group("3. development")]
ros-build-go2: 
    #!/usr/bin/env bash
    catkin build tbai_ros_go2_rl_mujoco

