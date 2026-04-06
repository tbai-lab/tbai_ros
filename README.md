# Towards Better Athletic Intelligence - ROS Noetic

ROS wrapper around [tbai](https://github.com/tbai-lab/tbai), for now the most mature tbai wrapper implementation.

<div align="center">

<img src="https://github.com/user-attachments/assets/5351d23c-59fd-47d2-8e8a-3f459b403339" width="60%" alt="demo"/>

</div>

## Implemented controllers

```
📦tbai
 ┣ 📂tbai_ros_static               # Static (high gain PD) controller
 ┣ 📂tbai_ros_mpc                  # NMPC controller (both perceptive and blind versions) [1]
 ┣ 📂tbai_ros_bob                  # RL walking controller, based on the wild-Anymal paper (both perceptive and blind versions) [2]
 ┣ 📂tbai_ros_dtc                  # DTC controller (perceptive) [3]
 ┣ 📂tbai_ros_joe                  # Perceptive NMPC controller with NN-based tracking controller [1], [3]
 ┣ ... (many more, this list is non-exhaustive)

 [1] Perceptive Locomotion through Nonlinear Model Predictive Control
     https://arxiv.org/abs/2208.08373
 [2] Learning robust perceptive locomotion for quadrupedal robots in the wild
     https://arxiv.org/abs/2201.08117
 [3] DTC: Deep Tracking Control
     https://arxiv.org/abs/2309.15462
```

## Installing tbai

> This is `tbai_ros` version 4.0. To view the older version of `tbai_ros`, built on top of the Gazebo simulator, see the [`release_v3.1.0`](https://github.com/tbai-lab/tbai_ros/tree/release_v3.1.0) branch.

To install `tbai_ros`, we recommend using `micromamba`, though `tbai_ros` is a full-fledged ROS package and it can be integrated into your projects in using conventional tools and methods. We use `micromamba` for reproducibility. Don't worry that ROS is past its end of life, micromamba (or pixi) will install everything for you (even on the newest Ubuntu release) 😮

### Alternative 1: micromamba
```bash
# Install micromamba
"${SHELL}" <(curl -L micro.mamba.pm/install.sh) # You might have to source your config again

# Clone tbai_ros
mkdir -p ros/src && cd ros/src
git clone https://github.com/tbai-lab/tbai_ros.git --recursive && cd tbai_ros

# Create conda environment
micromamba env create -f .conda/all-gpu-free.yaml
micromamba activate all-gpu-free

# Install tbai_ros
just fresh-install-all-gpu-free
```

Once the installation is complete, you can run one of our many examples, for instance:

```bash
# Activate pixi environment
micromamba activate all-gpu-free

# Run `just help` to see many available demos (subset of all)
$ just help
[2. demos]
    anymal_b_mpc_blind_mujoco      # ANYmal B blind MPC in MuJoCo
    anymal_d_bob_perceptive_mujoco # ANYmal D perceptive Bob in MuJoCo
    anymal_d_dtc_blind_mujoco      # ANYmal D blind DTC in MuJoCo
    anymal_d_mpc_perceptive_mujoco # ANYmal D perceptive MPC in MuJoCo
    franka_mujoco                  # Franka Panda in MuJoCo (with MPC + RViz)
    g1_mujoco                      # G1 humanoid in MuJoCo
    go2_mpc_blind_mujoco           # Go2 MPC in Mujoco
    ...

```

### Go2 deployment


https://github.com/user-attachments/assets/b424cec3-14c2-477c-bf13-06d57f93b8e6


Check out the [**tbai_ros_go2_rl**](./tbai_ros_go2/tbai_ros_go2_rl) folder for deployment-related documentation, pictures and videos 🤗

### G1 Dancing


https://github.com/user-attachments/assets/9800f52a-a25b-4aad-a526-606bdf1556b6


### Go2W handstand

https://github.com/user-attachments/assets/21dd7b1c-717b-46a5-bc86-c6228875caa0


### Go2W drive

https://github.com/user-attachments/assets/9144913f-0973-4293-87e6-a60e7bfbb3d7



### Perceptive MPC



https://github.com/lnotspotl/tbai_ros/assets/82883398/f451c12d-7525-4606-b722-726f63d852ca



### Blind MPC



[mpc_go2_blind.webm](https://github.com/user-attachments/assets/28f11d25-4ba0-4b7a-ae3d-e8b1c1a60de6)




### Perceptive Bob



https://github.com/lnotspotl/tbai_ros/assets/82883398/7f6bdefa-4299-454b-a0ef-55e463e0c88d



### DTC: Deep Tracking Control


https://github.com/lnotspotl/tbai_ros/assets/82883398/6cf672db-b737-4724-a6da-afa0c8dd19d5


### System architecture

![overview_01](https://github.com/lnotspotl/tbai_ros/assets/82883398/2c17f08d-6994-4982-8739-2b8246dfcb32)

## Controller architectures

### Mpc 
![mpc_03](https://github.com/lnotspotl/tbai_ros/assets/82883398/daabb2c2-8ced-4ffd-956e-35279b78563b)


### Bob

![bob_03](https://github.com/lnotspotl/tbai_ros/assets/82883398/3ea71f1c-b58c-4028-93d3-971592aa364d)


### DTC: Deep Tracking Control

![dtc_03](https://github.com/lnotspotl/tbai_ros/assets/82883398/10b3481d-7782-4a0e-ac31-24e2786c3402)

### Joe

![joe_03](https://github.com/lnotspotl/tbai_ros/assets/82883398/0139df20-d2ce-4de1-884f-ce37e770ee08)


### Credits
This project stands on the shoulders of giants.
None of this would have been possible were it not for the many amazing open-source projects here on Github. Please, navigate to [CREDITS](./CREDITS.md) to see a non-exhaustive list of repositories and links most inspiration was drawn from that have been instrumental during tbai's development.

Thank you all 🤗
