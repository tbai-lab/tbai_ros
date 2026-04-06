#!/usr/bin/env python3
"""
Convert TWIST2 pkl motion files to CSV format for C++ loader.

TWIST2 pkl format:
- fps: frames per second
- root_pos: (num_frames, 3)
- root_rot: (num_frames, 4) quaternion (x,y,z,w)
- dof_pos: (num_frames, 29)
- local_body_pos: (num_frames, num_bodies, 3)

Output CSV format:
- First line: fps
- Per frame: px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,j0,...,j28 (42 values)
"""

import argparse
import pickle
import numpy as np
import sys


def compute_velocities(positions, fps):
    """Compute velocities using gradient."""
    dt = 1.0 / fps
    velocities = np.gradient(positions, dt, axis=0)
    return velocities


def compute_angular_velocity(quaternions, fps):
    """Compute angular velocity from quaternion sequence."""
    dt = 1.0 / fps
    num_frames = quaternions.shape[0]
    ang_vel = np.zeros((num_frames, 3))

    for i in range(1, num_frames):
        q0 = quaternions[i-1]  # x, y, z, w
        q1 = quaternions[i]

        # Quaternion difference: dq = q1 * q0^-1
        # q^-1 = conjugate / |q|^2 = [-x, -y, -z, w] / |q|^2
        q0_inv = np.array([-q0[0], -q0[1], -q0[2], q0[3]])
        q0_inv /= np.dot(q0, q0)

        # Quaternion multiplication: dq = q1 * q0_inv
        dq = np.array([
            q1[3]*q0_inv[0] + q1[0]*q0_inv[3] + q1[1]*q0_inv[2] - q1[2]*q0_inv[1],
            q1[3]*q0_inv[1] - q1[0]*q0_inv[2] + q1[1]*q0_inv[3] + q1[2]*q0_inv[0],
            q1[3]*q0_inv[2] + q1[0]*q0_inv[1] - q1[1]*q0_inv[0] + q1[2]*q0_inv[3],
            q1[3]*q0_inv[3] - q1[0]*q0_inv[0] - q1[1]*q0_inv[1] - q1[2]*q0_inv[2],
        ])

        # Convert to angular velocity: omega = 2 * log(dq) / dt
        # log(q) = (x, y, z) * arccos(w) / sin(arccos(w)) for unit quaternion
        # Simplified: omega ≈ 2 * (x, y, z) / dt for small angles
        angle = 2.0 * np.arccos(np.clip(dq[3], -1.0, 1.0))
        if abs(angle) < 1e-6:
            ang_vel[i] = np.zeros(3)
        else:
            axis = dq[:3] / (np.sin(angle / 2.0) + 1e-10)
            ang_vel[i] = axis * angle / dt

    ang_vel[0] = ang_vel[1]  # Copy first frame from second

    return ang_vel


def convert_pkl_to_csv(input_path, output_path):
    """Convert TWIST2 pkl to CSV format."""

    print(f"Loading pkl file: {input_path}")
    with open(input_path, 'rb') as f:
        data = pickle.load(f)

    # Extract data
    fps = data['fps']
    root_pos = np.array(data['root_pos'])
    root_rot = np.array(data['root_rot'])  # x,y,z,w format
    dof_pos = np.array(data['dof_pos'])

    num_frames = root_pos.shape[0]
    print(f"  FPS: {fps}")
    print(f"  Frames: {num_frames}")
    print(f"  Duration: {(num_frames - 1) / fps:.2f}s")
    print(f"  root_pos shape: {root_pos.shape}")
    print(f"  root_rot shape: {root_rot.shape}")
    print(f"  dof_pos shape: {dof_pos.shape}")

    # Verify DOF count
    if dof_pos.shape[1] != 29:
        print(f"Warning: Expected 29 DOFs, got {dof_pos.shape[1]}")

    # Compute velocities
    print("Computing velocities...")
    root_vel = compute_velocities(root_pos, fps)
    root_ang_vel = compute_angular_velocity(root_rot, fps)

    # Write CSV
    print(f"Writing CSV file: {output_path}")
    with open(output_path, 'w') as f:
        # First line: fps
        f.write(f"{fps}\n")

        # Per frame: px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,j0,...,j28
        for i in range(num_frames):
            row = []
            # Root position
            row.extend(root_pos[i].tolist())
            # Root quaternion (x,y,z,w)
            row.extend(root_rot[i].tolist())
            # Root linear velocity
            row.extend(root_vel[i].tolist())
            # Root angular velocity
            row.extend(root_ang_vel[i].tolist())
            # DOF positions
            row.extend(dof_pos[i].tolist())

            f.write(','.join(f'{v:.8f}' for v in row) + '\n')

    print(f"Conversion complete! Output: {output_path}")
    print(f"  Total values per frame: {len(row)}")


def main():
    parser = argparse.ArgumentParser(description='Convert TWIST2 pkl to CSV format')
    parser.add_argument('input', help='Input pkl file path')
    parser.add_argument('-o', '--output', help='Output CSV file path (default: input with .csv extension)')
    args = parser.parse_args()

    input_path = args.input
    if args.output:
        output_path = args.output
    else:
        output_path = input_path.rsplit('.', 1)[0] + '.csv'

    convert_pkl_to_csv(input_path, output_path)


if __name__ == '__main__':
    main()
