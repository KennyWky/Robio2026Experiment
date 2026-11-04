# Multi-UAV Formation Navigation with Haptic Guidance and Adaptive CBF-Based Control

This repository contains the Unity simulation project and control framework developed for our ROBIO 2026 paper.

## Overview

The system consists of three UAVs in a leader–follower formation. The leader is controlled by a human operator using a haptic device, while two followers move autonomously.

The proposed framework combines:

- Adaptive elliptical Control Barrier Functions (CBFs) for obstacle avoidance.
- Risk-adaptive formation impedance for flexible formation control.
- Haptic guidance for the human-controlled leader.

## Requirements

- Unity 2022 (use the matching editor version specified in `ProjectSettings/ProjectVersion.txt`)
- High Definition Render Pipeline (HDRP)
- Required third-party assets and haptic device software, as applicable

## How to Run

1. Clone or download this repository.
2. Open the project using Unity Hub.
3. Allow Unity to import the required packages.
4. Open the experiment scene from the `Assets` folder.
5. Verify the required scene settings and external device connections.
6. Run the simulation.

## Reference

Kaiyuan Wang, Mikhail Svinin, Ruck Thawonmas, and Wenbin Liu, "Multi-UAV Formation Navigation with Haptic Guidance and Adaptive CBF-Based Control," ROBIO 2026.

## Notes

Please refer to the repository's version information and third-party dependency instructions before attempting to reproduce the experiments.
