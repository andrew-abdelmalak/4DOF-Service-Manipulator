# MATLAB / Simulink Simulation

MATLAB-based kinematic modeling, trajectory planning, and Simscape Multibody simulation for the 4-DOF manipulator.

## Directory Layout

```
matlab/
├── kinematics/        # Core kinematic functions
├── trajectory/        # Trajectory planners and data generation
├── simscape/          # Simulink Simscape Multibody model
└── tests/             # Validation scripts
```

## Quick Start

```matlab
% Add all paths
addpath('kinematics');
addpath('trajectory');
addpath('tests');
```

## Running Tests

The test suite validates FK, IK, and velocity kinematics across multiple joint configurations.

```matlab
% 1. Generate the symbolic FK function (required once)
run_fk_4dof

% 2. Validate forward kinematics
test_forward_position       % Zero-config → [0, 0, 0.4255] m

% 3. Validate inverse kinematics
test_inverse_kinematics     % FK → IK roundtrip, ‖q_rec − q‖ = 0

% 4. Validate velocity kinematics
test_velocity_kinematics    % FK/IK velocity roundtrip, error ~10⁻¹⁵
```

## Kinematics Functions

| Function | Description |
|----------|-------------|
| `forward_kinematics_func.m` | Coder-compatible FK: `(q1,q2,q3,q4) → [x,y,z]` |
| `transformation_func.m` | Standard DH transformation matrix (numeric + symbolic) |
| `jacobian_matrix.m` | 6×4 geometric Jacobian |
| `inverse_kinematics_func.m` | Newton–Raphson IK solver (DLS, λ = 0.05) |
| `inverse_kinematics_with_orientation.m` | IK with wrist orientation constraint |
| `forward_velocity_kinematics.m` | V = J(q) · q̇ |
| `get_transform_matrix.m` | Full 4×4 homogeneous transform for orientation |

## Trajectory Planning

| Script | Description |
|--------|-------------|
| `movement_generation.m` | Full tea-preparation sequence (5 phases) → `robot_tea_data.mat` |
| `square_trajectory.m` | 5 cm × 5 cm square path with TimeSeries output |
| `generate_simulink_data.m` | Circular trajectory for Simscape → `trajectory_data.mat` |

## Simscape Model

Open the Simulink model:

```matlab
open('simscape/x5_DOF_Robot_Assembly.slx')
```

The CAD assembly is imported via `x5_DOF_Robot_Assembly_DataFile.m` which contains rigid transforms, joint definitions, and solid body properties for all 13 bodies.

## DH Parameters

| Link | θᵢ | dᵢ (m) | aᵢ (m) | αᵢ (rad) |
|------|-----|---------|---------|----------|
| 1 | q₁ + π/2 | 0.04355 | 0 | π/2 |
| 2 | q₂ | 0 | 0.140 | π |
| 3 | q₃ | 0 | 0.133 | π |
| 4 | q₄ | 0 | 0.109 | 0 |
