% run_fk_4dof.m  |  Enter q1..q4 (deg) and get end-effector pose
clear; clc;

fprintf('=== 4-DOF Forward Kinematics (standard DH) ===\n');

% ----- Link lengths (mm) from your sheet
l1 = 43.55;   l2 = 140;   l3 = 133;   l4 = 109;
L  = [l1 l2 l3 l4];

% ----- Ask user for joint angles in degrees (easier to type)
q1_deg = input('Enter q1 (deg): ');
q2_deg = input('Enter q2 (deg): ');
q3_deg = input('Enter q3 (deg): ');
q4_deg = input('Enter q4 (deg): ');

% ----- Convert to radians for trig
d2r = pi/180;
q = [q1_deg q2_deg q3_deg q4_deg] * d2r;

% ----- Compute forward kinematics
[T04, p, R, A] = forward_kinematics_4dof(q, L);


fprintf('--- Overall Transform ^0T_4 ---\n');
disp(T04);

fprintf('Position of end-effector (in mm):\n');
fprintf('  x = %.3f\n  y = %.3f\n  z = %.3f\n', p(1), p(2), p(3));


