%% fk_4dof_demo.m
% Mirrors the provided Python forward_kinematics.py (with DH placeholders).
% - Prompts for q1..q4 (in degrees) each run
% - Uses standard DH transform to compute T04
% - Prints end-effector position [x, y, z] in meters
%
% IMPORTANT:
% Replace the DH placeholders below with your team’s correct DH parameters.

clear; clc;

% --- Link lengths (meters) • placeholders copied from the Python code ---
l1 = 0.04355;  % base offset (d1) placeholder
l2 = 0.140;    % a2 placeholder
l3 = 0.133;    % a3 placeholder
l4 = 0.109;    % a4 placeholder
% l5 = 0.050;  % kept in Python as a placeholder (not used here)

% --- Ask user for joint angles (degrees) then convert to radians ---
fprintf('Enter joint angles in DEGREES (placeholders DH assumed):\n');
q1 = deg2rad(input('q1 (deg) = '));
q2 = deg2rad(input('q2 (deg) = '));
q3 = deg2rad(input('q3 (deg) = '));
q4 = deg2rad(input('q4 (deg) = '));

% --- Forward kinematics (using same placeholder DH as the Python code) ---
[x, y, z, T04] = forward_kinematics_func(q1, q2, q3, q4, l1, l2, l3, l4);

% --- Report results ---
fprintf('\nEnd-effector position (meters):\n');
fprintf('x = %.6f\n', x);
fprintf('y = %.6f\n', y);
fprintf('z = %.6f\n', z);

disp('Final homogeneous transform T0_4 = ');
disp(T04);

% -------------------------------------------------------------------------
% Local functions
% -------------------------------------------------------------------------

function T = transformation_func(theta, d, a, alpha)
% Standard Denavit–Hartenberg homogeneous transform:
% Rot(z,theta) * Trans(z,d) * Trans(x,a) * Rot(x,alpha)
    ct = cos(theta);  st = sin(theta);
    ca = cos(alpha);  sa = sin(alpha);
    T = [ ct, -st*ca,  st*sa, a*ct;
          st,  ct*ca, -ct*sa, a*st;
           0,     sa,     ca,    d;
           0,      0,      0,    1 ];
end

function [x, y, z, T04] = forward_kinematics_func(q1, q2, q3, q4, l1, l2, l3, l4)
% Matches the placeholder DH used in your Python:
%   T01: theta=q1, d=l1, a=0,   alpha=+pi/2
%   T12: theta=q2, d=0,  a=l2,  alpha=0
%   T23: theta=q3, d=0,  a=l3,  alpha=0
%   T34: theta=q4, d=0,  a=l4,  alpha=0
% NOTE: These are PLACEHOLDERS. Update to your correct DH table when ready.

    T01 = transformation_func(q1, l1, 0,  pi/2);
    T12 = transformation_func(q2, 0,  l2, 0);
    T23 = transformation_func(q3, 0,  l3, 0);
    T34 = transformation_func(q4, 0,  l4, 0);

    T04 = T01 * T12 * T23 * T34;

    p = T04(1:3, 4);
    x = p(1); y = p(2); z = p(3);
end
