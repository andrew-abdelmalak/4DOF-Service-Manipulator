function [T0_4, p_EE, R0_4, A_all] = forward_kinematics_4dof(q, L)

% OUTPUTS:
%   T0_4 : overall 4x4 transform from base {0} to end-effector {4}
%   p_EE : 3x1 position vector of EE in {0}
%   R0_4 : 3x3 rotation matrix (orientation of {4} wrt {0})
%   A_all: cell array {A01,A12,A23,A34} for inspection/debug

q1=q(1); q2=q(2); q3=q(3); q4=q(4);
l1=L(1); l2=L(2); l3=L(3); l4=L(4);

% ---- Row 1: (theta=q1, d=l1, a=0,  alpha=+pi/2)
A01 = transformation_DH(q1, l1, 0,  pi/2);

% ---- Row 2: (theta=q2, d=0,  a=l2, alpha=0)
A12 = transformation_DH(q2, 0,  l2, 0);

% ---- Row 3: (theta=q3, d=0,  a=l3, alpha=0)
A23 = transformation_DH(q3, 0,  l3, 0);

% ---- Row 4: (theta=q4, d=0,  a=l4, alpha=0)
A34 = transformation_DH(q4, 0,  l4, 0);

% ---- Chain them: 0T4 = 0T1 * 1T2 * 2T3 * 3T4
T0_4 = A01 * A12 * A23 * A34;

% Useful slices
R0_4 = T0_4(1:3,1:3);   % orientation
p_EE = T0_4(1:3,4);     % position

% return all A's for checking
A_all = {A01, A12, A23, A34};
end
