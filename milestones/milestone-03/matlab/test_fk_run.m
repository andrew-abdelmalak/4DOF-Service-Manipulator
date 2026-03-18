%% test_fk_run.m
% Quick test to call fk_pos_func and show output

% make sure we are in the right folder (edit path if needed)
cd('K:\Semester 9\Robotics\GUC_747_67_59121_2025-10-19T15_25_34\5-dof-robotic-arm-5.snapshot.3\SimscapeFiles');

% (re)generate fk_pos_func and q_vals if needed:
run('run_fk_4dof.m');   % this creates q_vals and fk_pos_func

% List workspace variables
who

% Call the generated FK function and display result
pos_from_fk = fk_pos_func(q_vals);
disp('pos_from_fk =');
disp(pos_from_fk);
