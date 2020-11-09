close all;clc;close all;

addpath('/home/jacobi/projects/ros_ws/src/common/binary_logger/scripts')

data{1}=bin_convert('~/.ros/test_JointState__joint_states.bin',6*3+1);
data{2}=bin_convert('~/.ros/test_Tf_tool.bin',8);
data{3}=bin_convert('~/.ros/test_WrenchStamped__wrench.bin',7);

data_r=bin_resampling(data,2e-3);

t=data_r{1}(:,1);
t=t-t(1);
wrench=data_r{3}(:,2:end);

transformation=data_r{2}(:,2:end);

xyz=transformation(:,1:3);

subplot(2,1,1)
plot(t,xyz);
subplot(2,1,2)
plot(t,wrench(:,1:3))
