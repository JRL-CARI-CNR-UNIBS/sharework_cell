close all;clc;clear all;

addpath('/home/jacobi/projects/ros_ws/src/common/binary_logger/scripts')

test_name='test';
data{2}=bin_convert(sprintf('~/.ros/%s_WrenchStamped__wrench.bin',test_name),7);
data{1}=bin_convert(sprintf('~/.ros/%s_Tf_tool.bin',test_name),8);

data_r=bin_resampling(data,2e-3);

t=data_r{1}(:,1);
t=t-t(1);
wrench=data_r{2}(:,2:end);
transformation=data_r{1}(:,2:end);
xyz=transformation(:,1:3);

subplot(2,1,1)
plot(t,xyz);
subplot(2,1,2)
plot(t,wrench(:,1:3))
