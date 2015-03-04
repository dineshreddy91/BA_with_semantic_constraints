function [ noise_points ] = noise( points ,sigma_var)
% function [ noise_points ] = noise( points ,sigma_1)

sigma			=	[sigma_var 0 0;0 sigma_var 0 ;0 0 sigma_var];
noise_points	=	mvnrnd(	points, sigma );

