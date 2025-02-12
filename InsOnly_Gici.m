clc
clear all %#ok
close all

addpath(genpath(pwd))
global_variable;
 
% execute GINavCfg to configure input file
[opt,file,~]=GINavCfg;

% [obsr,~, ~ ,imu]=read_infile(opt,file);

% read imu file
if ~strcmp(file.imu,'')
    imu=readimu(opt,file.imu);
    if imu.n==0&&opt.ins.mode~=glc.GIMODE_OFF
        error('Number of imu data is zero!!!');
    end
elseif strcmp(file.imu,'')&&(opt.ins.mode==glc.GIMODE_LC||opt.ins.mode==glc.GIMODE_TC)
    error('GNSS/INS integration mode,but have no imu file!!!');
end


%% Segment 1
% t_i = gpst2time(2254, 121126.0);
% t_i =  t_i.time + t_i.sec;
% 
% t_f = gpst2time(2254,121152.0);
% t_f =  t_f.time + t_f.sec;
% 
% 
% pos_blh_i = [31.0299236017 121.44114007 16.961];
% pos_blh_i(1) = deg2rad(pos_blh_i(1));
% pos_blh_i(2) = deg2rad(pos_blh_i(2));
% 
% 
% att_i = deg2rad([ -0.90309001 0.80889952 -69.83475042]);
% vel_i = [8.2658899 3.1408411 0.026144235];

%% Segment 2
t_i = gpst2time(2254, 121009.0);
t_i =  t_i.time + t_i.sec;

t_f = gpst2time(2254, 121129.0);
t_f =  t_f.time + t_f.sec;


pos_blh_i = [31.030362935 121.4424361217 17.029];
pos_blh_i(1) = deg2rad(pos_blh_i(1));
pos_blh_i(2) = deg2rad(pos_blh_i(2));


att_i = deg2rad([  1.657  -1.349 110.337]);
vel_i = [ -5.362273     -2.2233678     0.06540917];

avp_i=[att_i,vel_i,pos_blh_i]';

ins=ins_init(opt.ins,avp_i);


header = {'time_utc', 'lat', 'lon', 'alt', ...
            'vel_e', 'vel_n', 'vel_u', ...
            'pitch', 'roll', 'yaw'};

nimu = imu.n;
pva_ins=zeros(nimu,10);

for i=1:nimu
   
    imud = imu.data(i);
    time_utc = gpst2utc(imud.time);
    time_utc = time_utc.time + time_utc.sec;

    ins=ins_mech(ins,imud);

    blh = ins.pos;
    blh(1) = rad2deg(ins.pos(1));
    blh(2) = rad2deg(ins.pos(2));

    pva_ins(i,:) = [time_utc,...
                    blh',...
                    ins.vel', ...
                    rad2deg(ins.att)', ...
                    ];

end


out = [header; num2cell(pva_ins)];

writecell(out, 'ins_only.csv');