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


%% Segment 2
t_i = gpst2time(2254, 121009.0);
t_i =  t_i.time + t_i.sec;

t_f = gpst2time(2254, 121129.0);
t_f =  t_f.time + t_f.sec;

pos_blh_i = [30.4530279166 114.4605521035 30.994];
pos_blh_i(1) = deg2rad(pos_blh_i(1));
pos_blh_i(2) = deg2rad(pos_blh_i(2));

vel_i = [ -2.117     11.63     0.019];

eul_zyx_ned = deg2rad([  0.97041508,  -1.47477896, -9.26712073]);

R_b2ned = eul2rotm(eul_zyx_ned, "XYZ");

R_ENU2NED = [0, 1, 0;
            1, 0, 0;
            0, 0, -1];

R_NED2ENU = R_ENU2NED;

R_b2enu = R_NED2ENU * R_b2ned * R_ENU2NED;

eul_zyx_enu = rotm2eul(R_b2enu, "XYZ");


avp_i=[eul_zyx_enu,vel_i,pos_blh_i]';

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