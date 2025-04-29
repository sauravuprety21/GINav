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

ref = load('D:\local\GINav\data\data_cpt\cpt_pva_ref.mat');

ref_i = 950+1;
ref_f = ref_i + 120 + 1;


t_i = gpst2utc(gpst2time(ref.reference(ref_i).week, ...
            ref.reference(ref_i).sow));
t_i =  t_i.time + t_i.sec;

t_f = gpst2utc(gpst2time(ref.reference(ref_f).week, ...
            ref.reference(ref_f).sow));
t_f =  t_f.time + t_f.sec;


pos_xyz_i = ref.reference(ref_i).pos;
[pos_blh_i, Cne_i] = xyz2blh(pos_xyz_i);

att_i = deg2rad(ref.reference(ref_i).att);

yaw = att_i(3);
if yaw > pi
    att_i(3) = 2*pi - yaw;
else
    att_i(3) = -1*yaw;
end
vel_i = Cne_i * ref.reference(ref_i).vel';

avp_i=[att_i,vel_i',pos_blh_i]';

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


pos_xyz_f = ref.reference(ref_f).pos;
[pos_blh_f, Cne_f] = xyz2blh(pos_xyz_f);

att_f = deg2rad(ref.reference(ref_f).att);
vel_f = Cne_f * ref.reference(ref_f).vel';

out = [header; num2cell(pva_ins)];

writecell(out, 'ins_only.csv');