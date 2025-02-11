clc
clear all %#ok
close all

addpath(genpath(pwd))
global_variable;
 
% execute GINavCfg to configure input file
[opt,file,~]=GINavCfg;

[obsr,~, ~ ,imu]=read_infile(opt,file);

ref = load('D:\local\GINav\data\data_cpt\cpt_pva_ref.mat');

ref_i = 1000;
ref_f = 1020;


t_i = gpst2time(ref.reference(ref_i).week, ...
            ref.reference(ref_i).sow);
t_i =  t_i.time + t_i.sec;

t_f = gpst2time(ref.reference(ref_f).week, ...
            ref.reference(ref_f).sow);
t_f =  t_f.time + t_f.sec;


pos_xyz = ref.reference(ref_i).pos;
pos_blh = xyz2blh(pos_xyz);

att = ref.reference(ref_i).att;
vel = ref.reference(ref_i).vel;

avp0=[att,vel,pos_blh]';

ins=ins_init(opt.ins,avp0);

while 1
     % search imu data
    [imud,imu,stat]=searchimu(imu);
    time1=imud.time.time+imud.time.sec;
    
    if(time1 >= t_i) && (time1 <= t_f)

        ins=ins_mech(ins,imud);
    end;

end