clc
clear all %#ok
close all


ref = load('D:\local\GINav\data\data_cpt\cpt_pva_ref.mat');
reference = ref.reference;
nref=size(reference,1);

 pva_ref=zeros(nref,10);

 header = {'time_utc', 'lat', 'lon', 'alt', ...
            'vel_e', 'vel_n', 'vel_u', ...
            'pitch', 'roll', 'yaw'};

 for i=1:nref
    t_utc = gpst2utc( ...
                gpst2time( ...
                    reference(i).week,reference(i).sow ...
                          ));
    t_utc = t_utc.time + t_utc.sec;

    [blh,Cne]=xyz2blh(reference(i).pos); 
    blh(1) = rad2deg(blh(1));
    blh(2) = rad2deg(blh(2));
    vel = Cne * reference(i).vel';

    pva_ref(i,:)=[t_utc, blh, vel', ...
                  reference(i).att];
 end

 out = [header; num2cell(pva_ref)];

 writecell(out, 'ground_truth.csv');