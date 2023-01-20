load('angles2.mat');
% [n,fo,mo,w] = remezord([0.3446,0.4],[1,0],[0.0001,0.00001],2*pi);
% b = remez(n,fo,mo,w);
% 
% for limb = 1:6
%     for joint = 1:3
%         ANG = angles(limb,joint,:);
%         ANG = ANG(:);
%         ANG = filter(b,1,ANG);
%         angles(limb,joint,:) = abs(ANG);
%     end
% end

ang1_1 = angles(1,1,:);ang1_1 = ang1_1(:);
ang2_1 = angles(2,1,:);ang2_1 = ang2_1(:);
ang1_2 = angles(1,2,:);ang1_2 = ang1_2(:);
ang2_2 = angles(2,2,:);ang2_2 = ang2_2(:);
nn = 1:length(ang2_2);

[b,a] = butter(2,25/400);
ang1_2_f = filter(b,a,ang1_2);
ang2_2_f = filter(b,a,ang2_2);

figure()
subplot(411), plot(nn,ang1_1);
subplot(412), plot(nn,ang2_1);
subplot(413), plot(nn,ang2_2);
subplot(414), plot(nn,ang2_2_f);

angles(1,1,:) = 0.2*sin(2*pi*nn/90);