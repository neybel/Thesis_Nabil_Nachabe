    clc
    clear 
    close all
    load("w_evol_old.mat");
    w_evol1_old=w_evol1;w_evol2_old=w_evol2;
    load ("w_evol_new.mat");
    W_xu1 = [1e1;1e1;1e-4;1e-4;1e-4;1e-2;1];
 
%% car 1
   figure(1)
 subplot(3,1,1)
 plot(1:length(w_evol1),w_evol1(1,:),1:length(w_evol1_old),w_evol1_old(2,:))
 hold on
 plot(1:length(w_evol1_old),ones(1,length(w_evol1_old))*W_xu1(1),'p')
 title('Evolution for Estimation for weights of car 1 ')

 grid on
 legend('Wxnew','Wxold');

 subplot(3,1,2)
 plot(1:length(w_evol1),w_evol1(6,:),1:length(w_evol1_old),w_evol1_old(6,:))
 hold on
 plot(1:length(w_evol1_old),ones(1,length(w_evol1_old))*W_xu1(6),'p')
 grid on
 legend('Wu1new','Wu1old');
 subplot(3,1,3)
 plot(1:length(w_evol1),w_evol1(7,:),1:length(w_evol1_old),w_evol1_old(7,:))
 hold on 
  plot(1:length(w_evol1_old),ones(1,length(w_evol1_old))*W_xu1(7),'p')
 legend('Wu2new','Wu2old');
 grid on
 %% car 2
    figure(2)
  subplot(3,1,1)
 plot(1:length(w_evol2),w_evol2(1,:),1:length(w_evol2_old),w_evol2_old(2,:))  
 hold on
 plot(1:length(w_evol1_old),ones(1,length(w_evol1_old))*W_xu1(1),'p')
 title('Evolution for Estimation for weights of car 1 ')

 grid on
 legend('Wxnew','Wxold');

 subplot(3,1,2)
 plot(1:length(w_evol2),w_evol2(6,:),1:length(w_evol2_old),w_evol2_old(6,:))
hold on
 plot(1:length(w_evol1_old),ones(1,length(w_evol1_old))*W_xu1(6),'p')
 grid on
 legend('Wu1new','Wu1old');

 subplot(3,1,3)
 plot(1:length(w_evol2),w_evol2(7,:),1:length(w_evol2_old),w_evol2_old(7,:))
 hold on
 plot(1:length(w_evol1_old),ones(1,length(w_evol1_old))*W_xu1(7),'p')
 legend('Wu2new','Wu2old');
 grid on