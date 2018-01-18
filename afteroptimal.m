tic
num_link=4;
% algorithm4;
% fourlink_after_cal_rel;
dgain=[24;24;24;24];pgain=1/4*[dgain(1)^2;dgain(2)^2;dgain(3)^2;dgain(4)^2];
ini=[q_aft; dq_aft];
desire=[pi/2;-pi;0;0]; effort=zeros(20,1);

for i1 = 1 : 20
    sim('sim_ex_4link_after_rel');
    abs_= zeros(200,num_link);
    for i = 1:num_link
        ang_(:,i) = ScopeData.signals(1,1).values(1:200,i);
        torque(:,i) = ScopeData.signals(1,3).values(1:200,i);
    end
    work=0;

    for j = 1:199
        for k = 1:num_link
            work=work+abs(torque(j,k)*(ang_(j+1,k)-ang_(j,k)));
        end
    end
    effort(i1)=work;
    dgain=[24;24;24;24]+3*(i1-3)*[1;1;1;1];pgain=1/4*[dgain(1)^2;dgain(2)^2;dgain(3)^2;dgain(4)^2];
end
i1=find(effort==min(effort));
dgain=[24;24;24;24]+3*(i1-3)*[1;1;1;1];pgain=1/4*[dgain(1)^2;dgain(2)^2;dgain(3)^2;dgain(4)^2];
sim('sim_ex_4link_after_rel');
toc