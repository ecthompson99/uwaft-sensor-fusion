%for oxts
load('range_test2_10mph.mat'); %range is a column vector with range and
ideal_time2 = load('range_test3_20mph.mat','ideal_time');
range2 = load('range_test3_20mph.mat','range');
sf_range2 = load('range_test3_20mph.mat','sf_range');
sf_rangeRate2 = load('range_test3_20mph.mat','sf_rangeRate');

increment = 0.01; %time increment used in data

%for oxts-----------------------------------------------
range_rate = zeros(length(ideal_time),1); %for oxts
for j = 1000:(length(ideal_time)-1)
    range_rate(j) = (range(j+1) - range(j))/increment;
end

range_rate2 = zeros(length(ideal_time2.ideal_time),1); %for oxts
for j = 1000:(length(ideal_time2.ideal_time)-1)
    range_rate2(j) = (range2.range(j+1) - range2.range(j))/increment;
end


%first plot for Rate vs Time for 10mph
subplot(2,2,1);
plot (ideal_time, range);
for i = 1:max_objects
    hold on; 
    plot (ideal_time, sf_range(:,i));
    hold off;
end    
legend('oxts','object 1','object 2');%change legend depending on the value of max_objects
title('Range vs Time for 10mph');

%second plot for 'Range Rate vs Time for 10mph
subplot(2,2,2);
plot (ideal_time, range_rate);
for i = 1:max_objects
    hold on; 
    plot (ideal_time, sf_rangeRate(:,i));
    hold off;
end    
legend('oxts','object 1','object 2');%change legend depending on the value of max_objects
title('Range Rate vs Time for 10mph');

%third plot for 'Range vs Time for 20mph
subplot(2,2,3);
plot (ideal_time2.ideal_time, range2.range);
for i = 1:max_objects2
    hold on; 
    plot (ideal_time2.ideal_time, sf_range2.sf_range(:,i));
    hold off;
end   
legend('oxts','object 1','object 2','object 3');%change legend depending on the value of max_objects
title('Range vs Time for 20mph');

%fourth plot for 'Range Rate vs Time for 20mph
subplot(2,2,4);
plot (ideal_time2.ideal_time, range_rate2);
for i = 1:max_objects2
    hold on; 
    plot (ideal_time2.ideal_time, sf_rangeRate2.sf_rangeRate(:,i));
    hold off;
end   
legend('oxts','object 1','object 2','object 3');%change legend depending on the value of max_objects
title('Range Rate vs Time for 20mph');

range_2 = range2.range;

save('rangedata.mat','range', 'range_2', 'range_rate', 'range_rate2');

