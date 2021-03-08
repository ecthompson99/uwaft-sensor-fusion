clear all
load('rangeraw_test2_10mph.mat'); 

a = load('rangeraw_test3_20mph.mat','relpos_oxts');
relpos_oxts2 = a.relpos_oxts;
a = load('rangeraw_test3_20mph.mat','sf_clean');
sf_clean2 = a.sf_clean;
a = load('rangeraw_test3_20mph.mat','max_objects');
max_objects2 = a;


%-------------------------------------------------------------------------
%OXTS
oxts_range = NaN(length(relpos_oxts),3); %make an empty array of NaN values
oxts_range(:,1) = relpos_oxts(:,1); %adding time to the first column
for i = 1:length(relpos_oxts)
    oxts_range(i,2) = sqrt(relpos_oxts(i,1)^2 + relpos_oxts(i,2)^2); %calculate and add in range to the second column
    if i > 1 
        oxts_range(i,3) = (oxts_range(i,2) - oxts_range(i-1,2))/(oxts_range(i,1) - oxts_range(i-1,1)); %calculate range rate and add it to the third column
    end
end

%SENSOR FUSION
max_objects = max(sf_clean.Num_Objects);%max objects detected
sf_range = NaN(size(sf_clean,1),max_objects*2+1); %make an empty array of NaN values
sf_range(:,1) = sf_clean.Time;%adding time to the first column


for j = 1:max_objects %repeat for the amount of objects
    prev = 0; 
    for i = 1:size(sf_clean,1) %loop through all the rows
        if not(isempty(sf_clean{i,j+2}{1})) %only if the cell has a value
            sf_range(i, j*2) = sqrt(sf_clean{i,j+2}{1}.Measurement(1)^2 + sf_clean{i,j+2}{1}.Measurement(2)^2); %calculate and add in range
            if (i > 1) & (prev ~= 0) %only if there has been a previous range value been read 
                %calculates range rate using the last range value read in
                %(not NaN) and then adds it in to the specified column
                sf_range(i, (j*2)+1) = (sf_range(i,j*2) - sf_range(prev,j*2))/(sf_range(i,1) - sf_range(prev,1)); 
            end
            prev = i; %keeps track of the index at which the last range value was read
        end
    end
end

%making string array for names to be put in legend for first test
legend_names = strings(1,1+ max_objects);
legend_names(1,1) = "OXTS";
for i = 1:max_objects
    legend_names(1,i+1) = "Object" + i;
end


%first plot for Rate vs Time for 10mph
subplot(2,2,1);
plot (oxts_range(:,1), oxts_range(:,2));
for i = 1:max_objects
    hold on; 
    plot (sf_range(:,1), sf_range(:,i*2));
    hold off;
end
legend(legend_names); 
title('Range vs Time for 10mph');

%second plot for 'Range Rate vs Time for 10mph
subplot(2,2,2);
plot (oxts_range(:,1), oxts_range(:,3));
for i = 1:max_objects
    hold on; 
    plot (sf_range(:,1), sf_range(:,(i*2)+1));
    hold off;
end   
legend(legend_names);
title('Range Rate vs Time for 10mph');

%-------------------------------------------------------------------------
%OXTS
oxts_range2 = NaN(length(relpos_oxts2),3);%make an empty array of NaN values
oxts_range2(:,1) = relpos_oxts2(:,1); %adding time to the first column
for i = 1:length(relpos_oxts2)
    oxts_range2(i,2) = sqrt(relpos_oxts2(i,1)^2 + relpos_oxts2(i,2)^2);%calculate and add in range to the second column
    if i > 1
        oxts_range2(i,3) = (oxts_range2(i,2) - oxts_range2(i-1,2))/(oxts_range2(i,1) - oxts_range2(i-1,1));%calculate range rate and add it to the third column
    end
end

%SENSOR FUSION
max_objects2 = max(sf_clean2.Num_Objects);%max objects detected
sf_range2 = NaN(size(sf_clean2,1),max_objects2*2+1);%make an empty array of NaN values
sf_range2(:,1) = sf_clean2.Time;%adding time to the first column


for j = 1:max_objects2 %repeat for the amount of objects
    prev = 0;
    for i = 1:size(sf_clean2,1) %loop through all the rows
        if not(isempty(sf_clean2{i,j+2}{1})) %only if the cell has a value
            sf_range2(i, j*2) = sqrt(sf_clean2{i,j+2}{1}.Measurement(1)^2 + sf_clean2{i,j+2}{1}.Measurement(2)^2); %calculate and add in range
            if (i > 1) & (prev ~= 0)
                %calculates range rate using the last range value read in
                %(not NaN) and then adds it in to the specified column
                sf_range2(i, (j*2)+1) = (sf_range2(i,j*2) - sf_range2(prev,j*2))/(sf_range2(i,1) - sf_range2(prev,1));
            end
            prev = i; %keeps track of the index at which the last range value was read
        end
    end
end

%making string array for names to be put in legend for second test
legend_names = strings(1,1+ max_objects2);
legend_names(1,1) = "OXTS";
for i = 1:max_objects2
    legend_names(1,i+1) = "Object" + i;
end

%third plot for 'Range vs Time for 20mph
subplot(2,2,3);
plot (oxts_range2(:,1), oxts_range2(:,2));
for i = 1:max_objects2
    hold on; 
    plot (sf_range2(:,1), sf_range2(:,i*2));
    hold off;
end
legend(legend_names);
title('Range vs Time for 20mph');

%fourth plot for 'Range Rate vs Time for 20mph
subplot(2,2,4);
plot (oxts_range2(:,1), oxts_range2(:,3));
for i = 1:max_objects2
    hold on; 
    plot (sf_range2(:,1), sf_range2(:,(i*2)+1));
    hold off;
end   
legend(legend_names);
title('Range Rate vs Time for 20mph');


