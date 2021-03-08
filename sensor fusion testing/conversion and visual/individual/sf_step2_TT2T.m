%% TURNING IT INTO A TABLE WHERE EACH OBJECT HAS A SEPERATE COLUMN (sf_clean)
clear all;

load('./mat files/sf_TT.mat');
sf_results_t = struct2table(sf_results);
length_sf = size(sf_results_t,1); %number of rows in sf_results
max_objects = max([sf_results_t.Num_Objects]'); %finding the max number of objects

sf_array = NaN(length_sf,2);
sf_clean = array2table(sf_array);
clear sf_array;

empty = cell(length_sf,1); %making an empty cell array for objects

sf_clean.Properties.VariableNames(1) = {'Time'}; %naming the first column Time
sf_clean.Properties.VariableNames(2) = {'Num_Objects'}; %naming the second column Num_Object
for i  = 1:max_objects
    nameofcolumn = "Object" + i; %naming the rest of the columns by object number
    sf_clean.(nameofcolumn) = empty; %adding the empty column for each object
    sf_clean.Properties.VariableNames(i+2) = nameofcolumn; %adding the object columns name to the table
end

sf_clean.Time = sf_results_t.Time; %fill in time
sf_clean.Num_Objects = sf_results_t.Num_Objects; %filling in number of objects

for i = 1:length_sf
    for j = 1:sf_clean.Num_Objects(i)
        nameofcolumn = "Object" + j;
        sf_clean.(nameofcolumn){i,1} = sf_results_t.Objects{i,1}(1,j); %filling in all the object columns
    end    
end   



save('./mat files/sf_cleanT', 'sf_clean','length_sf');
save('./mat files/range_test3_20mph', 'sf_clean');
