
radar_obj = 1;
radar_index = 1;
while radar_index < 20416
    if radar_obj > 32
        radar_obj = 1;
    end

    fields_A=fieldnames(radar_final.Signals{radar_index,1});
    dx_radar(radar_index, radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{7});
    dy_radar(radar_index, radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{6});

    radar_index = radar_index + 2;
    radar_obj = radar_obj + 1;
    

end

writematrix(dx_radar,'./Excel files/radar_test.xlsx','Sheet',1);
writematrix(dy_radar,'./Excel files/radar_test.xlsx','Sheet',2);

me_obj = 1;
me_index = 1;
while me_index < 498
    if me_obj > 10
        me_obj = 1;
    end

    fields_A_me=fieldnames(me_final.Signals{me_index,1});
    dx_mobileye(me_index, me_obj) = me_final.Signals{me_index,1}.(fields_A_me{9});
    dy_mobileye(me_index, me_obj) = me_final.Signals{me_index,1}.(fields_A_me{8});

        
    me_index = me_index + 3;
    me_obj = me_obj + 1;
    

end

writematrix(dx_mobileye,'./Excel files/me_test.xlsx','Sheet',1);
writematrix(dy_mobileye,'./Excel files/me_test.xlsx','Sheet',2);

