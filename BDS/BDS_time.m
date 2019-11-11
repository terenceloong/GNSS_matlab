function [BD_week, BD_second] = BDS_time(c)
% Convert UTC/GMT+8 to BDT week and BDT second.
% c = [year, month, date, hour, minute, second], UTC/GMT+8

day = datenum(c(1),c(2),c(3)) - datenum(2006,1,1);
BD_week = floor(day/7);
BD_second = (day-BD_week*7)*24*3600 + c(4)*3600 + c(5)*60 + floor(c(6));
BD_second = BD_second - 8*3600 + 4; %BD time is 4 seconds faster than UTC.
if BD_second<0
    BD_second = BD_second+604800;
    BD_week = BD_week-1;
end

end