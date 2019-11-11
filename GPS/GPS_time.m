function [GPS_week, GPS_second] = GPS_time(c)
% Convert UTC/GMT+8 to GPS week and GPS second.
% c = [year, month, date, hour, minute, second], UTC/GMT+8

% day = datenum(c(1),c(2),c(3)) - datenum(1999,8,22);
% day = datenum(c(1),c(2),c(3)) - datenum(2019,4,7);

if c(1)*10000+c(2)*100+c(3)>=20190407
    day = datenum(c(1),c(2),c(3)) - datenum(2019,4,7);
else
    day = datenum(c(1),c(2),c(3)) - datenum(1999,8,22);
end
GPS_week = floor(day/7);
GPS_second = (day-GPS_week*7)*24*3600 + c(4)*3600 + c(5)*60 + floor(c(6));
GPS_second = GPS_second - 8*3600 + 18; %GPS time is 18 seconds faster than UTC.
if GPS_second<0
    GPS_second = GPS_second+604800;
    GPS_week = GPS_week-1;
end

end