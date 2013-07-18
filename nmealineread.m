function [data,ierr]  =  nmealineread(nline)
%NMEALINEREAD reads an NMEA sentence into a MATLAB structure array
%
%  DATA  =  NMEALINEREAD(NLINE)
%  [DATA,IERR]  =  NMEALINEREAD(NLINE)
%
%  NLINE is an NMEA sentence. DATA is a MATLAB structure array with a
%  varying format, detailed below.
%
%  NMEALINEREAD currently supports the following NMEA sentences:
%                   $GPGGA  Global positioning system fixed data
%                   $GPGLL  Geographic poition [latitude, longitude & time]
%                   $GPRMC  Recommended Minimum [position,velocity]
%                   $GPVTG  Course over ground and ground speed
%                   $GPZDA  UTC date / time and local time zone offset
%                   $SDDBS  Echo sounder data
%
%  $GPGGA gives a DATA structure with the following fields:
%       Time        Day fraction
%       latitude    Decimal latitude north
%       longitude   Decimal longitude east
%
%  $GPGLL gives a DATA structure with the following fields:
%       Time    Day fraction
%       latitude    Decimal latitude north
%       longitude   Decimal longitude east
%
%  $GPVTG gives a DATA structure with the following fields:
%       speed       Speed over ground in knots
%       truecourse  Course over ground in degrees true
%
%  $GPZDA gives a DATA structure with the following fields:
%       Time    Day number
%       offset      The offset (as a fraction of a day) needed to 
%                   generate the local time from BODCTime
%
%  $SDDBS gives a DATA structure with the following field:
%       depth       Depth of water below surface (m)
%
%
%  IERR returns an error code:
%       -2  -  NMEA string recognised, but function not yet able to read
%              this string
%       -1  -  NMEA string not recognised
%       0   -  No errors

% Adam Leadbetter (alead@bodc.ac.uk) - 2006October24

ierr  =  0;

%
%  Set up a list of valid NMEA strings
%
nmea_options  =  ['$GPGGA'
                  '$GPGLL'
                  '$GPGSA'
                  '$GPGSV'
                  '$GPRMC'
                  '$GPVTG'
                  '$GPZDA'
                  '$SDDBS'];
%
%  Find which string we're dealing with
%
kk  =  1;
case_t  =  -99;
while(kk  <=  length(nmea_options(:,1)))
  if(strfind(nline,nmea_options(kk,:)))
    case_t  =  kk;
    kk  =  length(nmea_options(:,1)) + 1;
  end
  kk  =  kk + 1;
end
%
%  If no valid NMEA string found - quit with an error
%
if(case_t  ==  -99)
  fprintf(1,'\n\tWarning: Not a valid NMEA string  -  %s ...\n',nline);
  data  =  NaN;
  ierr  =  -1;
  return
end
%
%  Trim down to the beginning of the data stream
%
nline  =  nline(find(nline  ==  '$',1,'first') + 7 : end);
switch case_t
  case 1
%
%  Read global positioning system fixed data
%
    data.type = 'GGA';
    t_time  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    if(isempty(t_time))
      data.BODCTime  =  NaN;
    else
      data.BODCTime  =  datenum(t_time,'HHMMSS.FFF') - ...
        floor(datenum(t_time,'HHMMSS.FFF'));
    end
    clear t_time;
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    t_lat  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    data.latitude  =  ...
      str2double(t_lat(1:2)) + (str2double(t_lat(3:end))/60);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    if(nline(1)  ==  'S')
      data.latitude  =  data.latitude  *  -1;
    end
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    t_lat  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    data.longitude  =  ...
      str2double(t_lat(1:3)) + (str2double(t_lat(4:end))/60);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    if(nline(1)  ==  'W')
      data.longitude  =  data.longitude  *  -1;
    end
    clear t_lat;
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    data.quality  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    data.quality  =  str2num(data.quality);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    data.satellites  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    data.satellites  =  str2num(data.satellites);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    data.hdilution  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    data.hdilution  =  str2double(data.hdilution);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    data.hMSL  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    data.hMSL  =  str2double(data.hMSL);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    data.hWGS  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    data.hWGS  =  str2double(data.hWGS);
    
  case 2
%
%  Read geographic position [lat/lon] and time
%
    data.type = 'GLL';
    t_lat  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    data.latitude  =  str2double(t_lat(1:2)) + ...
      (str2double(t_lat(3:end)) / 60);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    t_lah  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    if(t_lah  ==  'S')
      data.latitude  =  data.latitude * -1;
    end
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    t_lon  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    data.longitude  =  str2double(t_lon(1:3)) + ...
      (str2double(t_lon(4:end)) / 60);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    if(nline(1)  ==  'W')
      data.longitude  =  data.longitude  *  -1;
    end
    if(find(nline  ==  ','))
      nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
      t_time  =  nline(1:find(nline  ==  ',',1,'first') - 1);
      data.BODCTime  =  datenum(t_time,'HHMMSS') - ...
        floor(datenum(t_time,'HHMMSS'));
    else
      data.BODCTime  =  NaN;
    end
  case 5
%
% Read recommended minimum
%
    data.type = 'RMC';
    t_time  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    t_status  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    data.status = t_status;
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    t_lat  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    data.latitude  =  ...
      str2double(t_lat(1:2)) + (str2double(t_lat(3:end))/60);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    if(nline(1)  ==  'S')
      data.latitude  =  data.latitude  *  -1;
    end
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    t_lat  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    data.longitude  =  ...
      str2double(t_lat(1:3)) + (str2double(t_lat(4:end))/60);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    if(nline(1)  ==  'W')
      data.longitude  =  data.longitude  *  -1;
    end
    clear t_lat;
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    data.speed  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    data.speed  =  str2double(data.speed);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    t_course  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    if(isempty(t_course))
      data.truecourse  =  NaN;
    else
      data.truecourse  =  str2double(t_course);
    end
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    t_date =  nline(1:find(nline  ==  ',',1,'first') - 1);
    if(isempty(t_time))
      data.BODCTime  =  NaN;
    else
      data.BODCTime  =  datenum(strcat(t_time,t_date),'HHMMSS.FFFddmmyy');
    end
    clear t_time;
    clear t_date;
    
  case 6
%
%  Read course over ground and ground speed
%
    data.type = 'VTG';
    t_course  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    if(isempty(t_course))
      data.truecourse  =  NaN;
    else
      data.truecourse  =  str2double(t_course);
    end
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    nline  =  nline(find(nline  ==  ',',1,'first') + 1:end);
    data.speed  =  nline(1:find(nline  ==  ',',1,'first') - 1);
    data.speed  =  str2double(data.speed);
  case 7
%
%  Read UTC Date / Time and Local Time Zone Offset
%
    data.type = 'ZDA';
    data.BODCTime  =  (datenum(nline(11:20),'dd,mm,yyyy') + ...
      (datenum(nline(1:6),'HHMMSS') - ...
      floor(datenum(nline(1:6),'HHMMSS'))));
    data.offset  =  (str2double(nline(22:23)) + ...
      (str2double(nline(25:26)) / 60)) / 24;
  case 8
%
%  Read echo sounder data
%
    data.type = 'DBS';
    com_mask  =  strfind(nline,',');
    data.depth  =  str2double(...
      nline(com_mask(2) + 1: com_mask(3)-1));
  otherwise
    data  =  NaN;
    ierr  =  -2;
    fprintf(1,...
      '\n\tWarning: NMEA reader not yet implemented for this string  -  %s  ...\n',...
      nline);
end
%
%  Tidy up the output structure
%
data  =  orderfields(data);