function [x,y,z] = LLA2ECEF(lat,long,alt)
%Converts LLA to ECEF
%   See http://microem.ru/files/2012/08/GPS.G1-X-00006.pdf

    %WGS84 constants
    a = 6378137.0;
    b = 6356752.314245;
    e = sqrt((a^2 - b^2)/a^2);

    %radius of curvature
    N = a / sqrt(1 - e^2 * (sind(lat))^2);
    
    %calculate coordinates
    x = (N + alt) * cosd(lat) * cosd(long);
    y = (N + alt) * cosd(lat) * sind(long);
    z = ((b/a)^2 * N + alt) * sind(lat);

end

