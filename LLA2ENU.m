function [xyz] = LLA2ENU(lat,long,alt)
%Converts LLA to ECEF
%   See http://microem.ru/files/2012/08/GPS.G1-X-00006.pdf and https://en.wikipedia.org/wiki/East_north_up#From_ECEF_to_ENU

    %WGS84 constants
    a = 6378137.0;
    b = 6356752.314245;
    e = sqrt((a^2 - b^2)/a^2);

    %radius of curvature
    N = a / sqrt(1 - e^2 * (sind(lat))^2);
    
    %calculate coordinates
    x_ecef = (N + alt) * cosd(lat) * cosd(long);
    y_ecef = (N + alt) * cosd(lat) * sind(long);
    z_ecef = ((b/a)^2 * N + alt) * sind(lat);
    
    %set a reference point if there is none
    global enu_ref
    if isempty(enu_ref)
        enu_ref = [x_ecef,y_ecef,z_ecef,lat,long,alt];
    end
    
    %convert to ENU coordinates
    DCM = [-sind(enu_ref(5)) , cosd(enu_ref(5)) , 0; ...
            -sind(enu_ref(4))*cosd(enu_ref(5)) , -sind(enu_ref(4))*sind(enu_ref(5)) , cosd(enu_ref(4)); ...
            cosd(enu_ref(4))*cosd(enu_ref(5)) , cosd(enu_ref(4))*sind(enu_ref(5)) , sind(enu_ref(4))];
    
    xyz = DCM * [x_ecef - enu_ref(1), y_ecef - enu_ref(2), z_ecef - enu_ref(3)]';
end

