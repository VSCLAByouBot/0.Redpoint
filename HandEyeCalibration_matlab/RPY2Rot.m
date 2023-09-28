%  Remark : ZYXR
%===================================================================================
function Rzyx = RPY2Rot(A, B, C)
	%--------------------------------------------------------------------------------------------------------------------------
    %------ Deg to Rad ---------------------------------------------
    DegToRad = pi/180;
    %------ Deg to Rad ---------------------------------------------
	alpha = A*DegToRad;   
	beta  = B*DegToRad;  
	gamma = C*DegToRad;  
    %------ 3x3 matrix ----------------------------------------------
    Rzyx = [ cos(alpha)*cos(beta),  cos(alpha)*sin(beta)*sin(gamma) - cos(gamma)*sin(alpha),    sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta) ;
             cos(beta)*sin(alpha),  cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma),    cos(gamma)*sin(alpha)*sin(beta) - cos(alpha)*sin(gamma) ;
            -sin(beta),             cos(beta)*sin(gamma),                                       cos(beta)*cos(gamma) ];
end
%============================================================================================











