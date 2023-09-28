%  Rotation Matrix to Euler Angle
%===================================================================================
function [A1, B1, C1] = Rot2RPY(matr3by3)

%------ Rad to Deg ---------------------------------------------
RadToDeg = 180/pi;

%----- judge ABC Posture --------------------------------------------------------------
if ( (abs(matr3by3(1,1))-1.7e-5) < 0 && (abs(matr3by3(2,1))-1.7e-5) < 0)
    
    if (matr3by3(3,1) >= 0)
        B1 = -pi/2;
        A1 = 0;
        C1 = atan2(-matr3by3(2,3), -matr3by3(1,3));
    elseif (matr3by3(3,1) < 0)
        B1 = pi/2;
        A1 = 0;
        C1 = atan2(matr3by3(2,3), matr3by3(1,3));
    end
else
    B1 = atan2(-matr3by3(3,1),  sqrt(matr3by3(1,1)*matr3by3(1,1) + matr3by3(2,1)*matr3by3(2,1)));
    
    if ((cos(B1) + 0.9999999) < 0)
        B1 = B1 + 0.0011111;
    end
    if (cos(B1) >= 0.0111111) %%
        A1 = atan2(matr3by3(2,1), matr3by3(1,1));
        C1 = atan2(matr3by3(3,2), matr3by3(3,3));
    elseif (cos(B1) < 0.0111111)
        A1 = atan2(-matr3by3(2,1), -matr3by3(1,1));
        C1 = atan2(-matr3by3(3,2), -matr3by3(3,3));
    end
end


A1 = A1*RadToDeg;
B1 = B1*RadToDeg;
C1 = C1*RadToDeg;

if (abs(round(A1)) == 180)
    A1 = 180;
else
    A1 = round(A1 * 1000)/1000;
end
if (abs(round(C1)) == 180)
    C1 = 180;
else
    C1 = round(C1 * 1000)/1000;
end

end
%============================================================================================











