function [ Info , EulerAngle , Position ] = ForwardKinemetics( DH_table , JointAngle)

% 讀取DH參數
dof = size( DH_table , 1 ) ;

theta = DH_table( : , 1 ) ;
d     = DH_table( : , 2 ) ;
a     = DH_table( : , 3 ) ;
alpha = DH_table( : , 4 ) ;

% 初始值
Info.JointPos = [0 ;0 ;0];                   % 各關節的座標位置矩陣 [ 3 x n  ], n = DOF
Info.JointDir = [1 0 0;                      % 各關節的座標向量矩陣 [ 3 x 3n ], n = DOF
                 0 1 0; 
                 0 0 1];
             
% 單位矩陣
T = eye( 4 , 4 ) ;
for i = 1 : dof
    
   A =  [ cos(JointAngle(i)+theta(i))        -1*sin(JointAngle(i)+theta(i))*cos(alpha(i))       sin(JointAngle(i)+theta(i))*sin(alpha(i))        a(i)*cos(JointAngle(i)+theta(i)) ;
          sin(JointAngle(i)+theta(i))        cos(JointAngle(i)+theta(i))*cos(alpha(i))          -1*cos(JointAngle(i)+theta(i))*sin(alpha(i))     a(i)*sin(JointAngle(i)+theta(i)) ;
                    0                        sin(alpha(i))                                      cos(alpha(i))                                                   d(i)              ;
                    0                                               0                                               0                                            1              ] ;
  
   T = T * A ;
   
   Info.JointPos = [ Info.JointPos T( 1:3, 4 ) ];                                         % 儲存關節的座標位置
   Info.JointDir = [ Info.JointDir T( 1:3, 1:3 ) ];                                       % 儲存關節的向量資訊
end

% wrist center position
End_Position = T( 1 : 3 , 4 ) ;

% wrist center direction
Wrist_Center_R = T( 1 : 3 , 1 : 3 ) ;

% 末端點位置
Position(1) = End_Position(1) ;
Position(2) = End_Position(2) ;
Position(3) = End_Position(3) ;


% 取 EulerAngle z-y-x
[EulerAngle(1), EulerAngle(2), EulerAngle(3)] = Rot2RPY(Wrist_Center_R);

end