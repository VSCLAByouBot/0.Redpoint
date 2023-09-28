function [A2B, B2A] = Transform_Matrix_SVD(A, B, n)
% Data 1: A
% Data 2: B
% Data Number: n

sz    = size(A);
order = sz(sz~=n);

% Reshape 
A = reshape(A, [order, n]);
B = reshape(B, [order, n]);

[B2A] = GetTransformation(A, B, n);
[A2B] = GetTransformation(B, A, n);

    function [Data2_to_Data1] = GetTransformation(Data1, Data2, n)
        
        % 建立資料中心點
        center_Data1 = sum(Data1, 2)/ n;
        center_Data2 = sum(Data2, 2)/ n;

        % 定義 M 矩陣
        M = zeros(3,3);
        for i = 1 : n
            M = M + (Data1(:,i) - center_Data1) * (Data2(:,i) - center_Data2)';
        end

        % 對 M 矩陣進行奇異值分解
        [U, ~, V] = svd(M);

        % A -> B
        tmp = [1, 0, 0; 
               0, 1, 0; 
               0, 0, det(U * V')];

        Data2_R_Data1 = U * tmp * V';
        Data2_T_Data1 = center_Data1 - Data2_R_Data1 * center_Data2;

        Data2_to_Data1 = [Data2_R_Data1, Data2_T_Data1; 
                          0, 0, 0,       1];
    end
end