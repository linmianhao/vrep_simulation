%这个函数是用来获得雅克比矩阵 伪逆雅克比矩阵 以及雅克比矩阵的逆
function [J,J_pinv_svd,J_pinv_direct,J_inv]=getPseudoInverse(theta1,theta2)
 
    %机械臂数据  
	%这里怎么保证精度 怎么保证精确到小数点的哪一位需要考虑一下
	l1=0.5;  %连杆程度 单位是米
	l2=0.5;  %连杆程度 单位是米

    %获得机械臂的雅克比矩阵
	J11=-l1*sin(theta1)-l2*sin(theta1+theta2);
	J12=-l2*sin(theta1+theta2);
	J21= l1*cos(theta1)+l2*cos(theta1+theta2);
	J22= l2*cos(theta1+theta2);
	J=[J11,J12;J21,J22];
	%获得雅克比矩阵的伪逆
	%在雅克比矩阵不是奇异的时候二者是相等的（此雅克比矩阵正好是一个方阵）
	%法1 使用MATLAB函数 利用了svd方法
	J_pinv_svd=pinv(J);
    %fprintf('伪逆使用svd：%08.8f\n',J_pinv_svd)
	
	%法2 直接求法
    
	J_pinv_direct=J'*inv(J*J');
	%fprintf('伪逆使用直接法：%08.8f\n',J_pinv_direct)
	
    %雅克比矩阵求逆
    
	J_inv=inv(J);
	%fprintf('雅克比矩阵求逆： %08.8f\n',J_inv)
	
end
%结果：
%在Matlab中,inf为无穷大量+∞,-inf为无穷小量-∞
%NAN Not A Number就是代表不是一个数据
%   [J,J_pinv_svd,J_pinv_direct,J_inv]=getPseudoInverse(0,0)
%   J_pinv_svd =
% 
%                    0   0.800000000000000
%                    0   0.400000000000000
% 
% 
% J_pinv_direct =
% 
%    NaN   NaN
%    NaN   NaN
% 
% 
% J_inv =
% 
%    Inf   Inf
%    Inf   Inf