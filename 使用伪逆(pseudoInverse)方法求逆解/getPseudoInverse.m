%�����������������ſ˱Ⱦ��� α���ſ˱Ⱦ��� �Լ��ſ˱Ⱦ������
function [J,J_pinv_svd,J_pinv_direct,J_inv]=getPseudoInverse(theta1,theta2)
 
    %��е������  
	%������ô��֤���� ��ô��֤��ȷ��С�������һλ��Ҫ����һ��
	l1=0.5;  %���˳̶� ��λ����
	l2=0.5;  %���˳̶� ��λ����

    %��û�е�۵��ſ˱Ⱦ���
	J11=-l1*sin(theta1)-l2*sin(theta1+theta2);
	J12=-l2*sin(theta1+theta2);
	J21= l1*cos(theta1)+l2*cos(theta1+theta2);
	J22= l2*cos(theta1+theta2);
	J=[J11,J12;J21,J22];
	%����ſ˱Ⱦ����α��
	%���ſ˱Ⱦ����������ʱ���������ȵģ����ſ˱Ⱦ���������һ������
	%��1 ʹ��MATLAB���� ������svd����
	J_pinv_svd=pinv(J);
    %fprintf('α��ʹ��svd��%08.8f\n',J_pinv_svd)
	
	%��2 ֱ����
    
	J_pinv_direct=J'*inv(J*J');
	%fprintf('α��ʹ��ֱ�ӷ���%08.8f\n',J_pinv_direct)
	
    %�ſ˱Ⱦ�������
    
	J_inv=inv(J);
	%fprintf('�ſ˱Ⱦ������棺 %08.8f\n',J_inv)
	
end
%�����
%��Matlab��,infΪ�������+��,-infΪ����С��-��
%NAN Not A Number���Ǵ�����һ������
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