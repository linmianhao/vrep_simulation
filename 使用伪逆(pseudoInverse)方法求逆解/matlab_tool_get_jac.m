%使用matlab 机器人工具箱求雅克比矩阵
%2.创建一个具有2连杆机械臂求它的雅可比矩阵  sdh
fprintf('创建一个具有2连杆机械臂求它的雅可比矩阵')
L(1)=Link('revolute', 'd',0, 'a', 0.5, 'alpha', 0);
L(2)=Link('revolute', 'd', 0, 'a', 0.5, 'alpha', 0);

%3.通过构造函数给创建的机械臂对象命名，并显示出对象的信息
two_link=SerialLink(L,'name',' link')
%机器人正运动学
%1.数组中的参数是每一个关节的角度
% 获得的是坐标系N相对于坐标系0 的变换矩阵

two_link.fkine([0 0 ]);%获得的是T30变换矩阵
%2.将创建的机械臂可视化 数组中的参数是每一个关节的角度
two_link.plot([0 0 ])
q=[0 0];
%求出
J0=two_link.jacob0(q)
    % 求雅克比矩阵的逆和伪逆
	%法1 使用MATLAB函数 利用了svd方法
	J0_pinv_svd=pinv(J0)
    %fprintf('伪逆使用svd：%08.8f\n',J_pinv_svd)
	
	%法2 直接求法
    
	J0_pinv_direct=J0'*inv(J0*J0')
	%fprintf('伪逆使用直接法：%08.8f\n',J_pinv_direct)
	
    %雅克比矩阵求逆
   
	J0_inv=inv(J0)
