%
% SSS.m
%
% find interior angle of a triangle with 3 sides given, using law of cosine
%
% input: L=3x3 vector -- coordinates of each vertex
%
% output: q=3x1 vector of the angles facing first column of L, second
% colume of L, and third column of L
%
function q=SSS(L)

a=norm(L(:,2)-L(:,3));
b=norm(L(:,1)-L(:,3));
c=norm(L(:,1)-L(:,2));
q=zeros(3,1);
q(1)=acos((b^2+c^2-a^2)/(2*b*c));
q(2)=acos((a^2+c^2-b^2)/(2*a*c));
q(3)=acos((a^2+b^2-c^2)/(2*a*b));

end