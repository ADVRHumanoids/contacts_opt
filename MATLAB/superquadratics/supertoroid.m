function [xo,yo,zo]=supertoroid(varargin)
% [X Y Z]=SUPERTOROID({AXES}, C, R, P, {N})
%   Much like SPHERE(), ELLIPSOID(), or SUPERQUAD(), this
%   function calculates and returns the matrices required to
%   plot a general supertoroid using MESH() or SURF().
%
%   Where the vector C=[Cx Cy Cz] defines the toroid center, 
%   the vector R=[Rx Ry Rz Rm] defines the axial and major radii.
%   The centerline of the volume is a superellipse with radii [Rx Ry]*Rm.
%
%   P=[Pv Ph] defines the curvature of vertical and horizontal sections.
%   If C, R, or P are scalar, the specified value will be used for all axes.
% 
%   As with the built-in functions, the number of points can be set.
%   The default value for N is 50.
% 
%   If no outputs are specified, a new surf plot will be created.
%   Display axes can be specified if the first argument is an axes handle.
%
%   See also: SPHERE, ELLIPSOID, SUPERQUAD, SUPERELLIPSOID


error(nargchk(3,5,nargin));
[ha,inargs,nargs]=axescheck(varargin{:});

[C,R,P]=deal(inargs{1:3});

n=50;
if nargs > 3
	n=inargs{4}; 
end

if numel(C)==1
	C=repmat(C,[1 3]);
end
if numel(R)==1
	R=repmat(R,[1 4]);
end
if numel(P)==1
	P=repmat(P,[1 2]);
end

M=2./P;

u=linspace(-pi,pi,n);
[VV UU]=meshgrid(u,u);

cosV=cos(VV);
sinV=sin(VV); % clamp error at endpoints
cosU=cos(UU);
sinU=sin(UU); % clamp error at endpoints
sinV(:,[1 n])=0;
sinU([1 n],:)=0;

x=C(1)+R(1)*(R(4)+sign(cosV).*abs(cosV).^M(1)) .* sign(cosU).*abs(cosU).^M(2);
y=C(2)+R(2)*(R(4)+sign(cosV).*abs(cosV).^M(1)) .* sign(sinU).*abs(sinU).^M(2);
z=C(3)+R(3)*sign(sinV).*abs(sinV).^M(1);

if(nargout == 0)
    ha=newplot(ha);
	surf(x,y,z,'parent',ha)
else
	xo=x;
	yo=y;
	zo=z;
end





















