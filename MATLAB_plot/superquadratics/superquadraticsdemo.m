% SUPERQUADRATICS DEMOS

% these demos only show a restricted set of cases for simplicity
% herein, radii are kept symmetric, centers are at [0 0 0]
% and superellipsoids are plotted with Px=Py

% superellipsoids
figure(1)
clc; clf

px=[0.5 1 2 4 20];
py=[0.5 1 2 100];

npx=numel(px);
npy=numel(py);

for m=1:npy
	for n=1:npx
		[x y z]=superellipsoid(0,1,[px(n) px(n) py(m)],100);
		
		ha=subplot_tight(npy,npx,(m-1)*npx+n);
		h=surf(x,y,z,'parent',ha);
		set(h,'edgealpha',0,'specularstrength',1);
		lighting gouraud
		camlight
		colormap winter
		axis equal 
		axis square
		axis off
		th=title(sprintf('P=%s',mat2str([px(n) px(n) py(m)])));
		set(th,'color',[1 1 1],'fontweight','bold')
		tp=get(th,'position');
		set(th,'position',[tp(1:2) 1.8])
	end
end
set(gcf,'color',[1 1 1]*0.4)


% supertoroids
figure(2)
clf

px=[0.5 1 2 4 8];
py=[0.5 1 2 4];

npx=numel(px);
npy=numel(py);

for m=1:npy
	for n=1:npx
		[x y z]=supertoroid(0,[1 1 1 2],[py(m) px(n)],100);
		
		ha=subplot_tight(npy,npx,(m-1)*npx+n);
		h=surf(x,y,z,'parent',ha);
		set(h,'edgealpha',0,'specularstrength',1);
		lighting gouraud
		camlight
		colormap winter
		axis equal 
		axis off
		th=title(sprintf('P=%s',mat2str([py(m) px(n)])));
		set(th,'color',[1 1 1],'fontweight','bold')
	end
end
set(gcf,'color',[1 1 1]*0.4)










