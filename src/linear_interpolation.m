function yout = linear_interpolation(x1,y1,x2,y2,xout)

k = (y2-y1)/(x2-x1);
m = y1 - k*x1;
yout = k*xout + m;

% xout = (yout - (y1-(y2-y1)/(x2-x1)*x1))/((y2-y1)/(x2-x1));