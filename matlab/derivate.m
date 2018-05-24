x   = VarName1;
y   = VarName2;
dydx= diff([eps;y(:)])./diff([eps;x(:)]);

figure;hold on;
xy=plot(x,y,'b');
xdydx = plot(x,dydx, 'r');
legend([xy, xdydx],["power\_consumption","d(power\_consumption)/dt"]);
