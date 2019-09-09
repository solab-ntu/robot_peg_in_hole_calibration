function f = myobj(x)

rng default % for reproducibility
d = linspace(0,3);
y = exp(-1.3*d) + 0.05*randn(size(d));
f = exp(-d*x)-y;

end