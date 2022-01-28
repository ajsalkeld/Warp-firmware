X = csvread("X.csv");
y = csvread("y.csv");

% Make model from TPH and MQ-135
[b, bint, r, rint, stats] = regress(y,X);
%rcoplot(r,rint);

b

% Model only using MQ-135, for when BME280 unavailable
[b2, bint2, r2, rint2, stats2] = regress(y,X(:,[1,end]));
% rcoplot(r2,rint2);

b2