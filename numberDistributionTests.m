NUM_PTS = 1000000;
NUM_BINS = 100;

X = rand(NUM_PTS,1);
Y = X.^2;
Z = sqrt(X);
W = X.^3;
S = sin(X);
AS = asin(X);
C = cos(X);
AC = acos(X);
H1 = [X ; Z ; Z ; Z ; Z];
H2 = [X ; X ; Z ; Z ; Z];
H3 = [X ; X ; X ; Z ; Z];
H4 = [X ; X ; X ; X ; Z];
G = (X + Z)/2;
O = readmatrix("precheckDistrib.csv");
D = readmatrix("distribution.csv");

O_X = readmatrix("distrib_x.csv");
O_Y = readmatrix("distrib_y.csv");
O_Z = readmatrix("distrib_z.csv");

F_X = readmatrix("after_x.csv");
F_Y = readmatrix("after_y.csv");
F_Z = readmatrix("after_z.csv");

subplot(3,3,1)
hold on
histogram(X, NUM_BINS, 'Normalization','probability')
histogram(Z, NUM_BINS, 'Normalization','probability')
legend('Uniform','sqrt(X)','Location','northwest')
subplot(3,3,2)
histogram(Y, NUM_BINS, 'Normalization','probability')
legend('X^2')
subplot(3,3,3)
histogram(W, NUM_BINS, 'Normalization','probability')
legend('x^3')
subplot(3,3,4)
hold on
histogram(O_X, NUM_BINS, 'Normalization','probability')
histogram(O_Y, NUM_BINS, 'Normalization','probability')
histogram(O_Z, NUM_BINS, 'Normalization','probability')
legend('B4 x', 'B4 y', 'B4 z','Location','northwest')
subplot(3,3,5)
hold on
histogram(S, NUM_BINS, 'Normalization','probability')
histogram(AS, NUM_BINS, 'Normalization','probability')
legend('sin(x)','asin(x)','Location','northwest')
subplot(3,3,6)
hold on
histogram(C, NUM_BINS, 'Normalization','probability')
histogram(AC, NUM_BINS, 'Normalization','probability')
legend('cos(x)','acos(x)','Location','northwest')
hold on
subplot(3,3,7)
hold on
histogram(H1, NUM_BINS, 'Normalization','probability')
histogram(H2, NUM_BINS, 'Normalization','probability')
histogram(H3, NUM_BINS, 'Normalization','probability')
histogram(H4, NUM_BINS, 'Normalization','probability')
legend('80%', '60%', '40%', '20%','Location','northwest')
subplot(3,3,8)
histogram(G, NUM_BINS, 'Normalization','probability')
legend('avg(sqrt(x), Uniform)','Location','northwest')
subplot(3,3,9)
hold on
histogram(F_X, NUM_BINS, 'Normalization','probability')
histogram(F_Y, NUM_BINS, 'Normalization','probability')
histogram(F_Z, NUM_BINS, 'Normalization','probability')
legend('Post x', 'Post y', 'Post z','Location','northwest')

%hold on
%X_TEST = 0:0.01:1;
%Y_TEST = X_TEST.^-1;
%plot(X_TEST,Y_TEST);