function result =   Co_Unco_rrelatedGaussianNoise(Q)
%% generates correlated or uncorrelated 0-mean Gaussian vector process
%
% inputs: Q - correlation matrix. must be positive definite
%               and size determines output vector
%
% output: result - vector of dimension Q rows.
%% algorithm

% Check dimenions - add other checking as necessary...
if(ndims(Q) ~= 2)
    error('Rpp must be a real-valued symmetric matrix');
end

% Symmeterize the correlation matrix
Q = 0.5 .*(double(Q) + double(Q'));

% Eigen decomposition
[V,D] = eig(Q);

% Check for positive definiteness
if(any(diag(Q) <= 0))
    error('Q must be a positive definite');
end

% Form correlating filter
W = V*sqrt(D);

% White noise sample
n = randn(size(Q,1), 1);

% Correlated (colored) noise
result = W * n;