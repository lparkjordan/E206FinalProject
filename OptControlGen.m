A = [0 0 1 0;
     0 0 0 1;
     0 0 0 0;
     0 0 0 0];
B = [0 0;
     0 0;
    -1 0;
    0 -1];
Q = [500000 0 0 0;
     0 500000 0 0
     0 0 100 0
     0 0 0 100];
R = [0.1 0;
     0 0.1];
N = 0;

[K, S, E] = lqr(A, B, Q, R, N);
K