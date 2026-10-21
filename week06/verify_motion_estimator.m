function verify_motion_estimator(csvPath)
% Compare host C++ estimator with independent double-precision MATLAB model.
% Generate CSV using the commands in README_motion_estimator.md.
if nargin < 1, csvPath='/tmp/motion-check.csv'; end
data=readmatrix(csvPath); x=zeros(8,1);
P=diag([0 .1 1 0 .1 1 .01 .001]);
maxState=0; maxCov=0;
for k=1:size(data,1)
    if mod(k-1,2)==0, dt=.009; else, dt=.011; end
    Fc=[1 dt dt^2/2;0 1 dt;0 0 1]; F=blkdiag(Fc,Fc,1,1);
    Qc=[dt^5/20 dt^4/8 dt^3/6;dt^4/8 dt^3/3 dt^2/2;dt^3/6 dt^2/2 dt];
    Q=blkdiag(Qc,4*Qc,1e-6*dt,1e-8*dt);
    x=F*x; P=F*P*F'+Q;
    if k<=500, v=.2; else, v=-.2; end
    H=zeros(4,8); H(1:2,2)=9.6/.0315;
    H(1:2,5)=[-1;1]*9.6*.14/(2*.0315);
    H(3,[3 7])=1; H(4,[5 8])=1;
    z=[H(1:2,2)*v+H(1:2,5)*.3;.03;.31]; R=[1 1 .04 .0001];
    for j=1:4
        h=H(j,:); gain=P*h'/(h*P*h'+R(j));
        x=x+gain*(z(j)-h*x); M=eye(8)-gain*h;
        P=M*P*M'+gain*R(j)*gain'; P=(P+P')/2;
    end
    maxState=max(maxState,max(abs(x-data(k,2:9)')));
    actualP=reshape(data(k,10:end),8,8)';
    maxCov=max(maxCov,max(abs(P-actualP),[],'all'));
end
assert(maxState<2e-4 && maxCov<2e-4,'C++/MATLAB mismatch');
fprintf('Motion reference passed: max state error %.3g, covariance error %.3g\n',maxState,maxCov);
end
