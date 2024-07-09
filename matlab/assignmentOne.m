syms E(t) E_dot E_ddot Phi(t) Phi_dot Phi_ddot Theta(t) Theta_dot Theta_ddot Psi(t) Psi_dot Psi_ddot J_E J_Psi k_s c

E_dot=diff(E,t);
Psi_dot=diff(Psi,t);
Phi_dot=diff(Phi,t);
Theta_dot=diff(Theta,t);


%% Lagrangian Equation

L=0.5*J_E*(E_dot^2+(Phi_dot^2)*(cos(E)^2))+0.5*J_Psi*Phi_dot^2-c*sin(E)-0.5*k_s*Phi^2;

%% First Differential
dLdE=diff(L,E);
dLdPsi=diff(L,Psi);
dLdTheta=diff(L,Theta);
q=[dLdE;dLdPsi;dLdTheta];

%% Second Differential
ddLdtE=diff(dLdE,t);
ddLdtPsi=diff(dLdPsi,t);
ddLdtTheta=diff(dLdTheta,t);
qdot=[ddLdtE;ddLdtPsi;ddLdtTheta];

%% Finding Q
Eddot=ddLdtE-dLdE;

Psiddot=ddLdtPsi-dLdPsi;

Thetaddot=ddLdtTheta-dLdTheta;

Q=q-qdot;


