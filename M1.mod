param Nfe := 100;
param Nv := 3;
param tf := 40;
param dt = tf / Nfe;
param Nobs;
param OC{i in {1..Nobs}, j in {0..Nfe}, k in {1..2}};
param InitialValue{i in {1..Nv}, j in {1..6}};
param TerminalLine{i in {1..Nv}, j in {1..4}, k in {1..3}};
param TerminalLocations{i in {1..Nv}, j in {1..4}, k in {1..2}};
param M{i in {1..Nv}, j in {1..4}};

param amax := 1.0;
param vmax := 3.0;
param wmax := 1.0;
param phymax := 0.7;
param L_tractor_front_hang := 0.25;
param L_tractor_wheelbase := 1.5;
param L_tractor_rear_hang := 0.25;
param LHW := 1;
param L := 3.0;
param L_trailer_front_hang := 1;
param L_trailer_rear_hang := 1;

##### Decision variabes besides tf #####
var x{i in {1..Nv}, j in {1..4}, k in {0..Nfe}};
var y{i in {1..Nv}, j in {1..4}, k in {0..Nfe}};
var v{i in {1..Nv}, j in {1..4}, k in {0..Nfe}};
var theta{i in {1..Nv}, j in {1..4}, k in {0..Nfe}};
var xc{i in {1..Nv}, j in {1..4}, k in {0..Nfe}};
var yc{i in {1..Nv}, j in {1..4}, k in {0..Nfe}};
var phy{i in {1..Nv}, k in {0..Nfe}};

############# Minimization objective ###############
minimize objective_:
sum{i in {1..Nv}, j in {1..4}}((x[i,j,Nfe] - TerminalLocations[i,j,1])^2 + (y[i,j,Nfe] - TerminalLocations[i,j,2])^2);


############# DAEs ############# 
s.t. DIFF_dxdt {i in {1..Nv}, j in {1..Nfe}}:
x[i,1,j] = x[i,1,j-1] + dt * v[i,1,j-1] * cos(theta[i,1,j-1]);
s.t. DIFF_dydt {i in {1..Nv}, j in {1..Nfe}}:
y[i,1,j] = y[i,1,j-1] + dt * v[i,1,j-1] * sin(theta[i,1,j-1]);
s.t. DIFF_dtheta1dt {i in {1..Nv}, j in {1..Nfe}}:
theta[i,1,j] = theta[i,1,j-1] + dt * v[i,1,j-1] * tan(phy[i,j-1]) / L_tractor_wheelbase;

s.t. ALGE_x_2_to_Nv {i in {1..Nv}, j in {2..4}, k in {0..Nfe}}:
x[i,j,k] = x[i,j-1,k] - L * cos(theta[i,j,k]) - M[i,j-1] * cos(theta[i,j-1,k]);
s.t. ALGE_y_2_to_Nv {i in {1..Nv}, j in {2..4}, k in {0..Nfe}}:
y[i,j,k] = y[i,j-1,k] - L * sin(theta[i,j,k]) - M[i,j-1] * sin(theta[i,j-1,k]);

s.t. DIFF_theta_2_to_Nv {i in {1..Nfe}, j in {2..4}, kk in {1..Nv}}:
L * (theta[kk,j,i] - theta[kk,j,i-1]) = dt * (v[kk,j-1,i] * sin(theta[kk,j-1,i] - theta[kk,j,i])) - M[kk,j-1] * cos(theta[kk,j-1,i] - theta[kk,j,i]) * (theta[kk,j-1,i] - theta[kk,j-1,i-1]);

s.t. DIFF_v_2_to_Nv {i in {1..Nfe}, j in {2..4}, kk in {1..Nv}}:
dt * v[kk,j,i] = dt * v[kk,j-1,i] * cos(theta[kk,j-1,i] - theta[kk,j,i]) + M[kk,j-1] * sin(theta[kk,j-1,i] - theta[kk,j,i]) * (theta[kk,j-1,i] - theta[kk,j-1,i-1]);

############# Specify (xc,yc) #############
s.t. SpecifyTractorXc1 {i in {1..Nv}, j in {0..Nfe}}:
xc[i,1,j] = x[i,1,j] + cos(theta[i,1,j]) * 0.75;
s.t. SpecifyTractorYc1 {i in {1..Nv}, j in {0..Nfe}}:
yc[i,1,j] = y[i,1,j] + sin(theta[i,1,j]) * 0.75;
s.t. SpecifyTractorXc2To4 {i in {1..Nv}, j in {0..Nfe}, kk in {2..4}}:
xc[i,kk,j] = x[i,kk,j];
s.t. SpecifyTractorYc2To4 {i in {1..Nv}, j in {0..Nfe}, kk in {2..4}}:
yc[i,kk,j] = y[i,kk,j];

######## Boundary value conditions ########
s.t. EQ_init_x {i in {1..Nv}} :
x[i,1,0] = InitialValue[i,1];
s.t. EQ_init_y {i in {1..Nv}} :
y[i,1,0] = InitialValue[i,2];
s.t. EQ_init_theta1to4 {i in {1..Nv}, kk in {1..4}} :
theta[i,kk,0] = InitialValue[i,kk+2];
s.t. EQ_bv_phy {i in {1..Nv}, index in {0,1,Nfe,Nfe-1}} :
phy[i,index] = 0;
s.t. EQ_bv_v {i in {1..Nv}, k in {1},index in {0,1,Nfe,Nfe-1}} :
v[i,k,index] = 0;

########## Path constraints ##########
s.t. Bonds_v1 {i in {1..Nfe}, kk in {1..Nv}}:
-vmax <= v[kk,1,i] <= vmax;
s.t. Bonds_a {i in {1..Nfe}, kk in {1..Nv}}:
-amax <= v[kk,1,i] - v[kk,1,i-1] <= amax;
s.t. Bonds_phy {i in {1..Nfe}, kk in {1..Nv}}:
-phymax <= phy[kk,i] <= phymax;
s.t. Bonds_w {i in {1..Nfe}, kk in {1..Nv}}:
-wmax <= phy[kk,i] - phy[kk,i-1] <= wmax;
s.t. Bonds_dtheta {i in {1..Nv}, j in {2..4}, k in {0..Nfe}}:
-1.5708 <= theta[i,j,k] - theta[i,j-1,k] <= 1.5708;


data;
param Nobs := include Nobs;
param: OC := include OC;
param: InitialValue := include InitialValue;
param: TerminalLine := include TerminalLine;
param: TerminalLocations := include TerminalLocations;
param: M := include M;