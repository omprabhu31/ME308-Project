param T;
param nNodes;

set k := {1..T};
set k1 := {1..(T-1)};
set centers := {1..nNodes};

param Gamma {centers};
param SchDept{centers, k}; 

param Capacity{centers, k};
param DeptLim{centers, k};
param ArrivLim{centers, k};
param TrafLim{centers, centers, k}:= 300;
param x{centers, k};

# Control Variables
var Beta{centers, centers, k} >=0, <=1;
var d{centers, k} >=0 integer;
var a{centers, k} >=0 integer;

# Cost Function
minimize z: (sum{i in centers, q in k} Gamma[i]*(SchDept[i,q] - d[i,q])) + (sum{i in centers, q in k} Gamma[i]*(sum{j in centers} Beta[j,i,q]*x[k,q] - a[i,q]);

# Constraints
s.t. Balance_Constraint {i in centers, q in k1}: x[i,q+1] = x[i,q] - sum{b in centers} (Beta[i,b,q]*x[i,q]) + sum{c in centers: c!=i} (Beta[c,i,q]*x[c,q]) - d[i,q] + a[i,q];
s.t. Beta_Constraint {i in centers, q in k}: sum{c in centers} (Beta[i,c,q]) = 1;
s.t. Capacity_Constraint {i in centers, q in k}: a[i,q] + d[i,q] <= Capacity[i,q];
s.t. DeptLim_Constraint {i in centers, q in k}: d[i,q] <= DeptLim[i,q];
s.t. ArrivLim_Constraint {i in centers, q in k}: a[i,q] <= ArrivLim[i,q];
s.t. Traffic_Flow_Limit {i in centers, j in centers, aq in k}: Beta[i,j,aq]*x[i,aq] + Beta[j,i,aq]*x[j,aq] <= TrafLim[i,j,aq];