function [StatementDot] = doDiff(Statement)

syms q1 q2 q3 q4 q5 q6 q7 real
syms dq1 dq2 dq3 dq4 dq5 dq6 dq7 real

StatementDot = diff(Statement,q1)*dq1 + diff(Statement,q2)*dq2 + diff(Statement,q3)*dq3 + diff(Statement,q4)*dq4 + diff(Statement,q5)*dq5 + diff(Statement,q6)*dq6 + diff(Statement,q7)*dq7;

end