classdef FunctionsPathPlan
    methods
        function T = T_mat(obj, time)
            T = [[1,0,0,0,0,0];
                [0,1,0,0,0,0];
                [0,0,2,0,0,0];
                [1,time,time^2,time^3,time^4,time^5];
                [0,1,2*time,3*time^2,4*time^3, 5*time^4];
                [0,0,2,6*time,12*time^2,20*time^3]];
        end
        function B = Q_mat(obj, pos_i, pos_f)
            B = [[pos_i];
                [0];
                [0];
                [pos_f];
                [0];
                [0]];
        end
        function res = GraphWithValues(obj, array, time)
            a0 = array(1);
            a1 = array(2);
            a2 = array(3);
            a3 = array(4);
            a4 = array(5);
            a5 = array(6);
            res = a0 + a1 * time  + a2 * time^2 + a3 * time^3 + a4 * time^4 + a5 * time^5;
        end
        function D = FirstDer(obj, array, time)
            
            a0 = array(1);
            a1 = array(2);
            a2 = array(3);
            a3 = array(4);
            a4 = array(5);
            a5 = array(6);
            
            D = a1 + 2* a2 * time + 3 * a3 * time^2 + 4 * a4 * time^3 + 5* a5 * time^4;
        end
        function D = SecondDer(obj, array, time)
            a0 = array(1);
            a1 = array(2);
            a2 = array(3);
            a3 = array(4);
            a4 = array(5);
            a5 = array(6);
            D = 2* a2 + 6 * a3 * time + 12 * a4 * time^2 + 20* a5 * time^3;
        end
        function D = IncrementalValues(obj, array, time, time_stamp)
            a = array(1) + array(2) * time  + array(3) * time^2 + array(4) * time^3 + array(5) * time^4 + array(6) * time^5;
            b = array(1) + array(2) * time_stamp  + array(3) * time_stamp^2 + array(4) * time_stamp^3 + array(5) * time_stamp^4 + array(6) * time_stamp^5;
            D = b-a;
        end
   end
end
    




