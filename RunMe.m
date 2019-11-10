% ==============================================================================
% MATLAB Source Codes for "Cooperative Motion Planning for Multiple
% Tractor-Trailer Vehicles with Moving Obstacles: A Numerical Optimal
% Control Method".
% ==============================================================================
%   Copyright (C) 2020 Bai Li
%   User must cite the following article if they utilize these codes for
%   their own researches: Bai Li et al., "Cooperative Motion Planning for
%   Multiple Tractor-Trailer Vehicles with Moving Obstacles: A Numerical
%   Optimal Control Method".
% ==============================================================================
% If there are inquiries, feel free to contact inde_of_case@zju.edu.cn
%
% (Very Important) The AMPL utilized in this pack is just a TRIAL version.
% The users must delete AMPL.exe in this version after trying it, and
% then apply for their own valid license files through following the
% official instructions at https://ampl.com/try-ampl/request-a-full-trial/
% ==============================================================================

clear all
close all
clc

Ncase = 4;
data = zeros(Ncase, 3); % solution status, CPU time, and terminal time period
dt = 20;

% _________________________________________________________________ %
for inde_of_case = 1 : Ncase
    clc
    str = num2str(inde_of_case);
    load(str);
    Nfe = 100;
    WriteFilesForNLP();
    tic
    !ampl r0.run
    !ampl r1.run
    sequence = [1 : dt : Nfe];
    if (sequence(end) ~= Nfe)
        sequence = [sequence, Nfe];
    end
    for iiii = sequence
        WriteVariable('IX', iiii);
        !ampl r2.run
        load sol_status.txt
        if (sol_status)
            val = EvaluateCollisionDegree();
            if (sum(val) == 0)
                break;
            end
        end
    end
    load sol_status.txt
    if ((sol_status)&&(sum(EvaluateCollisionDegree()) == 0))
        data(inde_of_case, 1) = 1;
    else
        data(inde_of_case, 1) = 0;
    end
    data(inde_of_case, 2) = toc;
    data(inde_of_case, 3) = iiii;
    
    if (data(inde_of_case, 1))
        DrawStaticFigure;
        drawnow;
    end
end

disp('Success rate : '); disp(mean(data(:,1)));
disp('Average CPU time : '); disp(mean(data(:,2)));
disp('Average terminating percentage : '); disp(mean(data(:,3)) / Nfe * 100);