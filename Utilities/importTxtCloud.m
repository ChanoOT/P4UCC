%%%%%%%%%%%%%%%%%%%%%%%%%
%% P4UCC Project
%% S. Ortega, A. Trujillo, J.M. Santana, C. Ram�rez, J.P. Su�rez
%% Universidad de Las Palmas de Gran Canaria, 2020
%%%%%%%%%%%%%%%%%%%%%%%%%

function data = importTxtCloud(filename)
%% Returns X-Y-Z-Label

t = importdata(filename);
data = t.data;

end

