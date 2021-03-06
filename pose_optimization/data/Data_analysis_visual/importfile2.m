function gpskitti20110930drive1 = importfile2(filename, dataLines)
%IMPORTFILE2 Import data from a text file
%  GPSKITTI20110930DRIVE1 = IMPORTFILE2(FILENAME) reads data from text
%  file FILENAME for the default selection.  Returns the numeric data.
%
%  GPSKITTI20110930DRIVE1 = IMPORTFILE2(FILE, DATALINES) reads data for
%  the specified row interval(s) of text file FILENAME. Specify
%  DATALINES as a positive scalar integer or a N-by-2 array of positive
%  scalar integers for dis-contiguous row intervals.
%
%  Example:
%  gpskitti20110930drive1 = importfile2("/home/boxi/Dev/eecs568-group17-project/pose_optimization/data/gps_kitti_2011_09_30_drive_0018.txt", [2, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 11-Apr-2022 23:44:11

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [2, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 19);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = " ";

% Specify column names and types
opts.VariableNames = ["time", "Var2", "Var3", "Var4", "Var5", "Var6", "Var7", "Var8", "Var9", "Var10", "Var11", "Var12", "Var13", "Var14", "Var15", "Var16", "Var17", "Var18", "Var19"];
opts.SelectedVariableNames = "time";
opts.VariableTypes = ["double", "string", "string", "string", "string", "string", "string", "string", "string", "string", "string", "string", "string", "string", "string", "string", "string", "string", "string"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
opts.ConsecutiveDelimitersRule = "join";
opts.LeadingDelimitersRule = "ignore";

% Specify variable properties
opts = setvaropts(opts, ["Var2", "Var3", "Var4", "Var5", "Var6", "Var7", "Var8", "Var9", "Var10", "Var11", "Var12", "Var13", "Var14", "Var15", "Var16", "Var17", "Var18", "Var19"], "WhitespaceRule", "preserve");
opts = setvaropts(opts, ["Var2", "Var3", "Var4", "Var5", "Var6", "Var7", "Var8", "Var9", "Var10", "Var11", "Var12", "Var13", "Var14", "Var15", "Var16", "Var17", "Var18", "Var19"], "EmptyFieldRule", "auto");

% Import the data
gpskitti20110930drive1 = readtable(filename, opts);

%% Convert to output type
gpskitti20110930drive1 = table2array(gpskitti20110930drive1);
end