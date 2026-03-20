function data = repdata(data0, n, aRowData)
% Repeat data, especially IMU or AVP in static base.
%
% Prototype: data = repdata(data0, n, aRowData)
% Inputs: 
%    data0 - some data with the last to be time tag
%    n - n-times
%    aRowData - a row-data to be repeated & added to the data
% Output: data - new data Repeated n-times
%
% See also  imuresample.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 16/01/2026
    data = repmat(data0,n,1);
    data(:,end) = diff(data0(1:2,end))*(1:length(data0)*n)'+data0(1,end);
    if exist('aRowData','var')
        data = [data(:,1:end-1), repmat(aRowData(:)',length(data),1), data(:,end)];
    end