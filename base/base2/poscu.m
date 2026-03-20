function pos = poscu(strname)
% Geographic position in common use / curren user.
%
% Prototype: pos = poscu(strname)
% Input: strname=string name of institude NO.
% Output: pos=[lat; lon; hgt]
% 
% See also  posset, bdpoint.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/09/2024
    if isnumeric(strname)
        switch strname
            case  16,  pos = llh(34.1767, 108.95115, 400);
            case 203,  pos = llh(34.1975, 108.9203273, 407);
            case 618,  pos = llh(34.1977, 108.8301, 400);
        end
    else
        switch upper(strname)
            case 'NWPU',  pos = llh(34.0343100, 108.7754270, 450);
            case 'JZ',    pos = llh(34.1716655, 108.8435892, 431);
            case 'HH',    pos = llh(34.199287,  108.857688,  387);
            case 'LD',    pos = llh();
        end
    end
