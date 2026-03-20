function stext = labeldef(stext)
% Define special labels for conciseness.
% 
% Prototype: stext = labeldef(stext)
% Input: stext - a short text input
% Output: stext - corresponding fully formated text output
%
% See also  xygo, ptitle, myfig.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/03/2014, 07/07/2021
    global glv
    global specl_string
    if isempty(specl_string) || nargin<1  % reload label strings, labeldef();
    specl_string = {...  % string cell
        't0'     '\itt\rm_0 / s';
        't/s'    '\itt \rm / s';
        't/m'    '\itt \rm / min';
        't/h'    '\itt \rm / h';
        't/d'    '\itt \rm / d';
        'k'      '\itk \rm / count';
        'phi',   '\it\phi\rm / ( \prime )';
        'phi-sec',   '\it\phi\rm / ( \prime\prime )';
        'phiE',  '\it\phi\rm_E / ( \prime\prime )';
        'phiN',  '\it\phi\rm_N / ( \prime\prime )';
        'phiU',  '\it\phi\rm_U / ( \prime )';
        'phiUsec',  '\it\phi\rm_U / ( \prime\prime )';
        'phiEN', '\it\phi\rm_E,\it\phi\rm_N / ( \prime\prime )';
        'phix',  '\it\phi_x\rm / ( \circ )';
        'phiy',  '\it\phi_y\rm / ( \circ )';
        'phiz',  '\it\phi_z\rm / ( \circ )';
        'phixy', '\it\phi _{x,y}\rm / ( \circ )';
        'mu',    '\it\mu \rm / ( \prime )';
        'mux',   '\it\mu_x \rm / ( \prime )';
        'muy',   '\it\mu_y \rm / ( \prime )';
        'muz',   '\it\mu_z \rm / ( \prime )';
        'theta', '\it\theta \rm / ( \prime )';
        'dvEN',  '\it\delta v\rm_{E,N} / ( m/s )';
        'dvE',   '\delta\it v\rm_E / ( m/s )';
        'dvN',   '\delta\it v\rm_N / ( m/s )';
        'dvU',   '\delta\it v\rm_U / ( m/s )';
        'dv',    '\delta\it v ^n\rm / ( m/s )';
        'pr',    '\it\theta , \gamma\rm / ( \circ )';
        'ry',    '\it\gamma , \psi\rm / ( \circ )';
        'p',     '\it\theta\rm / ( \circ )';
        'r',     '\it\gamma\rm / ( \circ )';
        'y',     '\it\psi\rm / ( \circ )';
        'att',   '\itAtt\rm / ( \circ )';
        'dpch'   '\delta\it\theta \rm / ( \prime )';
        'drll'   '\delta\it\gamma \rm / ( \prime )';
        'dyaw'   '\delta\it\psi \rm / ( \prime )';
        'datt',  '\itdAtt\rm / ( \prime )';
        'VEN',   '\itV \rm_{E,N} / ( m/s )';
        'VE',   '\itV \rm_E / ( m/s )';
        'VN',   '\itV \rm_N / ( m/s )';
        'VU',    '\itV \rm_U / ( m/s )';
        'V',     '\itV\rm / ( m/s )';
        'Vx',    '\itVx\rm / ( m/s )';
        'Vy',    '\itVy\rm / ( m/s )';
        'Vz',    '\itVz\rm / ( m/s )';
        'dlat',  '\delta\it L\rm / m';
        'dlon',  '\delta\it \lambda\rm / m';
        'dll',   '\delta\it L,\rm\delta\it \lambda\rm / ( \prime )';
        'dH',    '\delta\it h\rm / m';
        'dP',    '\delta\it p^n\rm / m';
        'dR',    '\delta\it R\rm / m';
        'lat',   '\itL\rm / ( \circ )';
        'lon',   '\it\lambda\rm / ( \circ )';
        'hgt',   '\ith\rm / ( m )';
        'xyz',   'XYZ / ( m )';
        'dxyz',  'dXYZ / ( m )';
        'est',   'East\rm / m';
        'nth',   'North\rm / m';
        'H',     '\ith\rm / m';
        'DP',    '\Delta\it P\rm / m';
        'ebxy',  '\it\epsilon _{x,y}\rm / ( (\circ)/h )';
        'ebyz',  '\it\epsilon _{y,z}\rm / ( (\circ)/h )';
        'ebx',  '\it\epsilon _x\rm / ( (\circ)/h )';
        'eby',  '\it\epsilon _y\rm / ( (\circ)/h )';
        'ebz',  '\it\epsilon _z\rm / ( (\circ)/h )';
        'eb',    '\it\epsilon\rm / ( (\circ)/h )';
        'en',    '\it\epsilon\rm / ( (\circ)/h )';
        'enE',    '\it\epsilon\rm_E / ( (\circ)/h )';
        'enN',    '\it\epsilon\rm_N / ( (\circ)/h )';
        'enU',    '\it\epsilon\rm_U / ( (\circ)/h )';
        'gS',    'gSens / ( (\circ)/h/g )';
        'db',    '\it\nabla\rm / \mu\itg';
        'dbxy',   '\it\nabla_{x,y}\rm / \mu\itg';
        'dbx',   '\it\nabla_x\rm / \mu\itg';
        'dby',   '\it\nabla_y\rm / \mu\itg';
        'dbz',   '\it\nabla_z\rm / \mu\itg';
        'dKij',  '\delta\itKij\rm / (\prime\prime)';
        'dKii',  '\delta\itKii\rm / ppm';
        'Ka2',   'Ka2 / ug/g^2';
        'Kap',   'Kap / ppm';
        'dbU',   '\it\nabla \rm_U / \mu\itg';
        'L',     '\itLever\rm / m';
        'dT',    '\delta\it T_{asyn}\rm / ms';
        'dKgzz',   '\delta\it Kgzz\rm / ppm';
        'dKg',   '\delta\it Kg\rm / ppm';
        'dAg',   '\delta\it Ag\rm / ( \prime\prime )';
        'dKa',   '\delta\it Ka\rm / ppm';
        'dAa',   '\delta\it Aa\rm / ( \prime\prime )';
		'wx',    '\it\omega_x\rm / ( (\circ)/s )';
		'wy',    '\it\omega_y\rm / ( (\circ)/s )';
		'wz',    '\it\omega_z\rm / ( (\circ)/s )';
		'w',     '\it\omega\rm / ( (\circ)/s )';
		'wxdph',    '\it\omega_x\rm / ( (\circ)/h )';
		'wydph',    '\it\omega_y\rm / ( (\circ)/h )';
		'wzdph',    '\it\omega_z\rm / ( (\circ)/h )';
		'wdph',     '\it\omega\rm / ( (\circ)/h )';
		'fx',    '\itf_x\rm / \itg';
		'fy',    '\itf_y\rm / \itg';
		'fz',    '\itf_z\rm / \itg';
		'f',     '\itf\rm / \itg';
		'fxug',    '\itf_x\rm / u\itg';
		'fyug',    '\itf_y\rm / u\itg';
		'fzug',    '\itf_z\rm / u\itg';
		'fug',     '\itf\rm / u\itg';
		'fxmg',    '\itf_x\rm / m\itg';
		'fymg',    '\itf_y\rm / m\itg';
		'fzmg',    '\itf_z\rm / m\itg';
		'fmg',     '\itf\rm / m\itg';
        'Temp',  '\itT\rm / \circC';
        'frq',  '\itf\rm / Hz';
		'dinst', '\delta\it\theta , \rm\delta\it\psi\rm / ( \prime )';
    };
        if nargin<1, stext=[]; return; end
    end
    if strcmp(stext,'t')==1
        switch glv.tscale(end)
            case 1, stext='t/s';
            case 60, stext='t/m';
            case 3600, stext='t/h';
            case 24*3600, stext='t/d';
        end
    end
    for k=1:length(specl_string)
        if strcmp(stext,specl_string(k,1))==1
            stext = specl_string{k,2};
            break;
        end
    end