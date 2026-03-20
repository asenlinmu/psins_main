glvs
strcom = {
    '1 #&#fdkkklllaxy',
    '2 #&#fdkkklll***',
    '4 #&#fdkkklllssssssssssss',
    };
s=serialport('COM11', 115200); 
writeline(s, '#start');
for k=1:length(strcom)
    ps = sscanf(strcom{k}, '%f');
    pause(ps);
    idx = strfind(strcom{k},'#&#')+3;
    writeline(s, strcom{k}(idx:end));
end
writeline(s, '#end');
clear s;