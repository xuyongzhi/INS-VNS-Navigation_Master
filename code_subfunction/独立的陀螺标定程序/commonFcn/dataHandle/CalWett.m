%  buaa xyz 2014.1.10

% Wet_t

function Wet_t = CalWett( Vet_t,Re,e,L )


RytDs = ( 1+2*e-3*e*(sin(L))^2 )/Re;     %  1/Ryt
RxtDs = ( 1-e*(sin(L))^2 )/Re ; %  1/Rxt
Wet_t = [   -Vet_t(2)*RytDs ; Vet_t(1)*RxtDs ;  Vet_t(1)*RxtDs*tan(L)  ];