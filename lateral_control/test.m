function [outa outb outc]=test ()
persistent hw
isempty(hw)
outa=[1 1];
outb=2;
outc=3;
return