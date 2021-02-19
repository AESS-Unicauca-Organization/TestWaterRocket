clc
clear all;

Datos=importdata('DatosR.txt');
Time=zeros(1,89);
A1=zeros(1,89);
A2=zeros(1,89);
A3=zeros(1,89);
A4=zeros(1,89);
A5=zeros(1,89);
A6=zeros(1,89);
A7=zeros(1,89);
A8=zeros(1,89);
Tam=size(Datos);
Tam1=Tam(1);
i=1;
j=1;  
while (i<=Tam1)
    Time(j)=Datos(i);
    j=j+1;
    i=i+10;
end
i=2;
j=1;
while (i<=Tam1)
    A1(j)=Datos(i);
    j=j+1;
    i=i+10;
end
i=3;
j=1;
while (i<=Tam1)
    A2(j)=Datos(i);
    j=j+1;
    i=i+10;
end
i=4;
j=1;
while (i<=Tam1)
    A3(j)=Datos(i);
    j=j+1;
    i=i+10;
end
i=5;
j=1;
while (i<=Tam1)
    A4(j)=Datos(i);
    j=j+1;
    i=i+10;
end
i=6;
j=1;
while (i<=Tam1)
    A5(j)=Datos(i);
    j=j+1;
    i=i+10;
end
i=7;
j=1;
while (i<=Tam1)
    A6(j)=Datos(i);
    j=j+1;
    i=i+10;
end
i=8;
j=1;
while (i<=Tam1)
    A7(j)=Datos(i);
    j=j+1;
    i=i+10;
end
i=9;
j=1;
while (i<=Tam1)
    A8(j)=Datos(i);
    j=j+1;
    i=i+10;
end

plot(Time,A3)
