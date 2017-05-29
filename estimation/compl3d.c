#include <math.h>
#include <stdio.h>
#include "estimation.h"


void angulos() {
    
    float t[2];


    t[0]=sqrt(square((float)accADC[2])+square((float)accADC[1]));
    t[1]=sqrt(square((float)accADC[2])+square((float)accADC[0]));

    if(accADC[2]<0){
        t[1]=-t[1];
    }

    angle[0]=atan2(accADC[0],t[0]);
    angle[1]=atan2(accADC[1],t[1]);

    
    t[0]=gyroADC[0]*cos(anglef[1])/sqrt(  1-square( sin(anglef[0])*sin(anglef[1])  )   );
    t[1]=gyroADC[2]*sin(anglef[1])/sqrt(  1-square( sin(anglef[0])*cos(anglef[1])  )   );
    t[0]=t[0]+t[1];
    
    anglef[0]=0.01*angle[0]+0.99*anglef[0]+0.000012020*(t[0]);
    
    t[0]=gyroADC[1]*cos(anglef[0])/sqrt(  1+square(sin(anglef[1])*sin(anglef[0])   )   );
    t[1]=gyroADC[2]*sin(anglef[0])/sqrt(  1-square(sin(anglef[1])*cos(anglef[0])   )   );
    t[0]=t[0]-t[1];

    anglef[1]=0.01*angle[1]+0.99*anglef[1]+0.000012020*(t[0]);

    
    
    anglef[2]=-atan2(magADC[0],magADC[2]);

    
 
    // Matlab code
    // v1(i+1)=0.01*aa(i)+0.99*v1(i)+0.99*( VarName1(i)*cos(v2(i))/sqrt(1-(sin(v1(i))*sin(v2(i)))^2)+VarName3(i)*sin(v2(i))/sqrt(1-(sin(v1(i))*cos(v2(i)))^2) )/m*pi/2 ;
    // v2(i+1)=0.01*bb(i)+0.99*v2(i)+0.99*( VarName2(i)*cos(v1(i))/sqrt(1+(sin(v2(i))*sin(v1(i)))^2)-VarName3(i)*sin(v1(i))/sqrt(1-(sin(v2(i))*cos(v1(i)))^2) )/m*pi/2 ;
 

/*
    anglef[0]=0.01*(float)angle[0]+0.02754*(float)gyroADC[0]+0.99*anglef[0];
    anglef[1]=0.01*(float)angle[1]+0.02754*(float)gyroADC[1]+0.99*anglef[1];
*/
    // 2000/8192*0.01*10 = 0.024414 //Multiwii *0.99
    // 4/14.375*0.01*10  = 0.02781  //Datasheer *0.99
    
    // 1/14.375*pi/180*0.01     = 1.21414208834388e-05 // Codigo novo em radianos *0.99
    
    
}
