#include <stdio.h>
#include "control.h"

void controle() {

    
    //pid[0]=((ref[0]-(int)anglef[0])*2-gyroADC[0])*0.25;
    //pid[1]=((ref[1]-(int)anglef[1])*2-gyroADC[1])*0.25;
    //pid[2]=(ref[3]-gyroADC[2]);

    pid[0]=(   (ref[0]*7-(int)(anglef[0]*2800) )   -gyroADC[0])*0.1;
    pid[1]=(   (ref[1]*7-(int)(anglef[1]*2800) )   -gyroADC[1])*0.1;
    pid[2]=(   (ref[3]*7-(int)(anglef[2]*2800) )   -gyroADC[2])*0.1;
  
}

void atualiza_pid() {
    int j;
    
    motor[0]=ref[2]+pid[0]-pid[1]-pid[2];
    motor[1]=ref[2]-pid[0]+pid[1]-pid[2];
    motor[2]=ref[2]-pid[0]-pid[1]+pid[2];
    motor[3]=ref[2]+pid[0]+pid[1]+pid[2];
    
    for(j=0;j<4;j++)
    {
        if(motor[j]>1000) motor[j]=1000;
        if(motor[j]<90) motor[j]=90;
        if(ref[2]<10) motor[j]=0;
        
    }
    
}
