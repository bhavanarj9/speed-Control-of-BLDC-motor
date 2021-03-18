#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<conio.h>
#define pi 3.142
float ls_lm,vas,rs,ias,lamda_p,fas_thr,vbs,ibs,fbs_thr,ics,vcs,fcs_thr,te,tl,inertia,pole,tt;
#define seq_ias(tt,ias)(1/ls_lm)*(vas-(rs*ias)-(lamda_p*fas_thr))
#define seq_ibs(tt,ibs)(1/ls_lm)*(vbs-(rs*ibs)-(lamda_p*fbs_thr))
#define seq_ics(tt,ics)(1/ls_lm)*(vcs-(rs*ics)-(lamda_p*fcs_thr))
#define seq_wr(tt,wr)(te-tl)/inertia
#define seq_thr(tt,thr)(pole/2)*wr

int main()
{
    FILE*fp1=fopen("current1.temp","w");
    FILE*fp2=fopen("current2.temp","w");
    FILE*fp3=fopen("current3.temp","w");
    FILE*fp4=fopen("torque.temp","w");
    FILE*fp5=fopen("speed.temp","w");
    FILE*fp6=fopen("theta.temp","w");
    FILE*fp7=fopen("emf1.temp","w");
    FILE*fp8=fopen("emf2.temp","w");
    FILE*fp9=fopen("emf3.temp","w");
    FILE*fp10=fopen("current1c.temp","w");
    FILE*fp11=fopen("current2c.temp","w");
    FILE*fp12=fopen("current3c.temp","w");
    FILE*fp13=fopen("torque2.temp","w");
    FILE*fp14=fopen("speed2.temp","w");

    float mode,phase,rs,ls_lm,inertia,b_fric,p_hp,pole,vdc,kb,lamda_p,ip_rate,te_rate,wbase_rpm,pbase,vbase,ibase,tbase,wbase,ias,ibs,ics,wr,thr,thr_old,
           wr_cmd,kp_w,ki_w,wr_intg,te_cmd,te_star,te_lim_p,te_lim_n,tl,kp_i,vasc,vbsc,vcsc,ftr,vtr_r,vc_lim_p,vc_lim_n,tt,dt,tf,wr_err,ip_cmd,thr_deg,
           ias_cmd,ibs_cmd,ics_cmd,fas_thr,fbs_thr,fcs_thr,eas,ebs,ecs,s1,s2,vtr,del_thr,ias_det,ibs_det,ics_det,vas,vbs,vcs,grad_1,grad_2,grad_3,grad_4,
           tas,tbs,tcs,te,slope_1,slope_2,slope_3,slope_4,slope_11,slope_12,slope_13,slope_14,wr1,thr1;
           rs=0.7;
           ls_lm=5e-3;
           inertia=0.0022;
           b_fric=0;
           p_hp=1;
           pole=4;
           vdc=48;
           kb=0.05238;
           lamda_p=kb*(pole/2);
           ip_rate=8.5;
           te_rate=1.5;
           wbase=1500;
           pbase=p_hp*746;
           vbase=vdc;
           ibase=ip_rate;
           tbase=te_rate;

           ias=0;
           ibs=0;
           ics=0;
           wr=0;
           thr=0;
           thr_old=thr;
           wr_cmd=0;
           kp_w=0.1;
           ki_w=0.001;
           wr_intg=0;
           te_cmd=0;
           te_star=0;
           te_lim_p=2*te_rate;
           te_lim_n=-2*te_rate;
           tl=0;
           kp_i=100;
           vasc=0;
           vbsc=0;
           vcsc=0;
           ftr=6000;
           vtr_r=10;
           vc_lim_p=9;
           vc_lim_n=-9;
           tt=0.0;
           dt=10e-6;
           tf=0.025;

           while(tt<=tf)
        {
            wr_err=wr_cmd-wr;
            wr_intg=wr_intg+ki_w*(wr_err+te_cmd-te_star)*dt;
            te_star=wr_intg+kp_w*wr_err;

            if(te_star>=te_lim_p)
            {
                te_cmd=te_lim_p;
            }
            else if(te_star<=te_lim_n)
            {
                te_cmd=te_lim_n;
            }
            else
                {
                    te_cmd=te_star;
                }
                ip_cmd=te_cmd/(2*lamda_p);
                thr_deg=thr*180.0/pi;
                while((thr_deg<0.0)||(thr_deg>360))
                {
                    if(thr_deg<0.0)
                    {thr_deg=thr_deg+360.0;}
                    else if(thr_deg>360.0)
                       {thr_deg=thr_deg-360.0;}
                }
             if(((thr_deg>=0.0)&(thr_deg<30.0))||((thr_deg>=330.0)&&(thr_deg<=360.0)))
             {
                 mode=1.0;
                 ias_cmd=0.0;
                  ibs_cmd=-ip_cmd;
                   ics_cmd=ip_cmd;
             }
            else if((thr_deg>=30.0)&&(thr_deg<90.0))
                 {
                 mode=2.0;
                 ics_cmd=0.0;
                  ibs_cmd=-ip_cmd;
                   ias_cmd=ip_cmd;
             }
             else if((thr_deg>=90.0)&&(thr_deg<150.0))
                 {
                 mode=3.0;
                 ibs_cmd=0.0;
                  ics_cmd=-ip_cmd;
                   ias_cmd=ip_cmd;
             }
              else if((thr_deg>=150.0)&&(thr_deg<210.0))
                 {
                 mode=4.0;
                 ias_cmd=0.0;
                  ics_cmd=-ip_cmd;
                   ibs_cmd=ip_cmd;
             }
              else if((thr_deg>=210.0)&&(thr_deg<270.0))
                 {
                 mode=5.0;
                 ics_cmd=0.0;
                  ibs_cmd=ip_cmd;
                   ias_cmd=-ip_cmd;
             }
              else if((thr_deg>=270.0)&&(thr_deg<330.0))
                 {
                 mode=6.0;
                 ibs_cmd=0.0;
                  ics_cmd=ip_cmd;
                   ias_cmd=-ip_cmd;
             }
             if((thr_deg>=0.0)&&(thr_deg<30.0))
             {
                 fas_thr=thr_deg/30.0;
             }
             else if((thr_deg>=30.0)&&(thr_deg<150.0))
             {
                 fas_thr=1.0;
             }
              else if((thr_deg>=150.0)&&(thr_deg<210.0))
             {
                 fas_thr=-(thr_deg/30.0)+6.0;
             }
              else if((thr_deg>=210.0)&&(thr_deg<330.0))
             {
                 fas_thr=-1.0;
             }
              else if((thr_deg>=330.0)&&(thr_deg<360.0))
             {
                 fas_thr=(thr_deg/30.0)-12.0;
             }


             if((thr_deg>=0.0)&&(thr_deg<90.0))
             {
                 fbs_thr=-1;
             }
             else if((thr_deg>=90.0)&&(thr_deg<150.0))
             {
                 fbs_thr=thr_deg/30.0-4.0;
             }
              else if((thr_deg>=150.0)&&(thr_deg<270.0))
             {
                 fbs_thr=1;
             }
              else if((thr_deg>=270.0)&&(thr_deg<330.0))
             {
                 fbs_thr=-(thr_deg/30.0)+10.0;
             }
              else if((thr_deg>=330.0)&&(thr_deg<360.0))
             {
                 fbs_thr=-1.0;
             }

             if((thr_deg>=0.0)&&(thr_deg<30.0))
             {
                 fcs_thr=1.0;
             }
             else if((thr_deg>=30.0)&&(thr_deg<90.0))
             {
                 fcs_thr=-(thr_deg/30.0)+2.0;
             }
              else if((thr_deg>=90.0)&&(thr_deg<210.0))
             {
                 fcs_thr=-1.0;
             }
              else if((thr_deg>=210.0)&&(thr_deg<270.0))
             {
                 fcs_thr=(thr_deg/30.0)-8.0;
             }
              else if((thr_deg>=270.0)&&(thr_deg<360.0))
             {
                 fcs_thr=1.0;
             }

             eas=fas_thr*kb*wr;
             ebs=fbs_thr*kb*wr;
             ecs=fcs_thr*kb*wr;

             s1=(4.0*ftr*vtr_r*tt);
             s2=(4.0*ftr*vtr_r);
             vtr=fmod(s1,s2);

             if(vtr<vtr_r)
             {
                 vtr=vtr;
             }
             else if(vtr<(3.0*vtr_r))
             {
                 vtr=2.0*vtr_r-vtr;
             }
             else{vtr=vtr-4.0*vtr_r;}
             del_thr=thr-thr_old;
             thr_old=thr;

             if((mode!=1.0)&(mode!=4.0))
             {
                 ias_det=1.0;
                 vasc=kp_i*(ias_cmd-ias);
                 if(vasc>vc_lim_p)
                 {
                     vasc=vc_lim_p;
                 }
                 else if(vasc<vc_lim_n)
                 {
                     vasc=vc_lim_n;
                 }
                 if(vasc>=vtr)
                 {
                     vas=vdc/2.0;

                 }
                 else{vas=-vdc/2.0;}

             }
           else if(((del_thr>=0.0)&&(mode==1.0)||((del_thr<0.0)&&(mode==4.0))))
            {
                if(ias>0.0)
                    {ias_det=0.0;
                 ias=0.0;
                 vas=eas;}
                else{ias_det=1.0; vas=vdc/2.0;}

            }
         if((mode!=3.0)&&(mode!=6))
            {
                ibs_det=1.0;
                vbsc=kp_i*(ibs_cmd-ibs);
                if(vbsc>vc_lim_p)
                {
                    vbsc=vc_lim_p;
                }
                else if(vbsc<vc_lim_n)
                {
                    vbsc=vc_lim_n;
                }
                if(vbsc>=vtr)
                {
                    vbs=vdc/2.0;
                }
                else{vbs=-vdc/2.0;}
            }
           else if(((del_thr<0.0)&&(mode==3.0)||((del_thr>0.0)&&(mode==6.0))))
          {
                if(ibs<0.0)
                    { ibs_det=0.0;
                      ibs=0.0;
                      vbs=ebs; }
                else{
                        ibs_det=1.0;
                        vbs=-vdc/2.0;}
                }
              else if(((del_thr>=0.0)&&(mode==3.0)||((del_thr<0.0)&&(mode==6.0))))
            {
                if(ibs>0.0)
                    {  ibs_det=0.0;
                       ibs=0.0;
                    vbs=ebs;  }
                else{ibs_det=1.0; vbs=vdc/2.0;}

            }
         if((mode!=5.0)&&(mode!=2))
            {
                ics_det=1.0;
                vcsc=kp_i*(ics_cmd-ics);
                if(vcsc>vc_lim_p)
                {
                    vcsc=vc_lim_p;
                }
                else if(vcsc<vc_lim_n)
                {
                    vbsc=vc_lim_n;
                }
                if(vcsc>=vtr)
                {
                    vcs=vdc/2.0;
                }
                else{vcs=-vdc/2.0;}
            }

           else if(((del_thr<0.0)&&(mode==5.0)||((del_thr>0.0)&&(mode==2.0))))
          {
                if(ics<0.0)
                    {ics_det=0.0;
                 ics=0.0;
                 vcs=ebs;}
                else{
                        ics_det=1.0;
                vcs=-vdc/2.0;
                }
          }
              else if(((del_thr>=0.0)&&(mode==5.0)||((del_thr<0.0)&&(mode==2.0))))
            {
                if(ics>=0)
                {
                    ics_det=0.0;
                    ics=0.0;
                    vcs=ecs;
                    }
                    else{ics_det=1.0; vcs=vdc/2.0;}
            }
            if(ias_det==1.0)
            {
                phase=1.0;
                grad_1=seq_ias(tt,ias);
                grad_2=seq_ias(tt+dt/2.0,ias+dt*grad_1/2.0);
                grad_3=seq_ias(tt+dt/2.0,ias+dt*grad_2/2.0);
                grad_4=seq_ias(tt+dt,ias+dt*grad_3);
                ias=ias+(grad_1+2.0*grad_2+2.0*grad_3+grad_4)*dt/6.0;

            }

             if(ibs_det==1.0)
            {
                phase=2.0;
                grad_1=seq_ibs(tt,ibs);
                grad_2=seq_ibs(tt+dt/2.0,ibs+dt*grad_1/2.0);
                grad_3=seq_ibs(tt+dt/2.0,ibs+dt*grad_2/2.0);
                grad_4=seq_ibs(tt+dt,ibs+dt*grad_3);
                ibs=ibs+(grad_1+2.0*grad_2+2.0*grad_3+grad_4)*dt/6.0;

            }
             if(ics_det==1.0)
            {
                phase=3.0;
                grad_1=seq_ics(tt,ics);
                grad_2=seq_ics(tt+dt/2.0,ics+dt*grad_1/2.0);
                grad_3=seq_ias(tt+dt/2.0,ics+dt*grad_2/2.0);
                grad_4=seq_ics(tt+dt,ics+dt*grad_3);
                ics=ics+(grad_1+2.0*grad_2+2.0*grad_3+grad_4)*dt/6.0;

            }
            tas=(pole/2)*kb*fas_thr*ias;
            tbs=(pole/2)*kb*fbs_thr*ibs;
            tcs=(pole/2)*kb*fcs_thr*ics;
            te=tas+tbs+tcs;


            slope_1=seq_wr(tt,wr);
            slope_2=seq_wr(tt+dt/2.0,wr+dt*slope_1/2.0);
            slope_3=seq_wr(tt+dt/2.0,wr+dt*slope_2/2.0);
            slope_4=seq_wr(tt+dt,wr+dt*slope_3);
            wr=wr+(slope_1+2.0*slope_2+2.0*slope_3+slope_4)*dt/6.0;
            slope_11=seq_thr(tt,thr);
            slope_12=seq_wr(tt+dt/2.0,thr+dt*slope_1/2.0);
            slope_13=seq_wr(tt+dt/2.0,thr+dt*slope_2/2.0);
            slope_14=seq_wr(tt+dt,thr+dt*slope_3);
            thr=thr+(slope_11+2.0*slope_12+2.0*slope_13+slope_14)*dt/6.0;

            wr1=wr;
            wr=wr1;
            thr1=thr;
            thr=thr1;
            fprintf(fp1,"%f\t  %f\n",tt,ias);
            fprintf(fp2,"%f\t  %f\n",tt,ibs);
            fprintf(fp3,"%f\t  %f\n",tt,ics);
            fprintf(fp4,"%f\t  %f\n",tt,te);
            fprintf(fp5,"%f\t  %f\n",tt,wr);
            fprintf(fp6,"%f\t  %f\n",tt,thr_deg);
            fprintf(fp10,"%f\t  %f\n",thr,ias_cmd);
            fprintf(fp11,"%f\t  %f\n",thr,ibs_cmd);
            fprintf(fp12,"%f\t  %f\n",thr,ics_cmd);
            fprintf(fp13,"%f\t  %f\n",tt,te_cmd);
            fprintf(fp14,"%f\t  %f\n",tt,wr_cmd);
            fprintf(fp7,"%f\t %f\n",tt,eas);
            fprintf(fp8,"%f\t  %f\n",tt,ebs);
            fprintf(fp9,"%f\t  %f\n",tt,ecs);
            printf("%f\n",thr_deg);
            tt=tt+dt;

             }
fclose(fp1);
fclose(fp2);
fclose(fp3);
fclose(fp4);
fclose(fp5);
fclose(fp6);
fclose(fp7);
fclose(fp8);
fclose(fp9);
fclose(fp10);
fclose(fp11);
fclose(fp12);
fclose(fp13);
fclose(fp14);
}
