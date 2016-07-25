#include "udf.h"
#include "thread_mem.h"
#include "seem.h"

#define min(x,y) ((x) < (y) ? (x) : (y))
#define max(x,y) ((x) > (y) ? (x) : (y))




/*GUI start*/
#if !RP_NODE
static Pointer
List_Ref(Pointer l, int n)
{
  register int i;
  for (i=0; i<n; ++i)
    {
      l = CDR(l);
    }

  return CAR(l);
}
#endif
/*GUI end*/



/*---------------------------------------------------------------------------*/
/* v1.0 does not allow for rotation in math. negative azimuthal direction    */
/* v2.0 allows for math. negative azimuthal rotation                         */
/* v3.0 URF for source terms, ramp up of rotor rpm                           */
/* v4.0 Thrust trimming in analog to lz                                      */
/* v5.0 Compute momentum coefficients, own trimming                          */
/* v5.1 Initialize xsource_old, ysource_old, and zsource_old with current    */
/*      source strength instead of 0                                         */
/* v6.0 GUI implementation. twist and type linear interpolation              */
/* v7.0 NON ideal gas for density allowed                                    */
/* v8.0 Torque calculation                                                   */
/* v9.0 Parallelized and Fluent 6.2 (boolean --> cxboolean)                  */
/*---------------------------------------------------------------------------*/
/*                 BEGIN GLOBAL INPUT VARIABLES                              */
/*---------------------------------------------------------------------------*/
/*Max. number of rotor zones 10*/
/*Max. number of blade sections 20*/
/*Max. name length for profile types 30 characters*/
/*Max. name length for airfoil tables 30 characters*/
/*Max. number of different airfoil tables 100*/
/*Max. number of AOA and coeff per cl or cd 80*/

/*GUI*/
  int nrtz;                          /*Number of Rotor zones*/
  int nbld[10];                      /*Number of blades*/
  real rrl[10];                      /*Rotor radius*/
  real rspe[10];                     /*Rotor speed*/
  real teff[10];                     /*Tip effect*/
  real dskco[10][3];                 /*Rotor disk origin 0x 1y 2z*/
  real dpit[10];                     /*Rotor disk pitch angle*/
  real dban[10];                     /*Rotor disk bank angle*/
  int fzon[10];                      /*Rotor face zone ID*/

  real bcop[10];                     /*Blade pitch collective*/
  real bcys[10];                     /*Blade pitch cyclic sin*/
  real bcyc[10];                     /*Blade pitch cyclic cos*/
  real bflco[10];                    /*Blade flapping cone*/
  real bfls[10];                     /*Blade flapping cyclic sin*/
  real bflc[10];                     /*Blade flapping cyclic cos*/

  int nsec[10];                      /*Number of blade sections along span*/
  real rin[10][20];                  /*Normalized inner radius*/
  real rout[10][20];                 /*Normalized outer radius*/
  real rsec[10][20];				 /*Normalizes radius from GUI*/
  real cin[10][20];                  /*Chord at normalized inner radius*/
  real cout[10][20];                 /*Chord at normalized outer radius*/
  real csec[10][20];				 /*Chord at normalized sectional radius (GUI)*/
  real twst[10][20];                 /*Twist*/
  char type[10][20][30];             /*Profile type name */

  int trmco[10]={0,0,0,0,0,0,0,0,0,0}; /*Trimming collective pitch 1(ON) 0(OFF)*/
  int trmcy[10]={0,0,0,0,0,0,0,0,0,0}; /*Trimming cyclic pitch 1(ON) 0(OFF)*/
  real trdf[10];                     /*Trimming damping factor*/
  int trufq[10];                     /*Trimming update frequency*/
  real ctd[10];                      /*Trimming desired ct*/
  real cmxd[10];                     /*Trimming desired cmx*/
  real cmyd[10];                     /*Trimming desired cmy*/
  int trpt;                          /*Trim around point 1(ON) 0(OFF)*/


  int ktot;							/*Number of different airfoiltables read in*/
  char file_name[100][30];			/*Name of airfoil tables, eg. airfoil.dat*/
  char check_name[100][30];			/*Name of airfoil, eg. naca0012*/
  char clorcd[100][50][10];			/*cl or cd entry*/
  float RE[100][50];				/*Airfoil Reynolds number*/
  float MA[100][50];				/*Airfoil Mach number*/
  int itot[100];					/*Number of entries per .dat file*/
  int jtot[100][50];				/*Number of aoa entries per set*/
  float aoa[100][50][80];			/*Angle of attack*/
  float coeff[100][50][80];			/*cl or cd*/
  cxboolean rho_const;                /*true if density const, false otherwise*/

/*TUI*/
  real urf_source=0.5;				/*URF for source calculation*/
  int i_start=10;					/*No iterations for linear rspe ramp up*/
  real coef_fac=1.0;				/*Factor to calculate the thrust, momentum,
										and torque coefficients.
										coef_fac=1.0   US
										coef_fac=0.5   EU*/
  real dalpha=10.0*M_PI/180.0;		/*Trimming: perturbation angle for Jacobian
                                        in deg*/
  real limiter=0.1;				/*Max. angle change relative to dalpha*/
/*---------------------------------------------------------------------------*/
/*                 END GLOBAL INPUT VARIABLES                                */
/*---------------------------------------------------------------------------*/
  int istflag=1;					/*flag 1 at start
								       0 after first ADJUST run              */
/*---------------------------------------------------------------------------*/
/*                 BEGIN GLOBAL TRIMMING VARIABLES                           */
/*---------------------------------------------------------------------------*/
  int trufq_co[10], trufq_cy[10];
  real trdf_co[10], trdf_cy[10];
  int up_co[10], up_cy[10];
  real sldity[10];
/*---------------------------------------------------------------------------*/
/*                   END GLOBAL TRIMMING VARIABLES                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                 BEGIN GLOBAL MEMORY                                       */
/*---------------------------------------------------------------------------*/
#if !RP_HOST
 int czon[10];                      /*Rotor cell zone ID*/
 char factor[10][30];               /*Memory: Geometry weighting factor*/
 char xsource[10][30];				/*Memory: Source x-component*/
 char ysource[10][30];				/*Memory: Source y-component*/
 char zsource[10][30];				/*Memory: Source z-component*/

 char xsource_old[10][30];	/*Memory: Source x-component previous timestep UR*/
 char ysource_old[10][30];	/*Memory: Source y-component previous timestep UR*/
 char zsource_old[10][30];	/*Memory: Source z-component previous timestep UR*/
#endif
/*---------------------------------------------------------------------------*/
/*                 END GLOBAL MEMORY                                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                 PUT ALL THE FUNCTIONS HERE                                */
/*---------------------------------------------------------------------------*/





#if !RP_HOST
/*---------------------------------------------------------------------------*/
 void get_solidity(real sldity_ptr[])
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  int i,j;
  real area, area_tot;

  Message0("%d get_solidity \n",myid);

  /*Loop through all zones*/
  i=0;
  while (i < nrtz)
  {
    area_tot=0.0;
	j=0;
	while (j < nsec[i])
	{
      area=0.5*(cout[i][j]+cin[i][j])*(rout[i][j]-rin[i][j])*rrl[i];

      area_tot=area_tot + area;

	  j += 1;
	}
	sldity_ptr[i]=nbld[i]*area_tot/(M_PI*rrl[i]*rrl[i]);

	i += 1;
  }
}





/*---------------------------------------------------------------------------*/
 void obtain_cell_id(int czon[])
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  int i, icell_f,icell_c0, icell_c1;
  Thread *tff = NULL;
  Thread *tc0 = NULL, *tc1 = NULL;
  face_t ff;
  cell_t cc;
  int sum_icell_f,sum_icell_c0, sum_icell_c1;

  Domain *domain;
  domain = Get_Domain(1);

  Message0("%d obtain_cell_id \n",myid);
  i=0;
  while (i < nrtz)
  {
     tff = Lookup_Thread(domain,fzon[i]);
     /*Message(" %d lookupthread test %d %d \n",myid,fzon[i],THREAD_ID(tff));*/

     icell_f=0;
     begin_f_loop(ff,tff)
       if PRINCIPAL_FACE_P(ff,tff) icell_f += 1;
     end_f_loop(ff,tff)

     tc0=F_C0_THREAD(fzon[i],tff);
     tc1=F_C1_THREAD(fzon[i],tff);

     icell_c0=0;
     begin_c_loop_int(cc,tc0)
       icell_c0 += 1;
     end_c_loop_int(cc,tc0)

     icell_c1=0;
     begin_c_loop_int(cc,tc1)
       icell_c1 += 1;
     end_c_loop_int(cc,tc1)


     sum_icell_f = PRF_GISUM1(icell_f);
     sum_icell_c0 = PRF_GISUM1(icell_c0);
     sum_icell_c1 = PRF_GISUM1(icell_c1);


     /*Message("%d icell_f = %d, icell_c0 = %d, icell_c1 = %d \n",myid, icell_f,icell_c0,icell_c1);*/
     Message0("  %d sum_icell_f = %d, sum_icell_c0 = %d, sum_icell_c1 = %d \n",
		     myid, sum_icell_f,sum_icell_c0,sum_icell_c1);

	 if (sum_icell_c0 == sum_icell_f)
     {
       czon[i]=THREAD_ID(tc0);
       Message0("  %d Rotor %d: czon %d \n",myid,i+1,czon[i]);
     }
     else if (sum_icell_c1 == sum_icell_f)
     {
       czon[i]=THREAD_ID(tc1);
       Message0("  %d Rotor %d: czon %d \n",myid,i+1,czon[i]);
     }
     else
     {
       Message0("  %d ERROR: Cell ID not determined \n",myid);
     }

     i += 1;
  }
}


/*---------------------------------------------------------------------------*/
 void allocate_memory(char factor[][30], char xsource[][30],
	                  char ysource[][30], char zsource[][30],
					  char xsource_old[][30],
                      char ysource_old[][30], char zsource_old[][30])
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{

  int i, count, sum_count;
  Thread *tc = NULL;
  cell_t cc;
  real *xsource_old_ptr,*ysource_old_ptr,*zsource_old_ptr;

  Domain *domain;
  domain = Get_Domain(1);

  Message0("%d allocate_memory \n",myid);


  (void)strcpy(factor[0],"factor0");
  (void)strcpy(factor[1],"factor1");
  (void)strcpy(factor[2],"factor2");
  (void)strcpy(factor[3],"factor3");
  (void)strcpy(factor[4],"factor4");
  (void)strcpy(factor[5],"factor5");
  (void)strcpy(factor[6],"factor6");
  (void)strcpy(factor[7],"factor7");
  (void)strcpy(factor[8],"factor8");
  (void)strcpy(factor[9],"factor9");

  (void)strcpy(xsource[0],"xsource0");
  (void)strcpy(xsource[1],"xsource1");
  (void)strcpy(xsource[2],"xsource2");
  (void)strcpy(xsource[3],"xsource3");
  (void)strcpy(xsource[4],"xsource4");
  (void)strcpy(xsource[5],"xsource5");
  (void)strcpy(xsource[6],"xsource6");
  (void)strcpy(xsource[7],"xsource7");
  (void)strcpy(xsource[8],"xsource8");
  (void)strcpy(xsource[9],"xsource9");

  (void)strcpy(ysource[0],"ysource0");
  (void)strcpy(ysource[1],"ysource1");
  (void)strcpy(ysource[2],"ysource2");
  (void)strcpy(ysource[3],"ysource3");
  (void)strcpy(ysource[4],"ysource4");
  (void)strcpy(ysource[5],"ysource5");
  (void)strcpy(ysource[6],"ysource6");
  (void)strcpy(ysource[7],"ysource7");
  (void)strcpy(ysource[8],"ysource8");
  (void)strcpy(ysource[9],"ysource9");

  (void)strcpy(zsource[0],"zsource0");
  (void)strcpy(zsource[1],"zsource1");
  (void)strcpy(zsource[2],"zsource2");
  (void)strcpy(zsource[3],"zsource3");
  (void)strcpy(zsource[4],"zsource4");
  (void)strcpy(zsource[5],"zsource5");
  (void)strcpy(zsource[6],"zsource6");
  (void)strcpy(zsource[7],"zsource7");
  (void)strcpy(zsource[8],"zsource8");
  (void)strcpy(zsource[9],"zsource9");


  (void)strcpy(xsource_old[0],"xsource_old0");
  (void)strcpy(xsource_old[1],"xsource_old1");
  (void)strcpy(xsource_old[2],"xsource_old2");
  (void)strcpy(xsource_old[3],"xsource_old3");
  (void)strcpy(xsource_old[4],"xsource_old4");
  (void)strcpy(xsource_old[5],"xsource_old5");
  (void)strcpy(xsource_old[6],"xsource_old6");
  (void)strcpy(xsource_old[7],"xsource_old7");
  (void)strcpy(xsource_old[8],"xsource_old8");
  (void)strcpy(xsource_old[9],"xsource_old9");

  (void)strcpy(ysource_old[0],"ysource_old0");
  (void)strcpy(ysource_old[1],"ysource_old1");
  (void)strcpy(ysource_old[2],"ysource_old2");
  (void)strcpy(ysource_old[3],"ysource_old3");
  (void)strcpy(ysource_old[4],"ysource_old4");
  (void)strcpy(ysource_old[5],"ysource_old5");
  (void)strcpy(ysource_old[6],"ysource_old6");
  (void)strcpy(ysource_old[7],"ysource_old7");
  (void)strcpy(ysource_old[8],"ysource_old8");
  (void)strcpy(ysource_old[9],"ysource_old9");

  (void)strcpy(zsource_old[0],"zsource_old0");
  (void)strcpy(zsource_old[1],"zsource_old1");
  (void)strcpy(zsource_old[2],"zsource_old2");
  (void)strcpy(zsource_old[3],"zsource_old3");
  (void)strcpy(zsource_old[4],"zsource_old4");
  (void)strcpy(zsource_old[5],"zsource_old5");
  (void)strcpy(zsource_old[6],"zsource_old6");
  (void)strcpy(zsource_old[7],"zsource_old7");
  (void)strcpy(zsource_old[8],"zsource_old8");
  (void)strcpy(zsource_old[9],"zsource_old9");



  i=0;
  while (i < nrtz)
  {
     tc = Lookup_Thread(domain,czon[i]);

     count = 0;
     begin_c_loop_int(cc,tc)
       count++;
     end_c_loop_int(cc,tc)

	 sum_count = PRF_GISUM1(count);
     Message0("  %d Rotor %d: Number of cells in volume=%d on this node=%d\n",myid,i+1,sum_count,count);

     Alloc_Thread_Memory(tc,factor[i],sizeof(real)*count);
     Alloc_Thread_Memory(tc,xsource[i],sizeof(real)*count);
     Alloc_Thread_Memory(tc,ysource[i],sizeof(real)*count);
     Alloc_Thread_Memory(tc,zsource[i],sizeof(real)*count);

     Alloc_Thread_Memory(tc,xsource_old[i],sizeof(real)*count);
     Alloc_Thread_Memory(tc,ysource_old[i],sizeof(real)*count);
     Alloc_Thread_Memory(tc,zsource_old[i],sizeof(real)*count);

     i += 1;
  }


/*Initialize old timestep for source terms*/
  /*if (istflag == 1)
    {*/
    i=0;
    while (i < nrtz)
    {
     tc = Lookup_Thread(domain,czon[i]);

     xsource_old_ptr= Get_Thread_Memory(tc,xsource_old[i]);
     ysource_old_ptr= Get_Thread_Memory(tc,ysource_old[i]);
     zsource_old_ptr= Get_Thread_Memory(tc,zsource_old[i]);

     begin_c_loop_int(cc,tc)
       xsource_old_ptr[cc]=0.0;
       ysource_old_ptr[cc]=0.0;
       zsource_old_ptr[cc]=0.0;
     end_c_loop_int(cc,tc)

     i += 1;
    }
    /*}*/
}


/*---------------------------------------------------------------------------*/
 void geom_factor(int czon[])
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  int i, count, n;
  Thread *tc = NULL, *tf = NULL, *t = NULL;
  cell_t cc;
  face_t ff;
  real sum_area, area, A[ND_ND], xrc[ND_ND], rc;
  real *area_ptr;
  real sum_sum_area;

  Domain *domain;
  domain = Get_Domain(1);
  Message0("%d geom_factor \n",myid);
  /*Message(" the value of pi %f \n",M_PI);*/


  i=0;
  while (i < nrtz)
  {
    tc = Lookup_Thread(domain,czon[i]);
    tf = Lookup_Thread(domain,fzon[i]);
    Message0("  %d Rotor %d: czon %d fzon %d \n",myid,i+1,czon[i],fzon[i]);
    area_ptr = Get_Thread_Memory(tc,factor[i]);


    sum_area=0.0;
    begin_c_loop_int(cc,tc)
	{
	  C_CENTROID(xrc,cc,tc);
      rc=sqrt( (xrc[0]-dskco[i][0])*(xrc[0]-dskco[i][0]) +
               (xrc[1]-dskco[i][1])*(xrc[1]-dskco[i][1]) +
               (xrc[2]-dskco[i][2])*(xrc[2]-dskco[i][2])
	    	 );
	  /*Message("rc %f",rc);*/

      count=0;
      c_face_loop(cc,tc,n)
	  {
	    t=C_FACE_THREAD(cc,tc,n);
	    if(t == tf)
		{
          ff = C_FACE(cc,tc,n);
		  F_AREA(A,ff,tf);
		  area=NV_MAG(A);

          count=count+1;
		}
	  }
	  if(count == 0)
	    Message0("ERROR: No area found");
	  if(count =! 1)
	    Message0("ERROR: Multiple areas found");

	  sum_area=sum_area+area;

	  area_ptr[cc]=((real) nbld[i])*area/(2.0*M_PI*rc);
	  /*area_ptr[cc]=area;*/
	}
    end_c_loop_int(cc,tc)
    sum_sum_area = PRF_GRSUM1(sum_area);
    Message0("  %d Rotor %d: Face area =%f on this node=%f \n",myid,i+1, sum_sum_area,sum_area);
    i += 1;
  }
}


/*---------------------------------------------------------------------------*/
 void start_trimming(int trufq_co[], int trufq_cy[],
                           real trdf_co[], real trdf_cy[],
                           int up_co[], int up_cy[])
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  int i;

  Message0("%d start_trimming \n",myid);



 /*Set update frequencies and damping factors for cyclic and colective as
    same*/
  i=0;
  while (i < nrtz)
    {
      trufq_co[i]=trufq[i];
      trufq_cy[i]=trufq[i];

      trdf_co[i]=trdf[i];
      trdf_cy[i]=trdf[i];

      up_co[i]=trufq_co[i];
      up_cy[i]=trufq_cy[i];

     i += 1;
    }
}

#endif



#if !RP_NODE
/*---------------------------------------------------------------------------*/
 void read_in_airfoil_tables(char check_name[][30], char clorcd[][50][10],
	                         float RE[][50], float MA[][50], int itot[],
							 int jtot[][50], float aoa[][50][80],
							 float coeff[][50][80])
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  FILE *f_ptr;
  int i, j, k;

  Message("\n read_in_airfoil_tables \n");
  Message("     -180 < AOA < 180 \n");

  k=0;
  while (k < ktot)
  {
	f_ptr = fopen(file_name[k],"r");
    fscanf(f_ptr,"%s \n",&check_name[k]);
    fscanf(f_ptr,"%d \n",&itot[k]);

    if(f_ptr == NULL)  {
    	Message("\nNull file pointer\n");
    	continue;
	}
    i=0;
    while (i < itot[k])
	{
      fscanf(f_ptr,"%s \n",&clorcd[k][i]);
      fscanf(f_ptr,"%f \n",&RE[k][i]);
      fscanf(f_ptr,"%f \n",&MA[k][i]);
      fscanf(f_ptr,"%d \n",&jtot[k][i]);

      j=0;
      while (j < jtot[k][i])
	  {
        fscanf(f_ptr,"%f %f \n",&aoa[k][i][j], &coeff[k][i][j]);
        j+=1;
	  }

	  j=1;
      while (j < jtot[k][i])
	  {
        if(aoa[k][i][j-1] > aoa[k][i][j])
          Message("ERROR: Airfoil table entries must be ordered with increasing AOA \n");

	    j+=1;
	  }

	  j=0;
      while (j < jtot[k][i])
	  {
        if (strcmp(clorcd[k][i],"cd") == 0)
		{
	      /*Message("  %s \n",clorcd[k][i]);*/
          if(coeff[k][i][j] < 0.0)
          Message("WARNING: Are you sure that cd is negative in %s, AOA %f ? \n",
	              check_name[k],aoa[k][i][j]);
		}

        if (strcmp(clorcd[k][i],"cl") == 0)
		{
	      /*Message("  %s \n",clorcd[k][i]);*/
		  if(aoa[k][i][j] < -90.0 &&  coeff[k][i][j] < 0.0)
            Message("WARNING: Are you sure that cl is negative in %s, AOA %f ? \n",
	                check_name[k],aoa[k][i][j]);

		  if(aoa[k][i][j] > 90.0 &&  coeff[k][i][j] > 0.0)
            Message("WARNING: Are you sure that cl is positive in %s, AOA %f ? \n",
	                check_name[k],aoa[k][i][j]);

          if(aoa[k][i][j] <= 90.0 && aoa[k][i][j] >= 0.0 && coeff[k][i][j] < 0.0)
            Message("WARNING: Are you sure that cl is negative in %s, AOA %f ? \n",
	                check_name[k],aoa[k][i][j]);

          if(aoa[k][i][j] >= -90.0 && aoa[k][i][j] < 0.0 && coeff[k][i][j] > 0.0)
            Message("WARNING: Are you sure that cl is positive in %s, AOA %f ? \n",
	                check_name[k],aoa[k][i][j]);
		}

	    j+=1;
	  }

	  i+=1;
	}
    k+=1;
  }

  fclose(f_ptr);
}

#endif



#if !RP_HOST
/*---------------------------------------------------------------------------*/
 void get_radial_position(int i, cell_t cc, Thread *tc, int count,
                          int count_max, real *r_pb_ptr, real *psi_pb_ptr,
						  real *r_pb2_ptr, real *x_pb_ptr, real *y_pb_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  real xrc[ND_ND], xx, yy, zz;
  real dpits, dpitc;
  real dbans, dbanc;
  real x_pb, y_pb, z_pb;
  real tmp1, tmp2;

  if(count == 0)
    Message0("  %d get_radial_position \n",myid);


  /*x y z coordinates*/
  C_CENTROID(xrc,cc,tc);
  xx = xrc[0]-dskco[i][0];
  yy = xrc[1]-dskco[i][1];
  zz = xrc[2]-dskco[i][2];


  /*Pitch "dpit[i]"*/
  dpits=sin(dpit[i]);
  dpitc=cos(dpit[i]);


  /*Bank "dban[i]"*/
  dbans=sin(dban[i]);
  dbanc=cos(dban[i]);


  x_pb= xx*dpitc                  - zz*dpits;
  *x_pb_ptr=x_pb;
  y_pb= xx*dpits*dbans + yy*dbanc + zz*dpitc*dbans;
  *y_pb_ptr=y_pb;
  z_pb= xx*dpits*dbanc - yy*dbans + zz*dpitc*dbanc;

  tmp1=sqrt (x_pb*x_pb + y_pb*y_pb + z_pb*z_pb);
  tmp1=tmp1/rrl[i];
  *r_pb_ptr=tmp1;

  *r_pb2_ptr=sqrt (x_pb*x_pb + y_pb*y_pb);

  tmp2=atan2(y_pb,x_pb);
  if(tmp2 < 0.0)
    tmp2=2.0*M_PI+tmp2;
  *psi_pb_ptr=tmp2;
  /*Now azimuthal angle [0,2pi]*/
  /*Message("  r_pb %f     psi_pb %f \n",r_pb_ptr, psi_pb_ptr*180/M_PI);*/

}



/*---------------------------------------------------------------------------*/
 void vel_xyz_to_blade(int i, cell_t cc, Thread *tc, int count,
                       int count_max, real psi_pb, real r_pb2,
					   real *Usc_ptr, real *Utc_ptr, real *Unc_ptr,
					   real *aeps_ptr, real *Utotal_ptr, real *beta_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  real Ux, Uy, Uz;
  real dpits, dpitc;
  real dbans, dbanc;
  real Ux_pb, Uy_pb, Uz_pb;
  real psi_pbs, psi_pbc;
  real Ur_pb, Ut_pb;
  real beta, betas, betac;
  real tmp,tmp1,Utc,Unc,aeps;
  real Ut_omega, Ut_total, Utotal;
  int i_count;
  real tmp2,tmp3,tmp4;

  if(count == 0)
    Message0("  %d vel_xyz_to_blade \n",myid);


  /*Velocities in x y z coordinates*/
  Ux=C_U(cc,tc);
  Uy=C_V(cc,tc);
  Uz=C_W(cc,tc);


  /*Pitch "dpit[i]"*/
  dpits=sin(dpit[i]);
  dpitc=cos(dpit[i]);


  /*Bank "dban[i]"*/
  dbans=sin(dban[i]);
  dbanc=cos(dban[i]);

  Ux_pb= Ux*dpitc                  - Uz*dpits;
  Uy_pb= Ux*dpits*dbans + Uy*dbanc + Uz*dpitc*dbans;
  Uz_pb= Ux*dpits*dbanc - Uy*dbans + Uz*dpitc*dbanc;


  /*Polar coordinates in pitch/bank plane*/
  /*Message("   psi_pb %f \n", psi_pb*180/M_PI);*/
  psi_pbs=sin(psi_pb);
  psi_pbc=cos(psi_pb);

  Ur_pb= Ux_pb*psi_pbc  + Uy_pb*psi_pbs;
  Ut_pb= -Ux_pb*psi_pbs + Uy_pb*psi_pbc;


  /*Flapping and coning plane*/
  /*Message("   bflco %f bfls %f bflc %f \n", bflco[i], bfls[i], bflc[i]);*/
  beta= bflco[i] - bflc[i]*psi_pbc - bfls[i]*psi_pbs;
  *beta_ptr=beta;
  betas=sin(beta);
  betac=cos(beta);
  if(rspe[i] > 0.0)
	  tmp1=-1.0;
  if(rspe[i] < 0.0)
	  tmp1=1.0;


  tmp= Ur_pb*betac  +       + Uz_pb*betas;
  *Usc_ptr=tmp;
  Utc=              tmp1*Ut_pb;
  *Utc_ptr=Utc;
  Unc= -Ur_pb*betas         + Uz_pb*betac;
  *Unc_ptr=Unc;


  i_count=N_ITER;
  /*Message("i_start %d i_count %d \n",i_start,i_count);*/
  if(i_count >= i_start)
  {
	  Ut_omega= fabs(rspe[i])*r_pb2;
	  /*Message("i_count > i_start");*/
  }
  else
  {
      tmp2=i_count;
	  tmp3=i_start;
	  tmp4=tmp2/tmp3;
	  Ut_omega= fabs(rspe[i])*r_pb2*tmp4;
  	  /*Message("i_count < i_start %f",tmp4);*/
  }

  Ut_total= Ut_omega+Utc;

  Utotal= sqrt(Ut_total*Ut_total + Unc*Unc);
  *Utotal_ptr=Utotal;

  /*Angle epsilon*/
  /*-pi < aeps < pi*/
  aeps=atan2(Unc,fabs(Ut_total));
  *aeps_ptr=aeps;
  if(aeps < -M_PI)
    *aeps_ptr=2.0*M_PI+aeps;
  if(aeps > M_PI)
    *aeps_ptr=aeps - 2.0*M_PI;

}



/*---------------------------------------------------------------------------*/
 void get_pitch(int i, cell_t cc, Thread *tc, int count, real r_pb,
	            real psi_pb, real *theta_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  int j;
  real psi_pbs, psi_pbc;
  real theta, tmp;

  if(count == 0)
    Message0("  %d get_pitch \n",myid);

  psi_pbs=sin(psi_pb);
  psi_pbc=cos(psi_pb);


  /*Twist*/
  /*Assumed to be linear between blade section*/
  /*Loop through all blade sections*/
  j=0;
  while (j < nsec[i])
  {
	  if (r_pb > rout[i][j])
         j += 1;
	  else if (r_pb > rin[i][j])
	  {
		  /*Message("%f %f %f \n",rin[i][j],r_pb,rout[i][j]);*/
          break;
	  }
	  else
		  Message("ERROR: Blade section radius not found. \n");

  }
  /*Message("%f %f %f %f \n",rin[i][j],r_pb,rout[i][j],twst[i][j]*180/M_PI);*/
  /*Message("%f %f %f %f %f \n",rsec[i][j],r_pb,rsec[i][j+1],
	                          twst[i][j]*180/M_PI,twst[i][j+1]*180/M_PI);*/

  tmp=twst[i][j]+(twst[i][j+1]-twst[i][j])/(rout[i][j]-rin[i][j])*
	                   (r_pb-rin[i][j]);
  /*Message("%f %f %f %f %f %f \n",rsec[i][j],r_pb,rsec[i][j+1],
	                 twst[i][j]*180/M_PI,tmp*180/M_PI,twst[i][j+1]*180/M_PI);*/

  theta= bcop[i] + tmp - bcyc[i]*psi_pbc - bcys[i]*psi_pbs;
  *theta_ptr=theta;

  /*-pi < theta < pi*/
  if(theta < -M_PI)
    *theta_ptr=2.0*M_PI+theta;
  if(theta > M_PI)
    *theta_ptr=theta - 2.0*M_PI;
}


/*---------------------------------------------------------------------------*/
 void get_chord(int i, cell_t cc, Thread *tc, int count, real r_pb,
	            real *chord_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
 int j;
 real chord;


  if(count == 0)
    Message0("  %d get_chord \n",myid);



  /*Chord*/
  /*Assumed to be linear per blade section*/
  /*Loop through all blade sections*/
  j=0;
  while (j < nsec[i])
  {
	  if (r_pb > rout[i][j])
         j += 1;
	  else if (r_pb > rin[i][j])
	  {
		  /*Message("%f %f %f \n",rin[i][j],r_pb,rout[i][j]);*/
          break;
	  }
	  else
		  Message("ERROR: Blade section radius not found. \n");

  }

  *chord_ptr=cin[i][j]+(cout[i][j]-cin[i][j])/(rout[i][j]-rin[i][j])*
	                   (r_pb-rin[i][j]);


  /*Message("%f %f %f | %f %f %f \n",rin[i][j],r_pb,rout[i][j],
	       cin[i][j],chord,cout[i][j]);*/
}



/*---------------------------------------------------------------------------*/
 void get_re(int i, cell_t cc, Thread *tc, int count, real chord, real Utotal,
	         real *Re_t_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{

  if(count == 0)
    Message0("  %d get_re \n",myid);

/*C_MU_L(cc,tc) laminar viscosity (in properies panel)
C_MU_T(cc,tc) turbulent viscosity (computed using turbulence model)
C_MU_EFF(cc,tc) effective viscosity (sum of laminar and turbulent)
C_R(cc,tc) density*/
/*Message("mul %f mut %f mueff %f rho %f \n",
		C_MU_L(cc,tc), C_MU_T(cc,tc), C_MU_EFF(cc,tc), C_R(cc,tc));*/


  *Re_t_ptr=Utotal*chord*C_R(cc,tc) / C_MU_L(cc,tc);
}



/*---------------------------------------------------------------------------*/
 void get_ma(int i, cell_t cc, Thread *tc, int count, real Utotal,
	         real *Ma_t_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  real kappa;

  if(count == 0)
    Message0("  %d get_ma \n",myid);

/*Message("R %f cp %f \n",C_RGAS(cc,tc), C_CP(cc,tc));*/

  if (rho_const)
  {
	  *Ma_t_ptr= 0.2;
      /*Message("rho is const");*/
  }
  else
  {
	  kappa=C_CP(cc,tc) / (C_CP(cc,tc) - C_RGAS(cc,tc));
      *Ma_t_ptr=Utotal/(sqrt(kappa*C_RGAS(cc,tc)*C_T(cc,tc)));
      /*Message("rho is ideal gas");*/
  }
}



/*---------------------------------------------------------------------------*/
 void get_aoa(int i, cell_t cc, Thread *tc, int count, real aeps,
	         real theta, real *alpha_t_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{

  if(count == 0)
    Message0("  %d get_aoa \n",myid);

    *alpha_t_ptr=aeps+theta;
}



/*---------------------------------------------------------------------------*/
 void get_cl(int i, cell_t cc, Thread *tc, int count, real r_pb,
	         real Ma_t, real Re_t, real alpha_t, real *CL_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  int j,k,it,jt;
  real Re_min, Re_max, Ma_min, Ma_max;
  int ire_min, ire_max, ima_min, ima_max;
  real alpha_min, alpha_max;
  int ial_max, ial_min;
  real clre_min, clre_max, clre, clma_min, clma_max, clma, cl, rfac;
  int iflag;
  real cl_m, cl_p;

  if(count == 0)
    Message0("  %d get_cl \n",myid);


  rfac=0.5;		/*0 --> cl=clma, 1 --> cl=clre*/


  /*Re_t=90000;*/
  /*Ma_t=0.96;*/
  /*alpha_t=3.0;*/

  alpha_t=alpha_t*180.0/M_PI;


  /*Re_min=min(1.2,Re_t);
  Re_max=max(1.3,Re_t);
  Message("Re_min %f Re_max %f \n",Re_min,Re_max);*/

  Re_min=1000000000.0;
  Re_max=-1000000000.0;
  Ma_min=1000000000.0;
  Ma_max=-1000000000.0;


  /*Assumed to be linear per blade section*/
  /*Loop through all blade sections*/
  j=0;
  iflag=0;
  while (j < nsec[i])
  {
	  if (r_pb > rout[i][j])
         j += 1;
	  else if (r_pb > rin[i][j])
	  {
		  /*Message("%f %f %f \n",rin[i][j],r_pb,rout[i][j]);*/
          break;
	  }
	  else
		  Message("ERROR: Blade section radius not found. \n");

  }

  /*Message("%f %f %f %s \n",rin[i][j],r_pb,rout[i][j],type[i][j]);*/




  k=0;
  while (k < ktot)
  {
    if (strcmp(check_name[k], type[i][j]) == 0 ||
		strcmp(check_name[k], type[i][j+1]) == 0)
	{
	  /*Message("  check %s type %s \n",check_name[k], type[i][j]);*/
      /*Message("  %d \n",itot[k]);*/


        it=0;
        while (it < itot[k])
		{
          if (strcmp(clorcd[k][it],"cl") == 0)
		  {
			/*Message("  %s \n",clorcd[k][it]);*/
            if (RE[k][it] < Re_min)
			{
			  Re_min=RE[k][it];
			  ire_min=it;
			}
            if (RE[k][it] > Re_max)
			{
			  Re_max=RE[k][it];
			  ire_max=it;
			}

            if (MA[k][it] < Ma_min)
			{
			  Ma_min=MA[k][it];
			  ima_min=it;
			}
            if (MA[k][it] > Ma_max)
			{
			  Ma_max=MA[k][it];
			  ima_max=it;
			}
		  }
           it+=1;
		}
        /*Message("Re_min %f Re_t %f Re_max %f \n",Re_min,Re_t,Re_max);
        Message("Ma_min %f Ma_t %f Ma_max %f \n",Ma_min,Ma_t,Ma_max);
        Message("Re_min %f Re_t %f Re_max %f \n",
			     RE[k][ire_min],Re_t,RE[k][ire_max]);
        Message("Ma_min %f Ma_t %f Ma_max %f \n",
			     MA[k][ima_min],Ma_t,MA[k][ima_max]);*/


		jt=0;
        if (aoa[k][ire_min][jt] > alpha_t)
		  Message("ERROR: Start AOA in airfoil table too big k %d i %d \n",
		           k, ire_min);

        if (aoa[k][ire_max][jt] > alpha_t)
		  Message("ERROR: Start AOA in airfoil table too big k %d i %d \n",
		           k, ire_max);

       if (aoa[k][ima_min][jt] > alpha_t)
		  Message("ERROR: Start AOA in airfoil table too big k %d i %d \n",
		           k, ima_min);

        if (aoa[k][ima_max][jt] > alpha_t)
		  Message("ERROR: Start AOA in airfoil table too big k %d i %d \n",
		           k, ima_max);



		jt=jtot[k][ire_min]-1;
		if (aoa[k][ire_min][jt] < alpha_t)
		  Message("ERROR: End AOA in airfoil table too small k %d i %d \n",
		          k, ire_min);

		jt=jtot[k][ire_max]-1;
		if (aoa[k][ire_max][jt] < alpha_t)
		  Message("ERROR: End AOA in airfoil table too small k %d i %d \n",
		          k, ire_max);

		jt=jtot[k][ima_min]-1;
		if (aoa[k][ima_min][jt] < alpha_t)
		  Message("ERROR: End AOA in airfoil table too small k %d i %d \n",
		          k, ima_min);

		jt=jtot[k][ima_max]-1;
		if (aoa[k][ima_max][jt] < alpha_t)
		  Message("ERROR: End AOA in airfoil table too small k %d i %d \n",
		          k, ima_max);





		/*------------------------------------------------------------------*/
        if ((Re_t > Re_min) && (Re_t < Re_max))
		{
			/*Message("RE interpolation \n");*/


			jt=0;
            while (jt < jtot[k][ire_min])
		  {
 			  if (aoa[k][ire_min][jt] > alpha_t)
			  {
				clre_min=coeff[k][ire_min][jt]+
					     (aoa[k][ire_min][jt] - alpha_t)/
					     (aoa[k][ire_min][jt] - aoa[k][ire_min][jt-1]) *
					 	 (coeff[k][ire_min][jt-1] - coeff[k][ire_min][jt]);
				break;
			  }
			  jt+=1;
		  }

			jt=0;
            while (jt < jtot[k][ire_max])
		  {

			  if (aoa[k][ire_max][jt] > alpha_t)
			  {
				clre_max=coeff[k][ire_max][jt]+
					     (aoa[k][ire_max][jt] - alpha_t)/
					     (aoa[k][ire_max][jt] - aoa[k][ire_max][jt-1]) *
					 	 (coeff[k][ire_max][jt-1] - coeff[k][ire_max][jt]);
				break;
			  }
              jt+=1;
		  }
			/*Message("clre_min %f clre_max %f \n",clre_min, clre_max);*/

			clre=clre_max+(Re_max-Re_t)/(Re_max-Re_min)*(clre_min-clre_max);

			/*Message("clre %f \n",clre);*/
		}
        /*------------------------------------------------------------------*/
        if (Re_t <= Re_min)
		{
			/*Message("Re_t smaller/equal \n");*/

			jt=0;
            while (jt < jtot[k][ire_min])
		  {
              if (aoa[k][ire_min][jt] > alpha_t)
			  {
				clre=coeff[k][ire_min][jt]+
					     (aoa[k][ire_min][jt] - alpha_t)/
					     (aoa[k][ire_min][jt] - aoa[k][ire_min][jt-1]) *
					 	 (coeff[k][ire_min][jt-1] - coeff[k][ire_min][jt]);
				break;
			  }
			  jt+=1;
		  }

			/*Message("clre = clre_min = %f \n",clre);*/

		}
        /*------------------------------------------------------------------*/
        if (Re_t >= Re_max)
		{
			/*Message("Re_t greater/equal \n");*/

			jt=0;
            while (jt < jtot[k][ire_max])
		  {

			  if (aoa[k][ire_max][jt] > alpha_t)
			  {
				clre=coeff[k][ire_max][jt]+
					     (aoa[k][ire_max][jt] - alpha_t)/
					     (aoa[k][ire_max][jt] - aoa[k][ire_max][jt-1]) *
					 	 (coeff[k][ire_max][jt-1] - coeff[k][ire_max][jt]);
				break;
			  }
              jt+=1;
		  }

			/*Message("clre = clre_max = %f \n",clre);*/

		}
        /*------------------------------------------------------------------*/



		/*------------------------------------------------------------------*/
		if ((Ma_t > Ma_min) && (Ma_t < Ma_max))
		{
			 /*Message("MA interpolation \n");*/


			jt=0;
            while (jt < jtot[k][ima_min])
		  {
              if (aoa[k][ima_min][jt] > alpha_t)
			  {
				clma_min=coeff[k][ima_min][jt]+
					     (aoa[k][ima_min][jt] - alpha_t)/
					     (aoa[k][ima_min][jt] - aoa[k][ima_min][jt-1]) *
					 	 (coeff[k][ima_min][jt-1] - coeff[k][ima_min][jt]);
				break;
			  }
			  jt+=1;
		  }


			jt=0;
            while (jt < jtot[k][ima_max])
		  {

			  if (aoa[k][ima_max][jt] > alpha_t)
			  {
				clma_max=coeff[k][ima_max][jt]+
					     (aoa[k][ima_max][jt] - alpha_t)/
					     (aoa[k][ima_max][jt] - aoa[k][ima_max][jt-1]) *
					 	 (coeff[k][ima_max][jt-1] - coeff[k][ima_max][jt]);
				break;
			  }
              jt+=1;
		  }
			/*Message("clma_min %f clma_max %f \n",clma_min, clma_max);*/

			clma=clma_max+(Ma_max-Ma_t)/(Ma_max-Ma_min)*(clma_min-clma_max);

			/*Message("clma %f \n",clma);*/
 		}
        /*------------------------------------------------------------------*/
        if (Ma_t <= Ma_min)
		{
			/*Message("Ma_t smaller/equal \n");*/

			jt=0;
            while (jt < jtot[k][ima_min])
		  {
              if (aoa[k][ima_min][jt] > alpha_t)
			  {
				clma=coeff[k][ima_min][jt]+
					     (aoa[k][ima_min][jt] - alpha_t)/
					     (aoa[k][ima_min][jt] - aoa[k][ima_min][jt-1]) *
					 	 (coeff[k][ima_min][jt-1] - coeff[k][ima_min][jt]);
				break;
			  }
			  jt+=1;
		  }
			/*Message("clma = clma_min = %f \n",clma);*/

		}
        /*------------------------------------------------------------------*/
        if (Ma_t >= Ma_max)
		{
			/*Message("Ma_t greater/equal \n");*/
			jt=0;
            while (jt < jtot[k][ima_max])
		  {

			  if (aoa[k][ima_max][jt] > alpha_t)
			  {
				clma=coeff[k][ima_max][jt]+
					     (aoa[k][ima_max][jt] - alpha_t)/
					     (aoa[k][ima_max][jt] - aoa[k][ima_max][jt-1]) *
					 	 (coeff[k][ima_max][jt-1] - coeff[k][ima_max][jt]);
				break;
			  }
              jt+=1;
		  }

			/*Message("clma = clma_max = %f \n",clma);*/

		}
        /*------------------------------------------------------------------*/

        if (strcmp(check_name[k], type[i][j]) == 0)
		{
			cl_m=clre*rfac+(1.0-rfac)*clma;
			iflag=iflag+1;
		    /*Message("  check %s type %s \n",check_name[k], type[i][j]);*/
		}

		if (strcmp(check_name[k], type[i][j+1]) == 0)
		{
			cl_p=clre*rfac+(1.0-rfac)*clma;
			iflag=iflag+1;
		    /*Message("  check %s type %s \n",check_name[k], type[i][j+1]);*/
		}

        if (iflag == 2) break;
	}
	k+=1;
  }
  cl = cl_m + (cl_p-cl_m)/(rout[i][j]-rin[i][j])*(r_pb-rin[i][j]);
  *CL_ptr=cl;

  /*Message("  rout %g rin %g alpha_t %g iflag %d \n",
	         rout[i][j], rin[i][j], alpha_t,iflag);
  Message("  cl_m %g cl %g cl_p %g \n",cl_m,cl,cl_p);*/
}



/*---------------------------------------------------------------------------*/
 void get_cd(int i, cell_t cc, Thread *tc, int count, real r_pb,
	         real Ma_t, real Re_t, real alpha_t, real *CD_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  int j,k,it,jt;
  real Re_min, Re_max, Ma_min, Ma_max;
  int ire_min, ire_max, ima_min, ima_max;
  real alpha_min, alpha_max;
  int ial_max, ial_min;
  real cdre_min, cdre_max, cdre, cdma_min, cdma_max, cdma, cd, rfac;
  int iflag;
  real cd_m, cd_p;

  if(count == 0)
    Message0("  %d get_cd \n",myid);


  rfac=0.5;		/*0 --> cd=cdma, 1 --> cd=cdre*/


  /*Re_t=600000;*/
  /*Ma_t=0.95;*/
  /*alpha_t=3.0;*/

  alpha_t=alpha_t*180.0/M_PI;


  /*Re_min=min(1.2,Re_t);
  Re_max=max(1.3,Re_t);
  Message("Re_min %f Re_max %f \n",Re_min,Re_max);*/

  Re_min=1000000000.0;
  Re_max=-1000000000.0;
  Ma_min=1000000000.0;
  Ma_max=-1000000000.0;


  /*Assumed to be linear per blade section*/
  /*Loop through all blade sections*/
  j=0;
  iflag=0;
  while (j < nsec[i])
  {
	  if (r_pb > rout[i][j])
         j += 1;
	  else if (r_pb > rin[i][j])
	  {
		  /*Message("%f %f %f \n",rin[i][j],r_pb,rout[i][j]);*/
          break;
	  }
	  else
		  Message("ERROR: Blade section radius not found. \n");

  }

  /*Message("%f %f %f %s \n",rin[i][j],r_pb,rout[i][j],type[i][j]);*/


  k=0;
  while (k < ktot)
  {
    if (strcmp(check_name[k], type[i][j]) == 0 ||
		strcmp(check_name[k], type[i][j+1]) == 0)
	{

      /*Message("  %d \n",itot[k]);*/


        it=0;
        while (it < itot[k])
		{
          if (strcmp(clorcd[k][it],"cd") == 0)
		  {
			/*Message("  %s \n",clorcd[k][it]);*/
            if (RE[k][it] < Re_min)
			{
			  Re_min=RE[k][it];
			  ire_min=it;
			}
            if (RE[k][it] > Re_max)
			{
			  Re_max=RE[k][it];
			  ire_max=it;
			}

            if (MA[k][it] < Ma_min)
			{
			  Ma_min=MA[k][it];
			  ima_min=it;
			}
            if (MA[k][it] > Ma_max)
			{
			  Ma_max=MA[k][it];
			  ima_max=it;
			}
		  }
           it+=1;
		}
        /*Message("Re_min %f Re_t %f Re_max %f \n",Re_min,Re_t,Re_max);
        Message("Ma_min %f Ma_t %f Ma_max %f \n",Ma_min,Ma_t,Ma_max);
        Message("Re_min %f Re_t %f Re_max %f \n",
			     RE[k][ire_min],Re_t,RE[k][ire_max]);
        Message("Ma_min %f Ma_t %f Ma_max %f \n",
			     MA[k][ima_min],Ma_t,MA[k][ima_max]);*/


		jt=0;
        if (aoa[k][ire_min][jt] > alpha_t)
		  Message("ERROR: Start AOA in airfoil table too big k %d i %d \n",
		           k, ire_min);

        if (aoa[k][ire_max][jt] > alpha_t)
		  Message("ERROR: Start AOA in airfoil table too big k %d i %d \n",
		           k, ire_max);

       if (aoa[k][ima_min][jt] > alpha_t)
		  Message("ERROR: Start AOA in airfoil table too big k %d i %d \n",
		           k, ima_min);

        if (aoa[k][ima_max][jt] > alpha_t)
		  Message("ERROR: Start AOA in airfoil table too big k %d i %d \n",
		           k, ima_max);



		jt=jtot[k][ire_min]-1;
		if (aoa[k][ire_min][jt] < alpha_t)
		  Message("ERROR: End AOA in airfoil table too small k %d i %d \n",
		          k, ire_min);

		jt=jtot[k][ire_max]-1;
		if (aoa[k][ire_max][jt] < alpha_t)
		  Message("ERROR: End AOA in airfoil table too small k %d i %d \n",
		          k, ire_max);

		jt=jtot[k][ima_min]-1;
		if (aoa[k][ima_min][jt] < alpha_t)
		  Message("ERROR: End AOA in airfoil table too small k %d i %d \n",
		          k, ima_min);

		jt=jtot[k][ima_max]-1;
		if (aoa[k][ima_max][jt] < alpha_t)
		  Message("ERROR: End AOA in airfoil table too small k %d i %d \n",
		          k, ima_max);





		/*------------------------------------------------------------------*/
        if ((Re_t > Re_min) && (Re_t < Re_max))
		{
			/*Message("RE interpolation \n");*/


			jt=0;
            while (jt < jtot[k][ire_min])
		  {
 			  if (aoa[k][ire_min][jt] > alpha_t)
			  {
				cdre_min=coeff[k][ire_min][jt]+
					     (aoa[k][ire_min][jt] - alpha_t)/
					     (aoa[k][ire_min][jt] - aoa[k][ire_min][jt-1]) *
					 	 (coeff[k][ire_min][jt-1] - coeff[k][ire_min][jt]);
				break;
			  }
			  jt+=1;
		  }

			jt=0;
            while (jt < jtot[k][ire_max])
		  {

			  if (aoa[k][ire_max][jt] > alpha_t)
			  {
				cdre_max=coeff[k][ire_max][jt]+
					     (aoa[k][ire_max][jt] - alpha_t)/
					     (aoa[k][ire_max][jt] - aoa[k][ire_max][jt-1]) *
					 	 (coeff[k][ire_max][jt-1] - coeff[k][ire_max][jt]);
				break;
			  }
              jt+=1;
		  }
			/*Message("cdre_min %f cdre_max %f \n",cdre_min, cdre_max);*/

			cdre=cdre_max+(Re_max-Re_t)/(Re_max-Re_min)*(cdre_min-cdre_max);

			/*Message("cdre %f \n",cdre);*/
		}
        /*------------------------------------------------------------------*/
        if (Re_t <= Re_min)
		{
			/*Message("Re_t smaller/equal \n");*/

			jt=0;
            while (jt < jtot[k][ire_min])
		  {
              if (aoa[k][ire_min][jt] > alpha_t)
			  {
				cdre=coeff[k][ire_min][jt]+
					     (aoa[k][ire_min][jt] - alpha_t)/
					     (aoa[k][ire_min][jt] - aoa[k][ire_min][jt-1]) *
					 	 (coeff[k][ire_min][jt-1] - coeff[k][ire_min][jt]);
				break;
			  }
			  jt+=1;
		  }

			/*Message("cdre = cdre_min = %f \n",cdre);*/

		}
        /*------------------------------------------------------------------*/
        if (Re_t >= Re_max)
		{
			/*Message("Re_t greater/equal \n");*/

			jt=0;
            while (jt < jtot[k][ire_max])
		  {

			  if (aoa[k][ire_max][jt] > alpha_t)
			  {
				cdre=coeff[k][ire_max][jt]+
					     (aoa[k][ire_max][jt] - alpha_t)/
					     (aoa[k][ire_max][jt] - aoa[k][ire_max][jt-1]) *
					 	 (coeff[k][ire_max][jt-1] - coeff[k][ire_max][jt]);
				break;
			  }
              jt+=1;
		  }

			/*Message("cdre = cdre_max = %f \n",cdre);*/

		}
        /*------------------------------------------------------------------*/



		/*------------------------------------------------------------------*/
		if ((Ma_t > Ma_min) && (Ma_t < Ma_max))
		{
			 /*Message("MA interpolation \n");*/


			jt=0;
            while (jt < jtot[k][ima_min])
		  {
              if (aoa[k][ima_min][jt] > alpha_t)
			  {
				cdma_min=coeff[k][ima_min][jt]+
					     (aoa[k][ima_min][jt] - alpha_t)/
					     (aoa[k][ima_min][jt] - aoa[k][ima_min][jt-1]) *
					 	 (coeff[k][ima_min][jt-1] - coeff[k][ima_min][jt]);
				break;
			  }
			  jt+=1;
		  }


			jt=0;
            while (jt < jtot[k][ima_max])
		  {

			  if (aoa[k][ima_max][jt] > alpha_t)
			  {
				cdma_max=coeff[k][ima_max][jt]+
					     (aoa[k][ima_max][jt] - alpha_t)/
					     (aoa[k][ima_max][jt] - aoa[k][ima_max][jt-1]) *
					 	 (coeff[k][ima_max][jt-1] - coeff[k][ima_max][jt]);
				break;
			  }
              jt+=1;
		  }
			/*Message("cdma_min %f cdma_max %f \n",cdma_min, cdma_max);*/

			cdma=cdma_max+(Ma_max-Ma_t)/(Ma_max-Ma_min)*(cdma_min-cdma_max);

			/*Message("cdma %f \n",cdma);*/
 		}
        /*------------------------------------------------------------------*/
        if (Ma_t <= Ma_min)
		{
			/*Message("Ma_t smaller/equal \n");*/

			jt=0;
            while (jt < jtot[k][ima_min])
		  {
              if (aoa[k][ima_min][jt] > alpha_t)
			  {
				cdma=coeff[k][ima_min][jt]+
					     (aoa[k][ima_min][jt] - alpha_t)/
					     (aoa[k][ima_min][jt] - aoa[k][ima_min][jt-1]) *
					 	 (coeff[k][ima_min][jt-1] - coeff[k][ima_min][jt]);
				break;
			  }
			  jt+=1;
		  }
			/*Message("cdma = cdma_min = %f \n",cdma);*/

		}
        /*------------------------------------------------------------------*/
        if (Ma_t >= Ma_max)
		{
			/*Message("Ma_t greater/equal \n");*/
			jt=0;
            while (jt < jtot[k][ima_max])
		  {

			  if (aoa[k][ima_max][jt] > alpha_t)
			  {
				cdma=coeff[k][ima_max][jt]+
					     (aoa[k][ima_max][jt] - alpha_t)/
					     (aoa[k][ima_max][jt] - aoa[k][ima_max][jt-1]) *
					 	 (coeff[k][ima_max][jt-1] - coeff[k][ima_max][jt]);
				break;
			  }
              jt+=1;
		  }

			/*Message("cdma = cdma_max = %f \n",cdma);*/

		}
        /*------------------------------------------------------------------*/

        if (strcmp(check_name[k], type[i][j]) == 0)
		{
			cd_m=cdre*rfac+(1.0-rfac)*cdma;
			iflag=iflag+1;
		    /*Message("  check %s type %s \n",check_name[k], type[i][j]);*/
		}

		if (strcmp(check_name[k], type[i][j+1]) == 0)
		{
			cd_p=cdre*rfac+(1.0-rfac)*cdma;
			iflag=iflag+1;
		    /*Message("  check %s type %s \n",check_name[k], type[i][j+1]);*/
		}

        if (iflag == 2) break;
	}
	k+=1;
  }


  cd = cd_m + (cd_p-cd_m)/(rout[i][j]-rin[i][j])*(r_pb-rin[i][j]);
  *CD_ptr=cd;

  /*Message("  rout %g rin %g alpha_t %g iflag %d \n",
	         rout[i][j], rin[i][j], alpha_t,iflag);
  Message("  cd_m %g cd %g cd_p %g \n",cd_m,cd,cd_p);*/
}



/*---------------------------------------------------------------------------*/
 void get_force(int i, cell_t cc, Thread *tc, real *factor_ptr,
                      real *sum_area_ptr, real sum_area, int count,
                      int count_max, real CL, real CD, real chord, real aeps,
					  real Utotal, real r_pb,
					  real *Ftc_ptr, real *Fnc_ptr, real *Fsc_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  real FL, FD;
  real aepss, aepsc;
  real tmp;


  if(count == 0)
    Message0("  %d get_force \n",myid);


  tmp=chord*0.5*Utotal*Utotal*C_R(cc,tc)*factor_ptr[cc];
  FL=CL*tmp;
  FD=CD*tmp;

  /*Tip effect*/
  if (r_pb > teff[i])
  {
	  FL=0.0;
      /*Message("Tip effect %f \n",r_pb);*/
  }


  aepss=sin(aeps);
  aepsc=cos(aeps);

  *Ftc_ptr= FD*aepsc - FL*aepss;
  *Fnc_ptr= FD*aepss + FL*aepsc;
  *Fsc_ptr= 0.0;


  /*tmp = sum_area+factor_ptr[cc];
  *sum_area_ptr = tmp;
  if(count == count_max-1)
     Message("    sum_area %f \n",tmp);*/
}



/*---------------------------------------------------------------------------*/
 void force_blade_to_xyz(int i, cell_t cc, Thread *tc, int count,
	                     real Ftc, real Fnc, real Fsc, real beta, real psi_pb,
						 real *Fx_ptr, real *Fy_ptr, real *Fz_ptr,
						 real *Fz_pb_ptr, real *Ft_pb_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  real betas, betac, Fr_pb, Ft_pb, Fz_pb;
  real psi_pbs, psi_pbc, Fx_pb, Fy_pb;
  real dpits, dpitc, dbans, dbanc;
  real tmp1;

  if(count == 0)
    Message0("  %d force_blade_to_xyz \n",myid);


  betas=sin(beta);
  betac=cos(beta);
  if(rspe[i] > 0.0)
	  tmp1=-1.0;
  if(rspe[i] < 0.0)
	  tmp1=1.0;

  Fr_pb= Fsc*betac -          Fnc*betas;
  Ft_pb=             tmp1*Ftc;
  Fz_pb= Fsc*betas +          Fnc*betac;


  psi_pbs=sin(psi_pb);
  psi_pbc=cos(psi_pb);

  Fx_pb= Fr_pb*psi_pbc - Ft_pb*psi_pbs;
  Fy_pb= Fr_pb*psi_pbs + Ft_pb*psi_pbc;


  /*Pitch "dpit[i]"*/
  dpits=sin(dpit[i]);
  dpitc=cos(dpit[i]);
  /*Bank "dban[i]"*/
  dbans=sin(dban[i]);
  dbanc=cos(dban[i]);

  *Fx_ptr= Fx_pb*dpitc + Fy_pb*dpits*dbans + Fz_pb*dpits*dbanc;
  *Fy_ptr=               Fy_pb*dbanc       - Fz_pb*dbans;
  *Fz_ptr=-Fx_pb*dpits + Fy_pb*dpitc*dbans + Fz_pb*dpitc*dbanc;


  *Fz_pb_ptr=Fz_pb;
  *Ft_pb_ptr=Ft_pb;
}



/*---------------------------------------------------------------------------*/
 void get_source_terms(int i, cell_t cc, Thread *tc, int count,
	                   real *xsource_ptr, real *ysource_ptr, real *zsource_ptr,
	                   real *xsource_old_ptr, real *ysource_old_ptr,
					   real *zsource_old_ptr,
					   real Fx, real Fy, real Fz, real urf_source)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  real vol;
  real xrc[ND_ND];
  int i_count;

  if(count == 0)
    Message0("  %d get_source_terms \n",myid);

  vol= C_VOLUME(cc,tc);

  i_count=N_ITER;
  /*Message("i_count %d istflag %d \n",i_count,istflag);*/
  if(istflag != 1 || i_count == 0)
  {
    xsource_ptr[cc]=(-Fx/vol)*urf_source + (1.0-urf_source)*xsource_old_ptr[cc];
    ysource_ptr[cc]=(-Fy/vol)*urf_source + (1.0-urf_source)*ysource_old_ptr[cc];
    zsource_ptr[cc]=(-Fz/vol)*urf_source + (1.0-urf_source)*zsource_old_ptr[cc];
    /*Message("start or typical");*/
  }
  else
  {
     xsource_ptr[cc]=(-Fx/vol);
     ysource_ptr[cc]=(-Fy/vol);
     zsource_ptr[cc]=(-Fz/vol);
	 /*Message("restart");*/
  }

  xsource_old_ptr[cc]=xsource_ptr[cc];
  ysource_old_ptr[cc]=ysource_ptr[cc];
  zsource_old_ptr[cc]=zsource_ptr[cc];


  /*C_CENTROID(xrc,cc,tc);

  xsource_ptr[cc]=xrc[0];
  ysource_ptr[cc]=xrc[1];
  zsource_ptr[cc]=xrc[2];*/
}



/*---------------------------------------------------------------------------*/
void get_ct(int i,real thrust,real *CT_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  real rho_ref;

  Message0("  %d get_ct \n",myid);


  rho_ref=RP_Get_Real("reference-density");
  /*Message("rho_ref = %f %f %f %f %f \n",rho_ref,
  coef_fac, rrl[i], rspe[i], M_PI);*/


  *CT_ptr=thrust/(coef_fac*rho_ref*rrl[i]*rrl[i]*M_PI*
	      rspe[i]*rspe[i]*rrl[i]*rrl[i]);
}



/*---------------------------------------------------------------------------*/
void get_cmx(int i,real Mx_pb,real *CMX_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  real rho_ref;

  Message0("  %d get_cmx \n",myid);


  rho_ref=RP_Get_Real("reference-density");
  /*Message("rho_ref = %f %f %f %f %f \n",rho_ref,
  coef_fac, rrl[i], rspe[i], M_PI);*/


  *CMX_ptr=Mx_pb/(coef_fac*rho_ref*rrl[i]*rrl[i]*M_PI*
	      rspe[i]*rspe[i]*rrl[i]*rrl[i]*rrl[i]);
}



/*---------------------------------------------------------------------------*/
void get_cmy(int i,real My_pb,real *CMY_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
  real rho_ref;

  Message0("  %d get_cmy \n",myid);


  rho_ref=RP_Get_Real("reference-density");
  /*Message("rho_ref = %f %f %f %f %f \n",rho_ref,
  coef_fac, rrl[i], rspe[i], M_PI);*/


  *CMY_ptr=My_pb/(coef_fac*rho_ref*rrl[i]*rrl[i]*M_PI*
	      rspe[i]*rspe[i]*rrl[i]*rrl[i]*rrl[i]);
}



/*---------------------------------------------------------------------------*/
 void update_trimming(int i, int trufq_co[], int trufq_cy[],
                            real trdf_co[], real trdf_cy[],
                            int up_co[], int up_cy[], real CT,
							int *flag_jac_ptr,
							real *ct_l_ptr, real *ct_s_ptr, real *ct_h_ptr,
							real *bcop_l_ptr, real *bcop_s_ptr, real *bcop_h_ptr,
							real CMX, real CMY,
							real *cmx_lc_ptr, real *cmx_ls_ptr,
							real *cmx_s_ptr,
							real *cmx_hc_ptr, real *cmx_hs_ptr,
							real *cmy_lc_ptr, real *cmy_ls_ptr,
							real *cmy_s_ptr,
							real *cmy_hc_ptr, real *cmy_hs_ptr,
							real *bcyc_l_ptr, real *bcyc_s_ptr, real *bcyc_h_ptr,
							real *bcys_l_ptr, real *bcys_s_ptr, real *bcys_h_ptr,
							real *ct_lc_ptr, real *ct_ls_ptr,
							real *ct_hc_ptr, real *ct_hs_ptr,
							real *cmx_l_ptr, real *cmx_h_ptr,
							real *cmy_l_ptr, real *cmy_h_ptr)
/*---------------------------------------------------------------------------*/
/*     								             */
/*     								             */
/*Version	Date	Name			Remarks		             */
/*---------------------------------------------------------------------------*/
{
     real delct, del_co;
	 real Ctmp, Btmp;
     real ct_l, ct_s, ct_h, bcop_l, bcop_s, bcop_h;
     real cmx_lc, cmx_ls, cmx_s, cmx_hc, cmx_hs;
	 real cmy_lc, cmy_ls, cmy_s, cmy_hc, cmy_hs;
	 real bcyc_l, bcyc_s, bcyc_h, bcys_l, bcys_s, bcys_h;
     real del_cyc, del_cys;
     real dcmx_bcys, dcmx_bcyc, dcmy_bcys, dcmy_bcyc, det_jac;
	 real ct_lc, ct_ls, ct_hc, ct_hs, cmx_l, cmx_h, cmy_l, cmy_h;
     real dct_bcop, dct_bcyc, dct_bcys, dcmx_bcop, dcmy_bcop;

	 Message("  %d update_trimming \n",myid);

/*THRUST ONLY*/
     if (trmco[i] == 1 && trmcy[i] == 0)
	 {
       if (up_co[i] < trufq_co[i])
	   {
         up_co[i] += 1;
	   }
	   else
	   {
         /*Message("flag_jac %d",*flag_jac_ptr);*/
		 if (*flag_jac_ptr == 0)
		 {
		   Message("    %d Trimming collective pitch: ON \n",myid);
           /*Message("    test %d %f \n", trufq_co[i], trdf_co[i]);*/
	       /*modify_bcop();*/

		   /*store ct_s*/
           *ct_s_ptr=CT;
           *bcop_s_ptr=bcop[i];

		   Message("    %d CT(actual)= %f CT(desired)= %f \n",myid, CT, ctd[i]);

		   Message("%d CO ALPHA PLUS \n",myid);
		   bcop[i]=*bcop_s_ptr+dalpha;
		   *bcop_h_ptr=bcop[i];
           *flag_jac_ptr = 1;

		   goto COEND;
		 }

		 if (*flag_jac_ptr == 1)
		 {
		   Message("%d CO ALPHA MINUS \n",myid);

		   /*store ct_h*/
           *ct_h_ptr=CT;
		   bcop[i]=*bcop_s_ptr-dalpha;
		   *bcop_l_ptr=bcop[i];

           *flag_jac_ptr = -1;

		   goto COEND;
		 }


		 if (*flag_jac_ptr == -1)
		 {
           /*store ct_l*/
           *ct_l_ptr=CT;

		   /*determine new angle*/
		   /*lz trimming*/
		   /*delct=ctd[i]-CT;
		   del_co=3.0*delct/(M_PI*sldity[i]);
		   tmp=bcop[i];
		   bcop[i]=bcop[i]+del_co;*/

           ct_l   = *ct_l_ptr;
		   ct_s   = *ct_s_ptr;
		   ct_h   = *ct_h_ptr;
		   bcop_l = *bcop_l_ptr;
		   bcop_s = *bcop_s_ptr;
		   bcop_h = *bcop_h_ptr;


		   Ctmp=(ct_s-ct_h-(bcop_s-bcop_h)/(bcop_s-bcop_l)*(ct_s-ct_l))/
					 ((bcop_s-bcop_h)*(bcop_h-bcop_l));

		   Btmp=(ct_s-ct_l-Ctmp*(bcop_s*bcop_s - bcop_l*bcop_l))/
					 (bcop_s-bcop_l);

		   del_co=trdf_co[i]*(ctd[i]-ct_s)/(Btmp+2.*Ctmp*bcop_s);

           /*limiter*/
           if(del_co > dalpha*limiter) del_co=dalpha*limiter;
           if(del_co < -dalpha*limiter) del_co=-dalpha*limiter;


           bcop[i]=bcop_s+del_co;


		   Message("%d bcop_l %f bcop_s %f bcop_h %f \n",myid,
					   bcop_l*180/M_PI,bcop_s*180/M_PI,bcop_h*180/M_PI);
		   Message("%d ct_l %f ct_s %f ct_h %f \n",myid,ct_l,ct_s,ct_h);

		   Message("    %d old collective pitch %f \n",myid, bcop_s*180/M_PI);
		   Message("    %d new collective pitch %f \n",myid, bcop[i]*180/M_PI);
		   Message("    %d difference %f \n",myid, del_co*180/M_PI);

		   *flag_jac_ptr = 10;

		   up_co[i]=1;
           goto COEND;
		 }
	   }
	 }
     COEND:

/*MOMENT ONLY*/
	 if (trmco[i] == 0 && trmcy[i] == 1)
	 {
       if (up_cy[i] < trufq_cy[i])
	   {
         up_cy[i] += 1;
	   }
	   else
	   {
  	     if (*flag_jac_ptr == 0)
		 {
                Message("    %d Trimming cyclic pitch: ON \n",myid);
                /*Message("    test %d %f \n", trufq_cy[i], trdf_cy[i]);*/
	            /*modify_bcys_and_bcyc();*/


				/*store cmx_s cmy_s*/
                *cmx_s_ptr=CMX;
                *cmy_s_ptr=CMY;

                *bcys_s_ptr = bcys[i];
                *bcyc_s_ptr = bcyc[i];


				Message("    %d CMX(actual)= %f CMX(desired)= %f \n",myid, CMX, cmxd[i]);
				Message("    %d CMY(actual)= %f CMY(desired)= %f \n",myid, CMY, cmyd[i]);


			    Message("%d CYC ALPHA PLUS \n",myid);

				bcys[i]=*bcys_s_ptr;
				bcyc[i]=*bcyc_s_ptr+dalpha;

				/**bcys_h_ptr = bcys[i];*/
                *bcyc_h_ptr = bcyc[i];

                *flag_jac_ptr = 2;

				goto CYEND;
		 }

		 if (*flag_jac_ptr == 2)
		 {

				/*store cmx_h cmy_h*/
				*cmx_hc_ptr=CMX;
				*cmy_hc_ptr=CMY;

			    Message("%d CYC ALPHA MINUS \n",myid);

				bcys[i]=*bcys_s_ptr;
				bcyc[i]=*bcyc_s_ptr-dalpha;

				/**bcys_l_ptr = bcys[i];*/
                *bcyc_l_ptr = bcyc[i];


                *flag_jac_ptr = -2;

                goto CYEND;
		 }


		 if (*flag_jac_ptr == -2)
		 {
				/*store cmx_h cmy_h*/
				*cmx_lc_ptr=CMX;
				*cmy_lc_ptr=CMY;

				Message("%d CYS ALPHA PLUS \n",myid);

				bcys[i]=*bcys_s_ptr+dalpha;
				bcyc[i]=*bcyc_s_ptr;

				*bcys_h_ptr = bcys[i];
                /**bcyc_l_ptr = bcyc[i];*/


                *flag_jac_ptr = 3;

                goto CYEND;
		 }


		 if (*flag_jac_ptr == 3)
		 {
				/*store cmx_h cmy_h*/
				*cmx_hs_ptr=CMX;
				*cmy_hs_ptr=CMY;

				Message("%d CYS ALPHA MINUS \n",myid);

				bcys[i]=*bcys_s_ptr-dalpha;
				bcyc[i]=*bcyc_s_ptr;

				*bcys_l_ptr = bcys[i];
                /**bcyc_l_ptr = bcyc[i];*/


                *flag_jac_ptr = -3;

                goto CYEND;
		 }


		 if (*flag_jac_ptr == -3)
		 {


               /*store cmx_l cmy_l*/
                *cmx_ls_ptr=CMX;
				*cmy_ls_ptr=CMY;


				/*determine new angle*/


                cmx_lc   = *cmx_lc_ptr;
                cmx_ls   = *cmx_ls_ptr;
                cmy_lc   = *cmy_lc_ptr;
                cmy_ls   = *cmy_ls_ptr;
				cmx_s    = *cmx_s_ptr;
                cmy_s    = *cmy_s_ptr;
				cmx_hc   = *cmx_hc_ptr;
				cmx_hs   = *cmx_hs_ptr;
                cmy_hc   = *cmy_hc_ptr;
                cmy_hs   = *cmy_hs_ptr;

				bcyc_l = *bcyc_l_ptr;
				bcyc_s = *bcyc_s_ptr;
				bcyc_h = *bcyc_h_ptr;
				bcys_l = *bcys_l_ptr;
				bcys_s = *bcys_s_ptr;
				bcys_h = *bcys_h_ptr;


               /*d(cmx)/d(bcys)*/
				Ctmp=(cmx_s-cmx_hs-(bcys_s-bcys_h)/(bcys_s-bcys_l)*(cmx_s-cmx_ls))/
					 ((bcys_s-bcys_h)*(bcys_h-bcys_l));
				Btmp=(cmx_s-cmx_ls-Ctmp*(bcys_s*bcys_s - bcys_l*bcys_l))/
					 (bcys_s-bcys_l);
                dcmx_bcys = Btmp+2.*Ctmp*bcys_s;


				/*d(cmx)/d(bcyc)*/
				Ctmp=(cmx_s-cmx_hc-(bcyc_s-bcyc_h)/(bcyc_s-bcyc_l)*(cmx_s-cmx_lc))/
					 ((bcyc_s-bcyc_h)*(bcyc_h-bcyc_l));
				Btmp=(cmx_s-cmx_lc-Ctmp*(bcyc_s*bcyc_s - bcyc_l*bcyc_l))/
					 (bcyc_s-bcyc_l);
                dcmx_bcyc = Btmp+2.*Ctmp*bcyc_s;


				/*d(cmy)/d(bcys)*/
				Ctmp=(cmy_s-cmy_hs-(bcys_s-bcys_h)/(bcys_s-bcys_l)*(cmy_s-cmy_ls))/
					 ((bcys_s-bcys_h)*(bcys_h-bcys_l));
				Btmp=(cmy_s-cmy_ls-Ctmp*(bcys_s*bcys_s - bcys_l*bcys_l))/
					 (bcys_s-bcys_l);
                dcmy_bcys = Btmp+2.*Ctmp*bcys_s;


				/*d(cmy)/d(bcyc)*/
				Ctmp=(cmy_s-cmy_hc-(bcyc_s-bcyc_h)/(bcyc_s-bcyc_l)*(cmy_s-cmy_lc))/
					 ((bcyc_s-bcyc_h)*(bcyc_h-bcyc_l));
				Btmp=(cmy_s-cmy_lc-Ctmp*(bcyc_s*bcyc_s - bcyc_l*bcyc_l))/
					 (bcyc_s-bcyc_l);
                dcmy_bcyc = Btmp+2.*Ctmp*bcyc_s;


                /*det_jac*/
                det_jac = dcmy_bcyc*dcmx_bcys - dcmy_bcys*dcmx_bcyc;
                /*if(det_jac <=0.0) det_jac=-1.0;
                if(det_jac > 0.0) det_jac=1.0;*/
                det_jac=1./det_jac;


                /*Message("%f %f %f %f \n",dcmy_bcyc,dcmx_bcys,dcmy_bcys,dcmx_bcyc);*/


				del_cyc = trdf_cy[i]*det_jac * (dcmx_bcys*(cmyd[i]-cmy_s) -
					                            dcmy_bcys*(cmxd[i]-cmx_s));

				/*Message("%f %f %f %f %f %f \n",trdf_cy[i],det_jac,dcmx_bcys,
					                     (cmyd[i]-cmy_s),
					                     dcmy_bcys,(cmxd[i]-cmx_s));*/
                /*Message("%f \n", dcmx_bcys*(cmyd[i]-cmy_s) -
					             dcmy_bcys*(cmxd[i]-cmx_s));*/


				del_cys = trdf_cy[i]*det_jac * (-dcmx_bcyc*(cmyd[i]-cmy_s) +
					                            dcmy_bcyc*(cmxd[i]-cmx_s));

				Message("%d del_cyc %f del_cys %f \n",myid,
					     del_cyc*180/M_PI,del_cys*180/M_PI);
                /*limiter*/
                if(del_cyc > dalpha*limiter) del_cyc=dalpha*limiter;
                if(del_cys > dalpha*limiter) del_cys=dalpha*limiter;
                if(del_cyc < -dalpha*limiter) del_cyc=-dalpha*limiter;
                if(del_cys < -dalpha*limiter) del_cys=-dalpha*limiter;


				bcyc[i]=bcyc_s+del_cyc;
                bcys[i]=bcys_s+del_cys;



				Message("%d bcyc_l %f bcyc_s %f bcyc_h %f \n",myid,
					    bcyc_l*180/M_PI,bcyc_s*180/M_PI,bcyc_h*180/M_PI);
				Message("%d bcys_l %f bcys_s %f bcys_h %f \n",myid,
					    bcys_l*180/M_PI,bcys_s*180/M_PI,bcys_h*180/M_PI);

				Message("%d cmx_lc %f cmx_s %f cmx_hc %f \n",myid,cmx_lc,cmx_s,cmx_hc);
				Message("%d cmx_ls %f cmx_s %f cmx_hs %f \n",myid,cmx_ls,cmx_s,cmx_hs);
				Message("%d cmy_lc %f cmy_s %f cmy_hc %f \n",myid,cmy_lc,cmy_s,cmy_hc);
				Message("%d cmy_ls %f cmy_s %f cmy_hs %f \n",myid,cmy_ls,cmy_s,cmy_hs);



				Message("    %d old cyclic pitch sin %f cos %f \n",myid,
					    bcys_s*180/M_PI, bcyc_s*180/M_PI);
			    Message("    %d new cyclic pitch sin %f cos %f \n",myid,
					    bcys[i]*180/M_PI, bcyc[i]*180/M_PI);
			    Message("    %d difference sin %f cos %f \n",myid,
					    del_cys*180/M_PI, del_cyc*180/M_PI);


				*flag_jac_ptr = 10;

			    up_cy[i]=1;
                goto CYEND;
         }
	   }
     }
     CYEND:


/*THRUST AND MOMENT*/
     if ((trmco[i] == 1) && (trmcy[i] == 1))
	 {
       if (up_co[i] < trufq_co[i])	/*will be updated at the collective
										  pitch update frequency*/
	   {
              up_co[i] += 1;
	   }
	   else
	   {
              /*Message("flag_jac %d",*flag_jac_ptr);*/
		      if (*flag_jac_ptr == 0)
			  {
		        Message("    %d Trimming collective and cyclic pitch: ON \n",myid);
                /*Message("    test %d %f \n", trufq_co[i], trdf_co[i]);*/
	            /*modify_bcop();*/

				/*store ct_s cmx_s cmy_s*/
                *ct_s_ptr  = CT;
                *cmx_s_ptr = CMX;
                *cmy_s_ptr = CMY;

				*bcop_s_ptr = bcop[i];
                *bcys_s_ptr = bcys[i];
                *bcyc_s_ptr = bcyc[i];

				Message("    %d CT(actual)= %f CT(desired)= %f \n",myid, CT, ctd[i]);
				Message("    %d CMX(actual)= %f CMX(desired)= %f \n",myid, CMX, cmxd[i]);
				Message("    %d CMY(actual)= %f CMY(desired)= %f \n",myid, CMY, cmyd[i]);


			    Message("%d CO ALPHA PLUS \n",myid);
			    bcop[i]=*bcop_s_ptr+dalpha;
				bcys[i]=*bcys_s_ptr;
				bcyc[i]=*bcyc_s_ptr;

				*bcop_h_ptr=bcop[i];

                *flag_jac_ptr = 1;

				goto COYEND;
			  }


			  if (*flag_jac_ptr == 1)
			  {

				/*store ct_h cmx_h cmy_h*/
				*ct_h_ptr  = CT;
                *cmx_h_ptr = CMX;
                *cmy_h_ptr = CMY;


				Message("%d CO ALPHA MINUS \n",myid);
			    bcop[i]=*bcop_s_ptr-dalpha;
				bcys[i]=*bcys_s_ptr;
				bcyc[i]=*bcyc_s_ptr;

				*bcop_l_ptr=bcop[i];

                *flag_jac_ptr = -1;

                goto COYEND;
			  }


			  if (*flag_jac_ptr == -1)
			  {
				/*store ct_l cmx_l cmy_l*/
				*ct_l_ptr  = CT;
                *cmx_l_ptr = CMX;
                *cmy_l_ptr = CMY;


			    Message("%d CYC ALPHA PLUS \n",myid);
			    bcop[i]=*bcop_s_ptr;
				bcys[i]=*bcys_s_ptr;
				bcyc[i]=*bcyc_s_ptr+dalpha;

                *bcyc_h_ptr = bcyc[i];

                *flag_jac_ptr = 2;

				goto COYEND;
			  }


			  if (*flag_jac_ptr == 2)
			  {
				/*store ct_hc cmx_hc cmy_hc*/
				*ct_hc_ptr  = CT;
                *cmx_hc_ptr = CMX;
                *cmy_hc_ptr = CMY;


			    Message("%d CYC ALPHA MINUS \n",myid);
			    bcop[i]=*bcop_s_ptr;
				bcys[i]=*bcys_s_ptr;
				bcyc[i]=*bcyc_s_ptr-dalpha;

                *bcyc_l_ptr = bcyc[i];

                *flag_jac_ptr = -2;

                goto COYEND;
			  }


			  if (*flag_jac_ptr == -2)
			  {
				/*store ct_lc cmx_lc cmy_lc*/
				*ct_lc_ptr  = CT;
                *cmx_lc_ptr = CMX;
                *cmy_lc_ptr = CMY;


				Message("%d CYS ALPHA PLUS \n",myid);
			    bcop[i]=*bcop_s_ptr;
				bcys[i]=*bcys_s_ptr+dalpha;
				bcyc[i]=*bcyc_s_ptr;

				*bcys_h_ptr = bcys[i];

                *flag_jac_ptr = 3;

                goto COYEND;
			  }


			  if (*flag_jac_ptr == 3)
			  {
				/*store ct_hs cmx_hs cmy_hs*/
				*ct_hs_ptr  = CT;
                *cmx_hs_ptr = CMX;
                *cmy_hs_ptr = CMY;


				Message("%d CYS ALPHA MINUS \n",myid);
			    bcop[i]=*bcop_s_ptr;
				bcys[i]=*bcys_s_ptr-dalpha;
				bcyc[i]=*bcyc_s_ptr;

				*bcys_l_ptr = bcys[i];

                *flag_jac_ptr = -3;

                goto COYEND;
			  }

			  if (*flag_jac_ptr == -3)
			  {
				/*store ct_ls cmx_ls cmy_ls*/
				*ct_ls_ptr  = CT;
                *cmx_ls_ptr = CMX;
                *cmy_ls_ptr = CMY;


				/*determine new angle*/

                ct_s     = *ct_s_ptr;
				cmx_s    = *cmx_s_ptr;
                cmy_s    = *cmy_s_ptr;
                ct_l     = *ct_l_ptr;
				cmx_l    = *cmx_l_ptr;
                cmy_l    = *cmy_l_ptr;
                ct_h     = *ct_h_ptr;
				cmx_h    = *cmx_h_ptr;
                cmy_h    = *cmy_h_ptr;
                ct_lc    = *ct_lc_ptr;
				cmx_lc   = *cmx_lc_ptr;
                cmy_lc   = *cmy_lc_ptr;
                ct_hc    = *ct_hc_ptr;
				cmx_hc   = *cmx_hc_ptr;
                cmy_hc   = *cmy_hc_ptr;
                ct_ls    = *ct_ls_ptr;
				cmx_ls   = *cmx_ls_ptr;
                cmy_ls   = *cmy_ls_ptr;
                ct_hs    = *ct_hs_ptr;
				cmx_hs   = *cmx_hs_ptr;
                cmy_hs   = *cmy_hs_ptr;

                bcop_l = *bcop_l_ptr;
                bcop_s = *bcop_s_ptr;
                bcop_h = *bcop_h_ptr;
				bcyc_l = *bcyc_l_ptr;
				bcyc_s = *bcyc_s_ptr;
				bcyc_h = *bcyc_h_ptr;
				bcys_l = *bcys_l_ptr;
				bcys_s = *bcys_s_ptr;
				bcys_h = *bcys_h_ptr;


               /*d(ct)/d(bcop)*/
				Ctmp=(ct_s-ct_h-(bcop_s-bcop_h)/(bcop_s-bcop_l)*(ct_s-ct_l))/
					 ((bcop_s-bcop_h)*(bcop_h-bcop_l));
				Btmp=(ct_s-ct_l-Ctmp*(bcop_s*bcop_s - bcop_l*bcop_l))/
					 (bcop_s-bcop_l);
                dct_bcop = Btmp+2.*Ctmp*bcop_s;


               /*d(cmx)/d(bcop)*/
				Ctmp=(cmx_s-cmx_h-(bcop_s-bcop_h)/(bcop_s-bcop_l)*(cmx_s-cmx_l))/
					 ((bcop_s-bcop_h)*(bcop_h-bcop_l));
				Btmp=(cmx_s-cmx_l-Ctmp*(bcop_s*bcop_s - bcop_l*bcop_l))/
					 (bcop_s-bcop_l);
                dcmx_bcop = Btmp+2.*Ctmp*bcop_s;


                /*d(cmy)/d(bcop)*/
				Ctmp=(cmy_s-cmy_h-(bcop_s-bcop_h)/(bcop_s-bcop_l)*(cmy_s-cmy_l))/
					 ((bcop_s-bcop_h)*(bcop_h-bcop_l));
				Btmp=(cmy_s-cmy_l-Ctmp*(bcop_s*bcop_s - bcop_l*bcop_l))/
					 (bcop_s-bcop_l);
                dcmy_bcop = Btmp+2.*Ctmp*bcop_s;


               /*d(ct)/d(bcys)*/
				Ctmp=(ct_s-ct_hs-(bcys_s-bcys_h)/(bcys_s-bcys_l)*(ct_s-ct_ls))/
					 ((bcys_s-bcys_h)*(bcys_h-bcys_l));
				Btmp=(ct_s-ct_ls-Ctmp*(bcys_s*bcys_s - bcys_l*bcys_l))/
					 (bcys_s-bcys_l);
                dct_bcys = Btmp+2.*Ctmp*bcys_s;


				/*d(cmx)/d(bcys)*/
				Ctmp=(cmx_s-cmx_hs-(bcys_s-bcys_h)/(bcys_s-bcys_l)*(cmx_s-cmx_ls))/
					 ((bcys_s-bcys_h)*(bcys_h-bcys_l));
				Btmp=(cmx_s-cmx_ls-Ctmp*(bcys_s*bcys_s - bcys_l*bcys_l))/
					 (bcys_s-bcys_l);
                dcmx_bcys = Btmp+2.*Ctmp*bcys_s;


				/*d(cmy)/d(bcys)*/
				Ctmp=(cmy_s-cmy_hs-(bcys_s-bcys_h)/(bcys_s-bcys_l)*(cmy_s-cmy_ls))/
					 ((bcys_s-bcys_h)*(bcys_h-bcys_l));
				Btmp=(cmy_s-cmy_ls-Ctmp*(bcys_s*bcys_s - bcys_l*bcys_l))/
					 (bcys_s-bcys_l);
                dcmy_bcys = Btmp+2.*Ctmp*bcys_s;


               /*d(ct)/d(bcyc)*/
				Ctmp=(ct_s-ct_hc-(bcyc_s-bcyc_h)/(bcyc_s-bcyc_l)*(ct_s-ct_lc))/
					 ((bcyc_s-bcyc_h)*(bcyc_h-bcyc_l));
				Btmp=(ct_s-ct_lc-Ctmp*(bcyc_s*bcyc_s - bcyc_l*bcyc_l))/
					 (bcyc_s-bcyc_l);
                dct_bcyc = Btmp+2.*Ctmp*bcyc_s;


				/*d(cmx)/d(bcyc)*/
				Ctmp=(cmx_s-cmx_hc-(bcyc_s-bcyc_h)/(bcyc_s-bcyc_l)*(cmx_s-cmx_lc))/
					 ((bcyc_s-bcyc_h)*(bcyc_h-bcyc_l));
				Btmp=(cmx_s-cmx_lc-Ctmp*(bcyc_s*bcyc_s - bcyc_l*bcyc_l))/
					 (bcyc_s-bcyc_l);
                dcmx_bcyc = Btmp+2.*Ctmp*bcyc_s;


				/*d(cmy)/d(bcyc)*/
				Ctmp=(cmy_s-cmy_hc-(bcyc_s-bcyc_h)/(bcyc_s-bcyc_l)*(cmy_s-cmy_lc))/
					 ((bcyc_s-bcyc_h)*(bcyc_h-bcyc_l));
				Btmp=(cmy_s-cmy_lc-Ctmp*(bcyc_s*bcyc_s - bcyc_l*bcyc_l))/
					 (bcyc_s-bcyc_l);
                dcmy_bcyc = Btmp+2.*Ctmp*bcyc_s;



                /*det_jac*/
                det_jac = dct_bcop*dcmy_bcyc*dcmx_bcys +
					      dct_bcyc*dcmy_bcys*dcmx_bcop +
						  dct_bcys*dcmy_bcop*dcmx_bcyc -
						  dct_bcys*dcmy_bcyc*dcmx_bcop -
						  dct_bcyc*dcmy_bcop*dcmx_bcys -
						  dct_bcop*dcmy_bcys*dcmx_bcyc;

                det_jac=1./det_jac;


				del_co=trdf_co[i]*det_jac * (
					   (dcmy_bcyc*dcmx_bcys - dcmy_bcys*dcmx_bcyc)*(ctd[i]-ct_s) +
                       (dct_bcys*dcmx_bcyc - dct_bcyc*dcmx_bcys)*(cmyd[i]-cmy_s) +
                       (dct_bcyc*dcmy_bcys - dct_bcys*dcmy_bcyc)*(cmxd[i]-cmx_s));

				del_cyc=trdf_cy[i]*det_jac * (
					   (dcmy_bcys*dcmx_bcop - dcmy_bcop*dcmx_bcys)*(ctd[i]-ct_s) +
					   (dct_bcop*dcmx_bcys - dct_bcys*dcmx_bcop)*(cmyd[i]-cmy_s) +
				       (dct_bcys*dcmy_bcop - dct_bcop*dcmy_bcys)*(cmxd[i]-cmx_s));

				del_cys=trdf_cy[i]*det_jac * (
					   (dcmy_bcop*dcmx_bcyc - dcmy_bcyc*dcmx_bcop)*(ctd[i]-ct_s) +
					   (dct_bcyc*dcmx_bcop - dct_bcop*dcmx_bcyc)*(cmyd[i]-cmy_s) +
					   (dct_bcop*dcmy_bcyc - dct_bcyc*dcmy_bcop)*(cmxd[i]-cmx_s));

				Message("%d del_co %f del_cyc %f del_cys %f \n",myid,
					     del_co*180/M_PI,del_cyc*180/M_PI,del_cys*180/M_PI);


	            /*limiter*/
                if(del_co  > dalpha*limiter) del_co =dalpha*limiter;
 				if(del_cyc > dalpha*limiter) del_cyc=dalpha*limiter;
                if(del_cys > dalpha*limiter) del_cys=dalpha*limiter;
                if(del_co  < -dalpha*limiter) del_co =-dalpha*limiter;
				if(del_cyc < -dalpha*limiter) del_cyc=-dalpha*limiter;
                if(del_cys < -dalpha*limiter) del_cys=-dalpha*limiter;


				bcop[i]=bcop_s+del_co;
				bcyc[i]=bcyc_s+del_cyc;
                bcys[i]=bcys_s+del_cys;



				Message("%d bcop_l %f bcop_s %f bcop_h %f \n",myid,
					    bcop_l*180/M_PI,bcop_s*180/M_PI,bcop_h*180/M_PI);
				Message("%d bcyc_l %f bcyc_s %f bcyc_h %f \n",myid,
					    bcyc_l*180/M_PI,bcyc_s*180/M_PI,bcyc_h*180/M_PI);
				Message("%d bcys_l %f bcys_s %f bcys_h %f \n",myid,
					    bcys_l*180/M_PI,bcys_s*180/M_PI,bcys_h*180/M_PI);

				Message("%d ct_l %f ct_s %f ct_h %f \n",myid,ct_l,ct_s,ct_h);
				Message("%d ct_lc %f ct_s %f ct_hc %f \n",myid,ct_lc,ct_s,ct_hc);
				Message("%d ct_ls %f ct_s %f ct_hs %f \n",myid,ct_ls,ct_s,ct_hs);
				Message("%d cmx_l %f cmx_s %f cmx_h %f \n",myid,cmx_l,cmx_s,cmx_h);
				Message("%d cmx_lc %f cmx_s %f cmx_hc %f \n",myid,cmx_lc,cmx_s,cmx_hc);
				Message("%d cmx_ls %f cmx_s %f cmx_hs %f \n",myid,cmx_ls,cmx_s,cmx_hs);
				Message("%d cmy_l %f cmy_s %f cmy_h %f \n",myid,cmy_l,cmy_s,cmy_h);
				Message("%d cmy_lc %f cmy_s %f cmy_hc %f \n",myid,cmy_lc,cmy_s,cmy_hc);
				Message("%d cmy_ls %f cmy_s %f cmy_hs %f \n",myid,cmy_ls,cmy_s,cmy_hs);

				Message("    %d old pitch: col. %f cyc. sin %f cos %f \n",myid,
					    bcop_s*180/M_PI, bcys_s*180/M_PI, bcyc_s*180/M_PI);
			    Message("    %d new pitch: col. %f cyc. sin %f cos %f \n",myid,
					    bcop[i]*180/M_PI, bcys[i]*180/M_PI, bcyc[i]*180/M_PI);
			    Message("    %d difference col %f sin %f cos %f \n",myid,
					    del_co*180/M_PI, del_cys*180/M_PI, del_cyc*180/M_PI);

				*flag_jac_ptr = 10;

			    up_co[i]=1;
                goto COYEND;
			  }
	   }
     }
     COYEND:;

}

#endif








/*---------------------------------------------------------------------------*/
DEFINE_ON_DEMAND(rotor_inputs)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{

  int number_of_sections, ic;
  char exst_type[100][30];
  int temp, trim,surf ;
  char *type_new[10][20];
  int col_pitch, cyc_pitch;
  real bcop_udf[10], bcyc_udf[10], bcys_udf[10];
  int trmco_dat[10], trmcy_dat[10];
  register int j,k;


  int ic2;
  /*real rsec_v[200], csec_v[200], twst_v[200];*/
  /*real dskco_0[10], dskco_1[10], dskco_2[10];*/
  /*char type_new_v[200];*/
  int itrm=0;



#if !RP_NODE
  Pointer l = rpgetvar ("rotor");


  Domain *d;
  Thread *t;
  d=Get_Domain(1);

  Message("\n\n\nIn Define on demand - rotor_inputs \n");

  nrtz = RP_Get_List_Length("rotor");

  temp = RP_Get_Integer("list-length-check");

  if(temp == 1)
	Message("\n\nPassing variables to the solver");
  else
	Message("\n\nPlease click on change/create... button to pass the variables to solver");


  for (j=0; j<nrtz; j++)
  {
    /*save old pitch angles and trimming flags from .dat file*/
    bcop_udf[j] = bcop[j];
    bcyc_udf[j] = bcyc[j];
    bcys_udf[j] = bcys[j];
    trmco_dat[j] = trmco[j];
    trmcy_dat[j] = trmcy[j];

	nbld[j] = fixnum_arg(List_Ref (List_Ref (List_Ref (l,j),1),0), "");
	rrl[j]  = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),1),1), "");
	rspe[j] = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),1),2), "");
	teff[j] = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),1),3), "");
	teff[j] = teff[j]*0.01;

    dskco[j][0] = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),1),4), "");
	/*dskco_0[j]=dskco[j][0];*/
	dskco[j][1] = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),1),5), "");
	/*dskco_1[j]=dskco[j][1];*/
	dskco[j][2] = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),1),6), "");
	/*dskco_2[j]=dskco[j][2];*/
	dpit[j] = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),1),7), "");
	dban[j] = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),1),8), "");
	bcop[j] = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),1),9), "");
	bcys[j] = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),1),10), "");
	bcyc[j] = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),1),11), "");
	bflco[j] = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),1),12), "");
	bfls[j] = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),1),13), "");
	bflc[j] = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),1),14), "");


	if(fixnum_arg(List_Ref (List_Ref (List_Ref (List_Ref (l,j),1),15),0), "") == 0)
	{
	  Message("\n Please select a surface for rotor face zone");
	  fzon[j] = 0;
	}
	else
	  fzon[j] = fixnum_arg(List_Ref (List_Ref (List_Ref (List_Ref (l,j),1),15),0), "");

	nsec[j] = fixnum_arg(List_Ref (List_Ref (List_Ref (l,j),2),0), "");

	number_of_sections = nsec[j] ;
	Message("\nafter section\n");

	for (k=0; k<number_of_sections; k++)
	{
	  rsec[j][k] = flonum_arg(List_Ref (List_Ref (List_Ref (List_Ref (l,j),2),2),k), "");
	  csec[j][k] = flonum_arg(List_Ref (List_Ref (List_Ref (List_Ref (l,j),2),3),k), "");
	  twst[j][k] = flonum_arg(List_Ref (List_Ref (List_Ref (List_Ref (l,j),2),4),k), "");
	  type_new[j][k] = string_arg(List_Ref (List_Ref (List_Ref (List_Ref (l,j),2),5),k), "");
      (void)strcpy(type[j][k],type_new[j][k]);
    }

	if (fixnum_arg(List_Ref (List_Ref (List_Ref (l,j), 3),0), "") == 1)
	{
	  itrm=1;
	  col_pitch = fixnum_arg(List_Ref (List_Ref (List_Ref (l,j),3),1), "");
	  if (col_pitch == 1)
	  {
		trmco[j] = 1;
		ctd[j]   = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),3),5), "");
	  }
	  else
	  {
		trmco[j] = 0;
	  }
	  cyc_pitch = fixnum_arg(List_Ref (List_Ref (List_Ref (l,j),3),2), "");

	  if (cyc_pitch == 1)
	  {
		trmcy[j] = 1;
		if (col_pitch == 1)
		{
	      cmxd[j]  = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),3),6), "");
		  cmyd[j]  = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),3),7), "");
		}
		else
		{
		  cmxd[j]  = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),3),5), "");
		  cmyd[j]  = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),3),6), "");
		}

	  }
	  else
	  {
		trmcy[j] = 0;
	  }

	  trufq[j] = fixnum_arg(List_Ref (List_Ref (List_Ref (l,j),3),3), "");
	  trdf[j]  = flonum_arg(List_Ref (List_Ref (List_Ref (l,j),3),4), "");
	}
  }
  /*Message("DONE");*/

  for (j=0; j<10; j++)
  {
	for (k=0; k<20; k++)
	{
      ic2 = j*20+k;
	  /*rsec_v[ic2]=rsec[j][k];*/
	  /*csec_v[ic2]=csec[j][k];*/
	  /*twst_v[ic2]=twst[j][k];*/
	  /*type_new_v[ic2]=type_new[j][k];*/
	  /*Message ("Name of the file = %s\n", type_new_v[ic2]);*/
	}
  }

#endif

/* ALL the information resides on the host. Now distribute to nodes */



/*Only host sends*/
  host_to_node_int_2(nrtz,itrm);
  host_to_node_int(nbld,10);
  host_to_node_real(rrl,10);
  host_to_node_real(rspe,10);
  host_to_node_real(teff,10);
  /*host_to_node_real(dskco_0,10);*/
  /*host_to_node_real(dskco_1,10);*/
  /*host_to_node_real(dskco_2,10);*/
  host_to_node_real(dskco,30);
  host_to_node_real(dpit,10);
  host_to_node_real(dban,10);

  host_to_node_int(trmco,10);
  host_to_node_int(trmcy,10);
  host_to_node_int(trmco_dat,10);
  host_to_node_int(trmcy_dat,10);
  host_to_node_real(trdf,10);
  host_to_node_int(trufq,10);
  host_to_node_real(ctd,10);
  host_to_node_real(cmxd,10);
  host_to_node_real(cmyd,10);

  host_to_node_real(bcop,10);
  host_to_node_real(bcop_udf,10);

  host_to_node_real(bcyc,10);
  host_to_node_real(bcyc_udf,10);
  host_to_node_real(bcys,10);
  host_to_node_real(bcys_udf,10);

  host_to_node_real(bflco,10);
  host_to_node_real(bfls,10);
  host_to_node_real(bflc,10);

  host_to_node_int(fzon,10);
  host_to_node_int(nsec,10);

  host_to_node_real(rsec,200);
  host_to_node_real(csec,200);
  host_to_node_real(twst,200);
  host_to_node_string(type,6000);



  /*Only nodes receive*/

#if !RP_HOST

  Message0 ("\n\n %d Rotor Model Inputs\n",myid);
  Message0 (" %d ------------------\n",myid);
  Message0 (" %d General Inputs \n",myid);
  Message0 (" %d ------------------\n",myid);
  Message0 (" %d Number of Rotor zones = %d\n", myid, nrtz);
  for (j=0; j<nrtz; j++)
  {
    Message0 ("\n\n %d Rotor zone %d\n", myid, j+1);
	Message0 (" %d Number of Blades = %d\n",myid, nbld[j]) ;
	Message0 (" %d Rotor Radius = %g\n",myid, rrl[j]) ;
	Message0 (" %d Rotor Speed = %g\n",myid, rspe[j]) ;
	Message0 (" %d Tip Effect = %g\n",myid, teff[j]) ;
    /*dskco[j][0]=dskco_0[j];*/
	Message0 (" %d Rotor Disk Origin - X = %g\n",myid, dskco[j][0]) ;
    /*dskco[j][1]=dskco_1[j];*/
	Message0 (" %d Rotor Disk Origin - Y = %g\n",myid, dskco[j][1]) ;
    /*dskco[j][2]=dskco_2[j];*/
	Message0 (" %d Rotor Disk Origin - Z = %g\n",myid, dskco[j][2]) ;
	Message0 (" %d Rotor Disk Pitch Angle = %g\n",myid, dpit[j]*180/M_PI) ;
	Message0 (" %d Rotor Disk Bank Angle = %g\n",myid, dban[j]*180/M_PI) ;
    if(trmco[j] == 1 && istflag != 1 || trmcy[j] == 1 && istflag != 1 ||
       trmco_dat[j] == 1 || trmcy_dat[j] == 1)
    {
	  bcop[j]=bcop_udf[j];
	  Message0 (" %d   GUI COLLECTIVE BLADE PITCH IGNORED\n",myid);
	}
	Message0 (" %d Blade Pitch - Collective = %g\n",myid, bcop[j]*180/M_PI) ;
    if(trmco[j] == 1 && istflag != 1 || trmcy[j] == 1 && istflag != 1 ||
       trmco_dat[j] == 1 || trmcy_dat[j] == 1)
    {
	  bcyc[j]=bcyc_udf[j];
	  bcys[j]=bcys_udf[j];
	  Message0 (" %d   GUI CYCLIC BLADE PITCH IGNORED\n",myid);
	}
	Message0 (" %d Blade Pitch - Cyclic Sin = %g\n",myid, bcys[j]*180/M_PI) ;
	Message0 (" %d Blade Pitch - Cyclic Cos = %g\n",myid, bcyc[j]*180/M_PI) ;
	Message0 (" %d Blade Flapping - Collective = %g\n",myid, bflco[j]*180/M_PI) ;
	Message0 (" %d Blade Flapping - Cyclic Sin = %g\n",myid, bfls[j]*180/M_PI) ;
	Message0 (" %d Blade Flapping - Cyclic Cos = %g\n",myid, bflc[j]*180/M_PI) ;
	Message0 (" %d Rotor Face Zone ID = %d\n",myid, fzon[j]) ;
	Message0 (" %d ------------------\n",myid);
	Message0 (" %d Geometry Inputs\n",myid);
	Message0 (" %d ------------------\n",myid);
	Message0 (" %d Number of Blade sections = %d\n",myid, nsec[j]);
	number_of_sections = nsec[j] ;
	for (k=0; k<number_of_sections; k++)
	{
	  Message0 ("\n %d Radius of section = %g\n",myid, rsec[j][k]) ;
	  Message0 (" %d Chord of section = %g\n",myid, csec[j][k]) ;
	  Message0 (" %d Twist of section = %g\n",myid, twst[j][k]*180/M_PI) ;
	  /*(void)strcpy(type[j][k],type_new[j][k]);*/
      /*Message ("Name of the file = %s\n", type_new[j][k]);*/
	  Message0 (" %d Name of the file = %s\n",myid, type[j][k]);
	}

	if(itrm == 1)
	{
	  Message0 (" %d ------------------\n",myid);
	  Message0 (" %d Trimming Inputs\n",myid);
	  Message0 (" %d ------------------\n",myid);
	  Message0 (" %d Collective Pitch = %d\n",myid, trmco[j]);
	  Message0 (" %d Cyclic Pitch = %d\n",myid, trmcy[j]);
	  Message0 (" %d Update Frequency = %d\n",myid, trufq[j]);
	  Message0 (" %d Damping Factor = %g\n",myid, trdf[j]);
	  if (trmco[j] == 1)
	  {
	    Message0 (" %d Desired thrust coefficient = %g\n",myid, ctd[j]);
	    Message0 (" %d ------------------\n",myid);
	  }
	  else
	  {
	    Message0 (" %d ------------------\n",myid);
	    Message0 (" %d Collective Pitch not selected\n",myid);
	    Message0 (" %d ------------------\n",myid);
	  }
	  if (trmcy[j] == 1)
	  {
	    Message0 (" %d Desired x-momentum coefficient = %g\n",myid, cmxd[j]);
	    Message0 (" %d Desired y-momentum coefficient = %g\n",myid, cmyd[j]);
	    Message0 (" %d ------------------\n",myid);
	  }
	  else
	  {
	    Message0 (" %d ------------------\n",myid);
	    Message0 (" %d Cyclic pitch not selected\n",myid);
	    Message0 (" %d -------------------------\n",myid);
	  }
	}
	else
	Message0 ("\n %d Trimming is not selected\n",myid) ;
  }





  /*CALCULATE rout, rin, cout, cin from rsec, csec*/
  for (j=0; j<nrtz; j++)
  {
    number_of_sections = nsec[j]-1 ;
    for (k=0; k<number_of_sections; k++)
	{
      rin[j][k]=rsec[j][k];
      cin[j][k]=csec[j][k];

      rout[j][k]=rsec[j][k+1];
      cout[j][k]=csec[j][k+1];
	 }
  }
  for (j=0; j<nrtz; j++)
  {
    number_of_sections = nsec[j]-1 ;
    for (k=0; k<number_of_sections; k++)
	{
      Message0("%d Rotor %d Section %d | rin %g rout %g | cin %g cout %g \n",myid,
	     	     j,k,rin[j][k], rout[j][k], cin[j][k], cout[j][k]);
	}
  }




 /*input_trimming(trmco,trmcy,trdf,trufq,ctd,cmxd,cmyd,&trpt);*/
  trpt = 0;


  /*input_airfoil_tables(&ktot,file_name);*/
  /*CALCULATE ktot*/
  ktot=1;
  for (j=0; j<nrtz; j++)
  {
    number_of_sections = nsec[j] ;
    for (k=0; k<number_of_sections; k++)
	{
      /*Message ("TEST\n") ;*/
	  for(ic=0; ic<ktot; ic++)
	  {
	    if (strcmp(type[j][k],exst_type[ic]) == 0)
	    {
		  /*Message("yes\n");*/
		  goto LABEL2;
		}
	  }
      (void)strcpy(exst_type[ktot-1],type[j][k]);
	  ktot=ktot+1;
	  LABEL2:;
	}
  }

  ktot=ktot-1;
  Message0("%d ktot %d \n",myid,ktot);
  Message0("%d nrtz %d \n",myid,nrtz);
  for(ic=0; ic<ktot; ic++)
  {
    Message0("%d Airfoils found %s\n",myid, exst_type[ic]);
    (void)strcpy(file_name[ic],exst_type[ic]);
	(void)strcat(file_name[ic],".dat");
    (void)strcpy(exst_type[ic],"");
	Message0("%d Airfoil files existing in wkdir: %s\n",myid, file_name[ic]);
  }


  get_solidity(sldity);

  /*deg_to_rad(dpit, dban, bcop, bcys, bcyc, bflco, bfls, bflc, twst, &dalpha);*/
  /*dalpha=dalpha*M_PI/180.0;*/

  obtain_cell_id(czon);

  if (istflag == 1) allocate_memory(factor,xsource,ysource,zsource,
	              xsource_old,ysource_old,zsource_old);

  geom_factor(czon);

  start_trimming(trufq_co, trufq_cy, trdf_co, trdf_cy, up_co, up_cy);
#endif

  node_to_host_int_1(ktot);
  node_to_host_string(file_name,3000);



#if !RP_NODE
  read_in_airfoil_tables(check_name, clorcd, RE, MA,
	                     itot, jtot, aoa, coeff); /*-180deg to +180deg*/


  /*istflag=1;*/

  /*determine if rho=const*/
  thread_loop_c(t,d)
  {
    if(FLUID_THREAD_P(t))
	{
	  if (DENSITY_METHOD(t) == RHO_CONSTANT)
	  {
		Message("DENSITY CONST \n");
        Message("Mach number interpolation ignored \n");
		rho_const = TRUE;
	  }
	  else
	  {
		rho_const = FALSE;
	  }
	}
  }
#endif


  /* ALL the information resides on the host. Now distribute to nodes */


  for(j=0;j<100;j++)   {
    host_to_node_float(RE[j],50);
    host_to_node_float(MA[j],50);
    host_to_node_int(jtot[j],50);
    host_to_node_string(check_name[j],30);

    for(k=0;k<50;k++)    {
      host_to_node_string(clorcd[j][k],10);
      host_to_node_float(aoa[j][k],80);
      host_to_node_float(coeff[j][k],80);
    }
  }

  host_to_node_int(itot,100);
  host_to_node_boolean_1(rho_const);

/*#if !RP_HOST
  if (rho_const)
  {
    Message("rho is const");
  }
  else
  {
    Message("rho is NOT const");
  }
#endif*/
}


/*---------------------------------------------------------------------------*/
DEFINE_ADJUST(SrcComp,domain)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{

#if !RP_HOST
  int i, count, count_max;
  Thread *tc = NULL;
  cell_t cc;
  real *factor_ptr, sum_area;
  real r_pb,psi_pb,r_pb2;
  real Usc, Utc, Unc, aeps, Utotal, beta;
  real theta, chord;
  real CL, CD;
  real Ftc, Fnc, Fsc;
  real Fx, Fy, Fz;
  real *xsource_ptr, *ysource_ptr, *zsource_ptr;
  real *xsource_old_ptr, *ysource_old_ptr, *zsource_old_ptr;
  real Re_t, Ma_t, alpha_t;
  real thrust, Fz_pb, CT;
  real x_pb, y_pb, Mx_pb, My_pb, CMX, CMY;
  int flag_jac;
  real ct_s, ct_h, ct_l, bcop_l, bcop_s, bcop_h;
  real cmx_lc, cmx_ls, cmx_s, cmx_hc, cmx_hs;
  real cmy_lc, cmy_ls, cmy_s, cmy_hc, cmy_hs;
  real bcyc_l, bcyc_s, bcyc_h, bcys_l, bcys_s, bcys_h;
  real ct_lc, ct_ls, ct_hc, ct_hs, cmx_l, cmx_h, cmy_l, cmy_h;
  real alpha_tmax, alpha_tmin;
  real torque, power, Ft_pb;
  real sum_thrust, sum_Mx_pb, sum_My_pb, sum_torque;
  real max_alpha_tmax, min_alpha_tmin;

  /*Domain *domain;*/
  domain = Get_Domain(1);


  Message0("%d In DEFINE_ADJUST - SrcComp \n",myid);

  /*Loop through all zones*/
  i=0;
  while (i < nrtz)
  {
    Message0("%d Rotor %d \n",myid,i+1);

    tc = Lookup_Thread(domain,czon[i]);
    factor_ptr = Get_Thread_Memory(tc,factor[i]);
    xsource_ptr= Get_Thread_Memory(tc,xsource[i]);
    ysource_ptr= Get_Thread_Memory(tc,ysource[i]);
    zsource_ptr= Get_Thread_Memory(tc,zsource[i]);

    xsource_old_ptr= Get_Thread_Memory(tc,xsource_old[i]);
    ysource_old_ptr= Get_Thread_Memory(tc,ysource_old[i]);
    zsource_old_ptr= Get_Thread_Memory(tc,zsource_old[i]);


	count=0;
    begin_c_loop_int(cc,tc)
      count++;
    end_c_loop_int(cc,tc)
    count_max=count;
    /*Message("count_max %d",count_max);*/

	flag_jac=0;
    LABEL1:
	  thrust=0.0;
	  torque=0.0;
	  Mx_pb=0.0;
	  My_pb=0.0;
      sum_area=0.0;
	  alpha_tmax=-1000.0;
	  alpha_tmin=1000.0;
      count=0;

      begin_c_loop_int(cc,tc)
        get_radial_position(i,cc,tc,count,count_max,&r_pb,&psi_pb,&r_pb2,
		                      &x_pb,&y_pb);
        /*Message0("  r_pb %f     psi_pb %f  x_pb %f   y_pb %f \n",
		    	    r_pb, psi_pb*180/M_PI, x_pb, y_pb);*/

	    vel_xyz_to_blade(i,cc,tc,count,count_max,psi_pb,r_pb2,
		    	             &Usc,&Utc,&Unc,&aeps,&Utotal,&beta);
        /*Message0("  Usc %f Utc %f Unc %f \n",Usc, Utc, Unc);*/
		/*Message0("aeps %f \n", aeps*180.0/M_PI);*/

	    get_pitch(i,cc,tc,count,r_pb,psi_pb,&theta);
        /*Message0("  psi_pb %f theta %f \n",psi_pb*180/M_PI, theta*180/M_PI);*/

        get_chord(i,cc,tc,count,r_pb, &chord);
		/*Message0("%f \n",chord);*/

		get_re(i,cc,tc,count,chord,Utotal,&Re_t);
		/*Message0("   Re_t %f \n",Re_t);*/

		get_ma(i,cc,tc,count,Utotal,&Ma_t);
		/*Message0("   Ma_t %f \n",Ma_t);*/

		get_aoa(i,cc,tc,count,aeps,theta,&alpha_t);
		/*Message0("   alpha_t %f \n",alpha_t*180/M_PI);*/

		get_cl(i,cc,tc,count,r_pb,Ma_t,Re_t,alpha_t,&CL);
		/*Message0("   CL %f \n",CL);*/

        get_cd(i,cc,tc,count,r_pb,Ma_t,Re_t,alpha_t,&CD);
		/*Message0("   CD %f \n",CD);*/

		get_force(i,cc,tc,factor_ptr,&sum_area,sum_area,count,count_max,
		            CL,CD,chord,aeps,Utotal,r_pb,&Ftc,&Fnc,&Fsc);
		/*Message0("Ftc=%f Fnc=%f Fsc=%f  \n",Ftc, Fnc, Fsc);*/

        force_blade_to_xyz(i,cc,tc,count,Ftc,Fnc,Fsc,beta,psi_pb,
		    	               &Fx,&Fy,&Fz,&Fz_pb,&Ft_pb);
		/*Message0("Fx= %f Fy= %f Fz= %f Fz_pb= %f \n",Fx, Fy, Fz, Fz_pb, Ft_pb);*/



        if (flag_jac == 0) get_source_terms(
			                  i,cc,tc,count,xsource_ptr,ysource_ptr,zsource_ptr,
			                  xsource_old_ptr,ysource_old_ptr,zsource_old_ptr,
			                  Fx, Fy, Fz, urf_source);

		thrust=thrust+Fz_pb;
		Mx_pb=Mx_pb+Fz_pb*y_pb;
		My_pb=My_pb+Fz_pb*x_pb;
        torque=torque+Ft_pb*sqrt(x_pb*x_pb + y_pb*y_pb);

		alpha_tmax=max(alpha_tmax,alpha_t);
		alpha_tmin=min(alpha_tmin,alpha_t);

	    count++;
      end_c_loop_int(cc,tc)


      /*Message("    sum_area %f \n",sum_area);*/
      sum_thrust = PRF_GRSUM1(thrust);
      sum_Mx_pb = PRF_GRSUM1(Mx_pb);
	  sum_My_pb = PRF_GRSUM1(My_pb);
	  sum_torque = PRF_GRSUM1(torque);
      max_alpha_tmax = PRF_GRHIGH1(alpha_tmax);
      min_alpha_tmin = PRF_GRLOW1(alpha_tmin);

      get_ct(i,sum_thrust,&CT);
      get_cmx(i,sum_Mx_pb,&CMX);
      get_cmy(i,sum_My_pb,&CMY);


	  if (flag_jac == 0)
	  {
        power=sum_torque*rspe[i];

		Message0("    %d Rotor %d Thrust %f ct= %f \n",myid,i+1,sum_thrust,CT);
		/*Message("    %d Rotor %d Thrust on node %f \n",myid,i+1,thrust);*/
        Message0("    %d Rotor %d Torque %f \n",myid,i+1,sum_torque);
        /*Message("    %d Rotor %d Torque on node %f \n",myid,i+1,torque);*/
		Message0("    %d Rotor %d Power %f \n",myid,i+1,power);
		Message0("    %d Rotor %d Mx_pb %f cmx= %f \n",myid,i+1,sum_Mx_pb,CMX);
    	/*Message("    %d Rotor %d Mx_pb on node %f \n",myid,i+1,Mx_pb);*/
		Message0("    %d Rotor %d My_pb %f cmy= %f \n",myid,i+1,sum_My_pb,CMY);
   	    /*Message("    %d Rotor %d My_pb on node %f \n",myid,i+1,My_pb);*/
		Message0("    %d Rotor %d Max AOA %f \n",myid,i+1,max_alpha_tmax*180/M_PI);
 		/*Message("    %d Rotor %d Max AOA on node %f \n",myid,i+1,alpha_tmax*180/M_PI);*/
		Message0("    %d Rotor %d Min AOA %f \n",myid,i+1,min_alpha_tmin*180/M_PI);
 		/*Message("    %d Rotor %d Max AOA on node %f \n",myid,i+1,alpha_tmin*180/M_PI);*/
	  }


	  update_trimming(i, trufq_co, trufq_cy, trdf_co, trdf_cy,
			            up_co, up_cy, CT, &flag_jac,
						&ct_l, &ct_s, &ct_h,
						&bcop_l, &bcop_s, &bcop_h,
						CMX, CMY,
                        &cmx_lc, &cmx_ls, &cmx_s, &cmx_hc, &cmx_hs,
						&cmy_lc, &cmy_ls, &cmy_s, &cmy_hc, &cmy_hs,
						&bcyc_l, &bcyc_s, &bcyc_h,
						&bcys_l, &bcys_s, &bcys_h,
					    &ct_lc, &ct_ls, &ct_hc, &ct_hs,
						&cmx_l, &cmx_h, &cmy_l, &cmy_h);


      if (flag_jac == 1 || flag_jac == -1 ||
		  flag_jac == 2 || flag_jac == -2 ||
		  flag_jac == 3 || flag_jac == -3) goto LABEL1;


    i += 1;

  }
  istflag=0;
#endif

}



/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(xmom_src_1,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *xsource_ptr;
  real source;
  int i=0;				/*Rotor 1*/

  /*Message("In DEFINE_SOURCE - xmom_src_1 \n");*/

  xsource_ptr= Get_Thread_Memory(tc,xsource[i]);
  source=xsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[0]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(ymom_src_1,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *ysource_ptr;
  real source;
  int i=0;				/*Rotor 1*/

  /*Message("In DEFINE_SOURCE - ymom_src_1 \n");*/

  ysource_ptr= Get_Thread_Memory(tc,ysource[i]);
  source=ysource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[1]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(zmom_src_1,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *zsource_ptr;
  real source;
  int i=0;				/*Rotor 1*/

  /*Message("In DEFINE_SOURCE - zmom_src_1 \n");*/

  zsource_ptr= Get_Thread_Memory(tc,zsource[i]);
  source=zsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[2]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}



/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(xmom_src_2,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *xsource_ptr;
  real source;
  int i=1;				/*Rotor 2*/

  /*Message("In DEFINE_SOURCE - xmom_src_2 \n");*/

  xsource_ptr= Get_Thread_Memory(tc,xsource[i]);
  source=xsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[0]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(ymom_src_2,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *ysource_ptr;
  real source;
  int i=1;				/*Rotor 2*/

  /*Message("In DEFINE_SOURCE - ymom_src_2 \n");*/

  ysource_ptr= Get_Thread_Memory(tc,ysource[i]);
  source=ysource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[1]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(zmom_src_2,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *zsource_ptr;
  real source;
  int i=1;				/*Rotor 2*/

  /*Message("In DEFINE_SOURCE - zmom_src_2 \n");*/

  zsource_ptr= Get_Thread_Memory(tc,zsource[i]);
  source=zsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[2]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}



/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(xmom_src_3,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *xsource_ptr;
  real source;
  int i=2;				/*Rotor 3*/

  /*Message("In DEFINE_SOURCE - xmom_src_3 \n");*/

  xsource_ptr= Get_Thread_Memory(tc,xsource[i]);
  source=xsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[0]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(ymom_src_3,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *ysource_ptr;
  real source;
  int i=2;				/*Rotor 3*/

  /*Message("In DEFINE_SOURCE - ymom_src_3 \n");*/

  ysource_ptr= Get_Thread_Memory(tc,ysource[i]);
  source=ysource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[1]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(zmom_src_3,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *zsource_ptr;
  real source;
  int i=2;				/*Rotor 3*/

  /*Message("In DEFINE_SOURCE - zmom_src_3 \n");*/

  zsource_ptr= Get_Thread_Memory(tc,zsource[i]);
  source=zsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[2]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}



/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(xmom_src_4,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *xsource_ptr;
  real source;
  int i=3;				/*Rotor 4*/

  /*Message("In DEFINE_SOURCE - xmom_src_4 \n");*/

  xsource_ptr= Get_Thread_Memory(tc,xsource[i]);
  source=xsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[0]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(ymom_src_4,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *ysource_ptr;
  real source;
  int i=3;				/*Rotor 4*/

  /*Message("In DEFINE_SOURCE - ymom_src_4 \n");*/

  ysource_ptr= Get_Thread_Memory(tc,ysource[i]);
  source=ysource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[1]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(zmom_src_4,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *zsource_ptr;
  real source;
  int i=3;				/*Rotor 4*/

  /*Message("In DEFINE_SOURCE - zmom_src_4 \n");*/

  zsource_ptr= Get_Thread_Memory(tc,zsource[i]);
  source=zsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[2]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}



/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(xmom_src_5,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *xsource_ptr;
  real source;
  int i=4;				/*Rotor 5*/

  /*Message("In DEFINE_SOURCE - xmom_src_5 \n");*/

  xsource_ptr= Get_Thread_Memory(tc,xsource[i]);
  source=xsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[0]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(ymom_src_5,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *ysource_ptr;
  real source;
  int i=4;				/*Rotor 5*/

  /*Message("In DEFINE_SOURCE - ymom_src_5 \n");*/

  ysource_ptr= Get_Thread_Memory(tc,ysource[i]);
  source=ysource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[1]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(zmom_src_5,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *zsource_ptr;
  real source;
  int i=4;				/*Rotor 5*/

  /*Message("In DEFINE_SOURCE - zmom_src_5 \n");*/

  zsource_ptr= Get_Thread_Memory(tc,zsource[i]);
  source=zsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[2]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}



/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(xmom_src_6,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *xsource_ptr;
  real source;
  int i=5;				/*Rotor 6*/

  /*Message("In DEFINE_SOURCE - xmom_src_6 \n");*/

  xsource_ptr= Get_Thread_Memory(tc,xsource[i]);
  source=xsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[0]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(ymom_src_6,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *ysource_ptr;
  real source;
  int i=5;				/*Rotor 6*/

  /*Message("In DEFINE_SOURCE - ymom_src_6 \n");*/

  ysource_ptr= Get_Thread_Memory(tc,ysource[i]);
  source=ysource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[1]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(zmom_src_6,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *zsource_ptr;
  real source;
  int i=5;				/*Rotor 6*/

  /*Message("In DEFINE_SOURCE - zmom_src_6 \n");*/

  zsource_ptr= Get_Thread_Memory(tc,zsource[i]);
  source=zsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[2]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}



/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(xmom_src_7,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *xsource_ptr;
  real source;
  int i=6;				/*Rotor 7*/

  /*Message("In DEFINE_SOURCE - xmom_src_7 \n");*/

  xsource_ptr= Get_Thread_Memory(tc,xsource[i]);
  source=xsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[0]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(ymom_src_7,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *ysource_ptr;
  real source;
  int i=6;				/*Rotor 7*/

  /*Message("In DEFINE_SOURCE - ymom_src_7 \n");*/

  ysource_ptr= Get_Thread_Memory(tc,ysource[i]);
  source=ysource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[1]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(zmom_src_7,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *zsource_ptr;
  real source;
  int i=6;				/*Rotor 7*/

  /*Message("In DEFINE_SOURCE - zmom_src_7 \n");*/

  zsource_ptr= Get_Thread_Memory(tc,zsource[i]);
  source=zsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[2]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}



/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(xmom_src_8,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *xsource_ptr;
  real source;
  int i=7;				/*Rotor 8*/

  /*Message("In DEFINE_SOURCE - xmom_src_8 \n");*/

  xsource_ptr= Get_Thread_Memory(tc,xsource[i]);
  source=xsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[0]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(ymom_src_8,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *ysource_ptr;
  real source;
  int i=7;				/*Rotor 8*/

  /*Message("In DEFINE_SOURCE - ymom_src_8 \n");*/

  ysource_ptr= Get_Thread_Memory(tc,ysource[i]);
  source=ysource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[1]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(zmom_src_8,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *zsource_ptr;
  real source;
  int i=7;				/*Rotor 8*/

  /*Message("In DEFINE_SOURCE - zmom_src_8 \n");*/

  zsource_ptr= Get_Thread_Memory(tc,zsource[i]);
  source=zsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[2]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}



/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(xmom_src_9,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *xsource_ptr;
  real source;
  int i=8;				/*Rotor 9*/

  /*Message("In DEFINE_SOURCE - xmom_src_9 \n");*/

  xsource_ptr= Get_Thread_Memory(tc,xsource[i]);
  source=xsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[0]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(ymom_src_9,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *ysource_ptr;
  real source;
  int i=8;				/*Rotor 9*/

  /*Message("In DEFINE_SOURCE - ymom_src_9 \n");*/

  ysource_ptr= Get_Thread_Memory(tc,ysource[i]);
  source=ysource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[1]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(zmom_src_9,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *zsource_ptr;
  real source;
  int i=8;				/*Rotor 9*/

  /*Message("In DEFINE_SOURCE - zmom_src_9 \n");*/

  zsource_ptr= Get_Thread_Memory(tc,zsource[i]);
  source=zsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[2]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}



/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(xmom_src_10,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *xsource_ptr;
  real source;
  int i=9;				/*Rotor 10*/

  /*Message("In DEFINE_SOURCE - xmom_src_10 \n");*/

  xsource_ptr= Get_Thread_Memory(tc,xsource[i]);
  source=xsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[0]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(ymom_src_10,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *ysource_ptr;
  real source;
  int i=9;				/*Rotor 10*/

  /*Message("In DEFINE_SOURCE - ymom_src_10 \n");*/

  ysource_ptr= Get_Thread_Memory(tc,ysource[i]);
  source=ysource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[1]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}

/*---------------------------------------------------------------------------*/
DEFINE_SOURCE(zmom_src_10,cc,tc,dS,eqn)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
#if !RP_HOST
  real xrc2[ND_ND];
  real *zsource_ptr;
  real source;
  int i=9;				/*Rotor 10*/

  /*Message("In DEFINE_SOURCE - zmom_src_10 \n");*/

  zsource_ptr= Get_Thread_Memory(tc,zsource[i]);
  source=zsource_ptr[cc];

  /*C_CENTROID(xrc2,cc,tc);
  Message("test1 %f \n",xrc2[2]-source);*/
  /*Message("test %f \n",source);*/

  return source;
#endif
}





/*---------------------------------------------------------------------------*/
DEFINE_RW_FILE(reader,fp)
/*---------------------------------------------------------------------------*/
/*				       				       */
/*				       				       */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
  /*real tmp2;*/
  int j, itmp;
#if !RP_NODE
/*host or seriel does this*/

  Message("reading from .dat file\n");

  /*fscanf(fp,"%e\n",&tmp2);
  Message("tmp %e \n",tmp2);*/


  fscanf(fp,"%d\n",&itmp);
  Message("nrtz %d \n",itmp);
  for (j=0; j<itmp; j++)
  {
    fscanf(fp,"%e\n",&bcop[j]);
    Message("Rotor %d bcop %e \n",j+1,bcop[j]*180/M_PI);
    fscanf(fp,"%e\n",&bcyc[j]);
    Message("Rotor %d bcyc %e \n",j+1,bcyc[j]*180/M_PI);
    fscanf(fp,"%e\n",&bcys[j]);
    Message("Rotor %d bcys %e \n",j+1,bcys[j]*180/M_PI);
    fscanf(fp,"%d\n",&trmco[j]);
    Message("Rotor %d trmco %d \n",j+1,trmco[j]);
    fscanf(fp,"%d\n",&trmcy[j]);
    Message("Rotor %d trmcy %d \n",j+1,trmcy[j]);
  }
#endif


/*Only host sends*/
  host_to_node_int_1(itmp);
  host_to_node_real(bcop,10);
  host_to_node_real(bcyc,10);
  host_to_node_real(bcys,10);
  host_to_node_int(trmco,10);
  host_to_node_int(trmcy,10);


#if RP_NODE
  /*Only nodes receive*/
  Message("\n");
  Message("On node %d itmp is %d \n",myid,itmp);
  for (j=0; j<itmp; j++)
  {
    Message("On node %d bcop[%d] is %e \n",myid,j,bcop[j]*180/M_PI);
    Message("On node %d bcyc[%d] is %e \n",myid,j,bcyc[j]*180/M_PI);
	Message("On node %d bcys[%d] is %e \n",myid,j,bcys[j]*180/M_PI);
	Message("On node %d trmco[%d] is %d \n",myid,j,trmco[j]);
	Message("On node %d trmcy[%d] is %d \n",myid,j,trmcy[j]);
  }

#endif



}




/*---------------------------------------------------------------------------*/
DEFINE_RW_FILE(writer,fp)
/*---------------------------------------------------------------------------*/
/*Assumption for parallelization is that all these variables are             */
/*available on ALL nodes (including node0)		       				         */
/*Version	Date	Name			Remarks	       	       */
/*---------------------------------------------------------------------------*/
{
  int j;
  /*real tmp1;*/
  /*nrtz=1;*/

  node_to_host_int_1(nrtz);
  node_to_host_real(bcop,10);
  node_to_host_real(bcyc,10);
  node_to_host_real(bcys,10);
  node_to_host_int(trmco,10);
  node_to_host_int(trmcy,10);


#if !RP_NODE
  Message("writing to .dat file\n");
  /*tmp1=0.123456789;
  fprintf(fp,"%e\n",tmp1);
  Message("tmp %e \n",tmp1);*/

  fprintf(fp,"%d\n",nrtz);
  Message("nrtz %d \n",nrtz);
  for (j=0; j<nrtz; j++)
  {
    fprintf(fp,"%e\n",bcop[j]);
    Message("Rotor %d bcop %e \n",j+1,bcop[j]*180/M_PI);
    fprintf(fp,"%e\n",bcyc[j]);
    Message("Rotor %d bcyc %e \n",j+1,bcyc[j]*180/M_PI);
    fprintf(fp,"%e\n",bcys[j]);
    Message("Rotor %d bcys %e \n",j+1,bcys[j]*180/M_PI);
    fprintf(fp,"%d\n",trmco[j]);
    Message("Rotor %d trmco %d \n",j+1,trmco[j]);
    fprintf(fp,"%d\n",trmcy[j]);
    Message("Rotor %d trmcy %d \n",j+1,trmcy[j]);
  }
#endif
}

