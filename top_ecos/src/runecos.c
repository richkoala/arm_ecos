/*
 * ECOS - Embedded Conic Solver.
 * Copyright (C) 2012-2015 A. Domahidi [domahidi@embotech.com],
 * Automatic Control Lab, ETH Zurich & embotech GmbH, Zurich, Switzerland.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


/* main file with example of how to run ECOS */

#include <stdio.h>
#include <stdlib.h>
#include <glblopts.h>
#ifdef ZCU102_HW_IMP
#include "xil_printf.h"
#include "platform.h"
#include "xparameters.h"
#include "sleep.h"
#include "gpio_led.h"
#include "comm_ps_pl.h"
#include "xbram_control.h"
#endif

#include "ecos.h"
#include "data.h"
#include "splamm.h"


#ifdef ZCU102_HW_IMP
int init_intr_sys(void)
{
	DMA_Init(&AxiDma,DMA_DEV_ID);							//DMA?㏒?谷3?那??‘
	System_Interrupt_Init(&system_interrupt_control); 		//?D???㏒?谷3?那??‘
	Setup_Intr_Exception(&system_interrupt_control);		//那1?邦車2?t?D??

	DMA_Interrupt_Setup(&system_interrupt_control,			//DMA?㏒?谷?D??車??豕??/?D??車3谷?
						&AxiDma,
						DMA_TX_INTR_ID,
						DMA_RX_INTR_ID);

	DMA_Interrupt_Enable(&system_interrupt_control,&AxiDma);
}
#endif

int main(void)
{
#ifdef ZCU102_HW_IMP
	int bram_cnt;
	int bram_value;
	int dma_comm_err_cnt;
#else
	dump_config_int frame_idx=0;
	FILE *fid_log;
	dump_config_int log_idx=0;
	char *file_log_name[50];
#endif

    idxint exitflag = ECOS_FATAL;
	pwork* mywork;
//#if PROFILING > 1 && PRINTLEVEL > 2
    double torder, tkktcreate, ttranspose, tfactor, tkktsolve, ttotal, tsetup, tsolve;
//#endif

//#if PROFILING ==3 && PRINTLEVEL > 0
	//double torder, tkktcreate, ttranspose, tfactor, tkktsolve, ttotal, tsetup, tsolve;
//#endif

#if PROFILING == 3
	int kkt_factor_cnt;
	int ldl_lsolve2_cnt;
	int ldl_dsolve_cnt;
	int ldl_ltsolve_cnt;
	double ldl_lsolve2_time;
	double ldl_dsolve_time;
	double ldl_ltsolve_time;
#endif


#ifdef ZCU102_HW_IMP
	init_platform();
	init_intr_sys();
	BramControl_init(BRAM_DEVICE_ID);
	//dma_comm_err_cnt = DMA_COMM_TEST();
/*
	for (bram_cnt=0;bram_cnt<1024;bram_cnt++)
	{
		XBram_Out32(XPAR_BRAM_CTRL_S_AXI_BASEADDR+bram_cnt*4,bram_cnt);
		bram_value = XBram_In32(XPAR_BRAM_CTRL_S_AXI_BASEADDR+bram_cnt*4);
		if (bram_value!=bram_cnt)
			printf("bram wr_rd error");
	}
*/
	// Cache disable
    //Xil_DCacheDisable();
    //Xil_ICacheDisable();
	gpio_conf();
	XGpioPs_WritePin(&Gpio, 23, LED_OFF);
	sleep(1);
#endif


#ifdef ZCU102_HW_IMP
	XGpioPs_WritePin(&Gpio, 23, LED_ON);
#endif
	mywork = ECOS_setup(n, m, p, l, ncones, q, 0, Gpr, Gjc, Gir, Apr, Ajc, Air, c, h, b);
 
	if( mywork != NULL ){

	/* solve */
		exitflag = ECOS_solve(mywork);

#ifdef ZCU102_HW_IMP    
		XGpioPs_WritePin(&Gpio, 23, LED_OFF);
#else
		dumpDenseMatrix_UD(mywork->x        , n, 1, "./data/dout/result_x.txt");
		dumpDenseMatrix_UD(mywork->y        , p, 1, "./data/dout/result_y.txt");
		dumpDenseMatrix_UD(mywork->z        , m, 1, "./data/dout/result_z.txt");
		dumpDenseMatrix_UD(mywork->s        , m, 1, "./data/dout/result_s.txt");
		dumpDenseMatrix_UD(mywork->lambda   , m, 1, "./data/dout/result_lambda.txt");
		dumpDenseMatrix_UD(&(mywork->kap)   , 1, 1, "./data/dout/result_kap.txt");
		dumpDenseMatrix_UD(&(mywork->tau)   , 1, 1, "./data/dout/result_tau.txt");
#endif
		/* test second solve
		exitflag = ECOS_solve(mywork); */

#if PROFILING > 1 && PRINTLEVEL > 2
		/* some statistics in milliseconds */
		tsolve = mywork->info->tsolve         * 1000;
		tsetup = mywork->info->tsetup         * 1000;
		ttotal = tsetup + tsolve;

		torder = mywork->info->torder         * 1000;
		tkktcreate = mywork->info->tkktcreate * 1000;
		ttranspose = mywork->info->ttranspose * 1000;
		tfactor = mywork->info->tfactor       * 1000;
		tkktsolve = mywork->info->tkktsolve   * 1000;
	
		printf("ECOS timings (all times in milliseconds):\n\n");
		printf("1. Setup: %7.3f (%4.1f%%)\n", tsetup,  tsetup / ttotal*100);
		printf("2. Solve: %7.3f (%4.1f%%)\n", tsolve,  tsolve / ttotal*100);
		printf("----------------------------------\n");
		printf(" Total solve time: %7.3f ms\n\n", ttotal);

		printf("Detailed timings in SETUP:\n");
		printf("Create transposes: %7.3f (%4.1f%%)\n", ttranspose, ttranspose / tsetup*100);
		printf("Create KKT Matrix: %7.3f (%4.1f%%)\n", tkktcreate, tkktcreate / tsetup*100);
		printf(" Compute ordering: %7.3f (%4.1f%%)\n", torder,         torder / tsetup*100);
		printf("            Other: %7.3f (%4.1f%%)\n", tsetup-torder-tkktcreate-ttranspose,         (tsetup-torder-tkktcreate-ttranspose) / tsetup*100);
		printf("\n");

		printf("Detailed timings in SOLVE:\n");
		printf("   Factorizations: %7.3f (%4.1f%% of tsolve / %4.1f%% of ttotal)\n", tfactor,     tfactor / tsolve*100, tfactor / ttotal*100);
		printf("       KKT solves: %7.3f (%4.1f%% of tsolve / %4.1f%% of ttotal)\n", tkktsolve, tkktsolve / tsolve*100, tfactor / ttotal*100);
		printf("            Other: %7.3f (%4.1f%% of tsolve / %4.1f%% of ttotal)\n", tsolve-tkktsolve-tfactor, (tsolve-tkktsolve-tfactor) / tsolve*100, (tsolve-tkktsolve-tfactor) / ttotal*100);
#endif

#if PROFILING == 3
	
		tsolve = mywork->info->tsolve         * 1000;
		tsetup = mywork->info->tsetup         * 1000;
		ttotal = tsetup + tsolve;

		torder = mywork->info->torder         * 1000;
		tkktcreate = mywork->info->tkktcreate * 1000;
		ttranspose = mywork->info->ttranspose * 1000;
		tfactor = mywork->info->tfactor       * 1000;
		tkktsolve = mywork->info->tkktsolve   * 1000;

		printf("ECOS timings (all times in milliseconds):\n\n");
		printf("1. Setup: %7.3f (%4.1f%%)\n", tsetup,  tsetup / ttotal*100);
		printf("2. Solve: %7.3f (%4.1f%%)\n", tsolve,  tsolve / ttotal*100);
		printf("----------------------------------\n");
		printf(" Total solve time: %7.3f ms\n\n", ttotal);

		kkt_factor_cnt	 = mywork->info->kkt_factor_cnt;
		ldl_lsolve2_cnt  = mywork->info->ldl_lsolve2_cnt;
		ldl_dsolve_cnt   = mywork->info->ldl_dsolve_cnt;
		ldl_ltsolve_cnt  = mywork->info->ldl_ltsolve_cnt;

		ldl_lsolve2_time = mywork->info->ldl_lsolve2_time* 1000;
		ldl_dsolve_time  = mywork->info->ldl_dsolve_time * 1000;
		ldl_ltsolve_time = mywork->info->ldl_ltsolve_time* 1000;

		printf("\n");
		printf("   KKT_factor_num : %5d Factorizations Time : %12.3f \n", kkt_factor_cnt,tfactor);
		printf("   ldl_ltsolve_num: %5d ldl_ltsolve_time    : %12.8f \n", ldl_dsolve_cnt,ldl_ltsolve_time);
		printf("   ldl_dsolve_num : %5d ldl_dsolve_time     : %12.8f \n", ldl_dsolve_cnt,ldl_dsolve_time);
		printf("   ldl_lsolve2_num: %5d ldl_lsolve2_time    : %12.8f \n", ldl_lsolve2_cnt,ldl_lsolve2_time);
		printf("\n");
		printf("\n");

		printf("==========================================================\n");
		printf("===========================SUMMARY========================\n");
		printf("==========================================================\n");
		printf("   ALL Process Time         : %12.8f \n", tfactor+ldl_ltsolve_time+ldl_dsolve_time+ldl_lsolve2_time );
		printf("   Factorizations Time      : %12.8f \n", tfactor);
		printf("   Func LDLx=b Time         : %12.8f \n", ldl_ltsolve_time+ldl_dsolve_time+ldl_lsolve2_time );
		printf("   One iter  Process Time   : %12.8f \n", (tfactor+ldl_ltsolve_time+ldl_dsolve_time+ldl_lsolve2_time)/kkt_factor_cnt );
		printf("==========================================================\n");
		printf("==========================================================\n");
		printf("\n");
#endif
    	/* clean up memory */
		ECOS_cleanup(mywork, 0);

  }
    
    /* test version number
    ECOS_ver(ver);
    printf("This test has been run on ECOS version %s\n", ver);
     */
#ifdef ZCU102_HW_IMP
	cleanup_platform();
#endif
    /* explicitly truncate exit code */
	return (int)exitflag;
}
