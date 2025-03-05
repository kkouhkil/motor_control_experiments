/* For CPU_ZERO and CPU_SET macros */
//#define _GNU_SOURCE

/*****************************************************************************/
#include <ecrt.h> 
#include <string.h>
#include <stdio.h>
/* For setting the process's priority (setpriority) */
#include <sys/resource.h>
/* For pid_t and getpid() */
#include <unistd.h>
#include <sys/types.h>
/* For locking the program in RAM (mlockall) to prevent swapping */



#include <sys/mman.h>
/* clock_gettime, struct timespec, etc. */
#include <time.h>
/* Header for handling signals (definition of SIGINT) */
#include <signal.h>
/* For using real-time scheduling policy (FIFO) and sched_setaffinity */
#include <sched.h>
/* For using uint32_t format specifier, PRIu32 */
#include <inttypes.h>

#include <iostream>
using namespace std;

int32_t control_mode_pos_vel_trq = 0;

/* The actual and target values of the drive */
int32_t actPos0, targetPos0, desAccumulatedPos0 = 0;
int32_t actPos1, targetPos1, desAccumulatedPos1 = 0;
int32_t actPos2, targetPos2, desAccumulatedPos2 = 0;
int32_t actPos3, targetPos3, desAccumulatedPos3 = 0;
int32_t actPos4, targetPos4, desAccumulatedPos4 = 0;
int32_t actPos5, targetPos5, desAccumulatedPos5 = 0;
int32_t actPos6, targetPos6, desAccumulatedPos6 = 0;

/*****************************************************************************/
/* Comment to disable distributed clocks. */
#define DC

/* One motor revolution increments the encoder by 2^19 -1. */
#define ENCODER_RES 524287

/* The maximum stack size which is guranteed safe to access without faulting. */
#define MAX_SAFE_STACK (8 * 1024)

/* Calculate the time it took to complete the loop. */
#define MEASURE_TIMING

#define SET_CPU_AFFINITY

#define NSEC_PER_SEC (1000000000L)
#define FREQUENCY 1000
/* Period of motion loop, in nanoseconds */
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#ifdef DC

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

#endif

/* SYNC0 event happens halfway through the cycle */
#define SHIFT0 (PERIOD_NS/2)

ec_master_t* master;

void ODwrite(ec_master_t* master, uint16_t slavePos, uint16_t index, uint8_t subIndex, uint8_t objectValue)
{
	/* Blocks until a reponse is received */
	uint8_t retVal = ecrt_master_sdo_download(master, slavePos, index, subIndex, &objectValue, sizeof(objectValue), NULL);
	/* retVal != 0: Failure */
	if (retVal)
		printf("OD write unsuccessful\n");
}

void initDrive(ec_master_t* master, uint16_t slavePos)
{
	if (control_mode_pos_vel_trq == 0){
		/* Mode of operation, CSP */
		// Position mode
		ODwrite(master, slavePos, 0x6060, 0x00, 0x08);  // 0x08 for CSP mode
	}else if (control_mode_pos_vel_trq == 1){
		/* Mode of operation, CSV */
		// Velocity mode
		// ODwrite(master, slavePos, 0x6060, 0x00, 0x09);  // 0x09 for CSV mode
	}else if (control_mode_pos_vel_trq == 2){	
		/* Mode of operation, CST */
		// Torque mode
		// ODwrite(master, slavePos, 0x6060, 0x00, 0x0A);  // 0x0A for CST mode
	}

	/* Reset alarm */
	ODwrite(master, slavePos, 0x6040, 0x00, 0x80);
}

/*****************************************************************************/

/* Add two timespec structures (time1 and time2), store the the result in result. */
/* result = time1 + time2 */
inline void timespec_add(struct timespec* result, struct timespec* time1, struct timespec* time2)
{

	if ((time1->tv_nsec + time2->tv_nsec) >= NSEC_PER_SEC)
	{
		result->tv_sec  = time1->tv_sec + time2->tv_sec + 1;
		result->tv_nsec = time1->tv_nsec + time2->tv_nsec - NSEC_PER_SEC;
	}
	else
	{
		result->tv_sec  = time1->tv_sec + time2->tv_sec;
		result->tv_nsec = time1->tv_nsec + time2->tv_nsec;
	}

}

/* Substract two timespec structures (time1 and time2), store the the result in result.
/* result = time1 - time2 */
inline void timespec_sub(struct timespec* result, struct timespec* time1, struct timespec* time2)
{

	if ((time1->tv_nsec - time2->tv_nsec) < 0)
	{
		result->tv_sec  = time1->tv_sec - time2->tv_sec - 1;
		result->tv_nsec = NSEC_PER_SEC - (time1->tv_nsec - time2->tv_nsec);
	}
	else
	{
		result->tv_sec  = time1->tv_sec - time2->tv_sec;
		result->tv_nsec = time1->tv_nsec - time2->tv_nsec;
	}

}

/*****************************************************************************/

/* We have to pass "master" to ecrt_release_master in signal_handler, but it is not possible
   to define one with more than one argument. Therefore, master should be a global variable.
*/
void signal_handler(int sig)
{
	printf("\nReleasing master...\n");
	ecrt_release_master(master);
	pid_t pid = getpid();
	kill(pid, SIGKILL);
}

/*****************************************************************************/

/* We make sure 8kB (maximum stack size) is allocated and locked by mlockall(MCL_CURRENT | MCL_FUTURE). */
void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

/*****************************************************************************/
#define EC_NEWTIMEVAL2NANO(TV) \
(((TV).tv_sec - 946684800ULL) * 1000000000ULL + (TV).tv_nsec)
uint32_t interval_=(uint32_t)(1000000000.0 / 1000);

// Add state definitions
#define STATE_FAULT              0x0008
#define STATE_SWITCH_ON_DISABLED 0x0040
#define STATE_READY_TO_SWITCH_ON 0x0021
#define STATE_SWITCHED_ON        0x0023
#define STATE_OPERATION_ENABLED  0x0027

// Add function to get drive state
uint16_t getDriveState(uint16_t statusWord) {
    return statusWord & 0x6f; // Mask to get state bits
}

// Add these control word commands
#define CONTROL_WORD_SHUTDOWN          0x0006
#define CONTROL_WORD_SWITCH_ON         0x0007
#define CONTROL_WORD_ENABLE_OPERATION  0x000F
#define CONTROL_WORD_FAULT_RESET       0x0080

int main(int argc, char **argv)
{

	#ifdef SET_CPU_AFFINITY
	cpu_set_t set;
	/* Clear set, so that it contains no CPUs. */
	CPU_ZERO(&set);
	/* Add CPU (core) 1 to the CPU set. */
	CPU_SET(4, &set);
	#endif

	/* 0 for the first argument means set the affinity of the current process. */
	/* Returns 0 on success. */
	if (sched_setaffinity(0, sizeof(set), &set))
	{
		printf("Setting CPU affinity failed!\n");
		return -1;
	}

	/* SCHED_FIFO tasks are allowed to run until they have completed their work or voluntarily yield. */
	/* Note that even the lowest priority realtime thread will be scheduled ahead of any thread with a non-realtime policy;
	   if only one realtime thread exists, the SCHED_FIFO priority value does not matter.
	*/
	struct sched_param param = {};
	param.sched_priority = sched_get_priority_max(SCHED_FIFO);
	printf("Using priority %i.\n", param.sched_priority);
	if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
	{
		perror("sched_setscheduler failed\n");
	}

	/* Lock the program into RAM to prevent page faults and swapping */
	/* MCL_CURRENT: Lock in all current pages.
	   MCL_FUTURE:  Lock in pages for heap and stack and shared memory.
	*/
	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
	{
		printf("mlockall failed\n");
		return -1;
	}

	/* Allocate the entire stack, locked by mlockall(MCL_FUTURE). */
	stack_prefault();
	/* Register the signal handler function. */
	signal(SIGINT, signal_handler);

	/* Reserve the first master (0) (/etc/init.d/ethercat start) for this program */
	master = ecrt_request_master(0);
	if (!master)
		printf("Requesting master failed\n");

	initDrive(master, 0);

	uint16_t alias = 0;

	uint16_t position0 = 0;
	uint16_t position1 = 1;
	uint16_t position2 = 2;
	uint16_t position3 = 3;
	uint16_t position4 = 4;
	uint16_t position5 = 5;
	uint16_t position6 = 6;

	uint32_t vendor_id = 0x5a65726f;
	uint32_t product_code = 0x00029252;

	/* Creates and returns a slave configuration object, ec_slave_config_t*, for the given alias and position. */
	/* Returns NULL (0) in case of error and pointer to the configuration struct otherwise */

	ec_slave_config_t* drive0 = ecrt_master_slave_config(master, alias, position0, vendor_id, product_code);
	ec_slave_config_t* drive1 = ecrt_master_slave_config(master, alias, position1, vendor_id, product_code);
	ec_slave_config_t* drive2 = ecrt_master_slave_config(master, alias, position2, vendor_id, product_code);
	ec_slave_config_t* drive3 = ecrt_master_slave_config(master, alias, position3, vendor_id, product_code);
	ec_slave_config_t* drive4 = ecrt_master_slave_config(master, alias, position4, vendor_id, product_code);
	ec_slave_config_t* drive5 = ecrt_master_slave_config(master, alias, position5, vendor_id, product_code);
	ec_slave_config_t* drive6 = ecrt_master_slave_config(master, alias, position6, vendor_id, product_code);

	ec_slave_config_state_t slaveState0;
	ec_slave_config_state_t slaveState1;
	ec_slave_config_state_t slaveState2;
	ec_slave_config_state_t slaveState3;
	ec_slave_config_state_t slaveState4;
	ec_slave_config_state_t slaveState5;
	ec_slave_config_state_t slaveState6;

	/* If the drive0 = NULL or drive1 = NULL */
	if (!drive0)
	{
		printf("Failed to get slave0 configuration\n");
		return -1;
	}

	if (!drive1)
	{
		printf("Failed to get slave1 configuration\n");
		return -1;
	}

	if (!drive2)
	{
		printf("Failed to get slave2 configuration\n");
		return -1;
	}

	if (!drive3)
	{
		printf("Failed to get slave3 configuration\n");
		return -1;
	}

	if (!drive4)
	{
		printf("Failed to get slave4 configuration\n");
		return -1;
	}

	if (!drive5)
	{
		printf("Failed to get slave5 configuration\n");
		return -1;
	}

	if (!drive6)
	{
		printf("Failed to get slave6 configuration\n");
		return -1;
	}

	/***************************************************/
	/* Slave 0's structures, obtained from $ethercat cstruct -p 0 */
	ec_pdo_entry_info_t slave_0_pdo_entries[] = {
		{0x607a, 0x00, 32}, /* Target Position */
		{0x60fe, 0x00, 32}, /* Digital outputs */
		{0x6040, 0x00, 16}, /* Control Word */
		{0x6064, 0x00, 32}, /* Position Actual Value */
		{0x60fd, 0x00, 32}, /* Digital inputs */
		{0x6041, 0x00, 16}, /* Status Word */
	};
	
	ec_pdo_info_t slave_0_pdos[] = {
		{0x1600, 3, slave_0_pdo_entries + 0}, /* R0PDO */
		{0x1a00, 3, slave_0_pdo_entries + 3}, /* T0PDO */
	};
	
	ec_sync_info_t slave_0_syncs[] = {
		{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
		{3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
		{0xff}
	};

	/* Slave 1's structures, obtained from $ethercat cstruct -p 1 */
	ec_pdo_entry_info_t slave_1_pdo_entries[] = {
		{0x607a, 0x00, 32}, /* Target Position */
		{0x60fe, 0x00, 32}, /* Digital outputs */
		{0x6040, 0x00, 16}, /* Control Word */
		{0x6064, 0x00, 32}, /* Position Actual Value */
		{0x60fd, 0x00, 32}, /* Digital inputs */
		{0x6041, 0x00, 16}, /* Status Word */
	};
	
	ec_pdo_info_t slave_1_pdos[] = {
		{0x1600, 3, slave_1_pdo_entries + 0}, /* R0PDO */
		{0x1a00, 3, slave_1_pdo_entries + 3}, /* T0PDO */
	};
	
	ec_sync_info_t slave_1_syncs[] = {
		{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 1, slave_1_pdos + 0, EC_WD_ENABLE},
		{3, EC_DIR_INPUT, 1, slave_1_pdos + 1, EC_WD_DISABLE},
		{0xff}
	};

	/* Slave 2's structures, obtained from $ethercat cstruct -p 1 */
	ec_pdo_entry_info_t slave_2_pdo_entries[] = {
		{0x607a, 0x00, 32}, /* Target Position */
		{0x60fe, 0x00, 32}, /* Digital outputs */
		{0x6040, 0x00, 16}, /* Control Word */
		{0x6064, 0x00, 32}, /* Position Actual Value */
		{0x60fd, 0x00, 32}, /* Digital inputs */
		{0x6041, 0x00, 16}, /* Status Word */
	};
		
	ec_pdo_info_t slave_2_pdos[] = {
		{0x1600, 3, slave_2_pdo_entries + 0}, /* R0PDO */
		{0x1a00, 3, slave_2_pdo_entries + 3}, /* T0PDO */
	};
		
	ec_sync_info_t slave_2_syncs[] = {
		{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 1, slave_2_pdos + 0, EC_WD_ENABLE},
		{3, EC_DIR_INPUT, 1, slave_2_pdos + 1, EC_WD_DISABLE},
		{0xff}
	};

	/* Slave 3's structures, obtained from $ethercat cstruct -p 1 */
	ec_pdo_entry_info_t slave_3_pdo_entries[] = {
		{0x607a, 0x00, 32}, /* Target Position */
		{0x60fe, 0x00, 32}, /* Digital outputs */
		{0x6040, 0x00, 16}, /* Control Word */
		{0x6064, 0x00, 32}, /* Position Actual Value */
		{0x60fd, 0x00, 32}, /* Digital inputs */
		{0x6041, 0x00, 16}, /* Status Word */
	};
		
	ec_pdo_info_t slave_3_pdos[] = {
			{0x1600, 3, slave_3_pdo_entries + 0}, /* R0PDO */
			{0x1a00, 3, slave_3_pdo_entries + 3}, /* T0PDO */
	};
		
	ec_sync_info_t slave_3_syncs[] = {
		{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 1, slave_3_pdos + 0, EC_WD_ENABLE},
		{3, EC_DIR_INPUT, 1, slave_3_pdos + 1, EC_WD_DISABLE},
		{0xff}
	};

	/* Slave 4's structures, obtained from $ethercat cstruct -p 1 */
	ec_pdo_entry_info_t slave_4_pdo_entries[] = {
		{0x607a, 0x00, 32}, /* Target Position */
		{0x60fe, 0x00, 32}, /* Digital outputs */
		{0x6040, 0x00, 16}, /* Control Word */
		{0x6064, 0x00, 32}, /* Position Actual Value */
		{0x60fd, 0x00, 32}, /* Digital inputs */
		{0x6041, 0x00, 16}, /* Status Word */
	};
		
	ec_pdo_info_t slave_4_pdos[] = {
		{0x1600, 3, slave_4_pdo_entries + 0}, /* R0PDO */
		{0x1a00, 3, slave_4_pdo_entries + 3}, /* T0PDO */
	};
		
	ec_sync_info_t slave_4_syncs[] = {
		{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 1, slave_4_pdos + 0, EC_WD_ENABLE},
		{3, EC_DIR_INPUT, 1, slave_4_pdos + 1, EC_WD_DISABLE},
		{0xff}
	};

	/* Slave 5's structures, obtained from $ethercat cstruct -p 1 */
	ec_pdo_entry_info_t slave_5_pdo_entries[] = {
		{0x607a, 0x00, 32}, /* Target Position */
		{0x60fe, 0x00, 32}, /* Digital outputs */
		{0x6040, 0x00, 16}, /* Control Word */
		{0x6064, 0x00, 32}, /* Position Actual Value */
		{0x60fd, 0x00, 32}, /* Digital inputs */
		{0x6041, 0x00, 16}, /* Status Word */
	};
			
	ec_pdo_info_t slave_5_pdos[] = {
		{0x1600, 3, slave_5_pdo_entries + 0}, /* R0PDO */
		{0x1a00, 3, slave_5_pdo_entries + 3}, /* T0PDO */
	};
			
	ec_sync_info_t slave_5_syncs[] = {
		{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 1, slave_5_pdos + 0, EC_WD_ENABLE},
		{3, EC_DIR_INPUT, 1, slave_5_pdos + 1, EC_WD_DISABLE},
		{0xff}
	};

	/* Slave 6's structures, obtained from $ethercat cstruct -p 1 */
	ec_pdo_entry_info_t slave_6_pdo_entries[] = {
		{0x607a, 0x00, 32}, /* Target Position */
		{0x60fe, 0x00, 32}, /* Digital outputs */
		{0x6040, 0x00, 16}, /* Control Word */
		{0x6064, 0x00, 32}, /* Position Actual Value */
		{0x60fd, 0x00, 32}, /* Digital inputs */
		{0x6041, 0x00, 16}, /* Status Word */
	};
					
	ec_pdo_info_t slave_6_pdos[] = {
		{0x1600, 3, slave_6_pdo_entries + 0}, /* R0PDO */
		{0x1a00, 3, slave_6_pdo_entries + 3}, /* T0PDO */
	};
					
	ec_sync_info_t slave_6_syncs[] = {
		{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
		{1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
		{2, EC_DIR_OUTPUT, 1, slave_6_pdos + 0, EC_WD_ENABLE},
		{3, EC_DIR_INPUT, 1, slave_6_pdos + 1, EC_WD_DISABLE},
		{0xff}
	};

	/***************************************************/

	if (ecrt_slave_config_pdos(drive0, EC_END, slave_0_syncs))
	{
		printf("Failed to configure slave 0 PDOs\n");
		return -1;
	}

	if (ecrt_slave_config_pdos(drive1, EC_END, slave_1_syncs))
	{
		printf("Failed to configure slave 1 PDOs\n");
		return -1;
	}

	if (ecrt_slave_config_pdos(drive2, EC_END, slave_2_syncs))
	{
		printf("Failed to configure slave 2 PDOs\n");
		return -1;
	}

	if (ecrt_slave_config_pdos(drive3, EC_END, slave_3_syncs))
	{
		printf("Failed to configure slave 3 PDOs\n");
		return -1;
	}

	if (ecrt_slave_config_pdos(drive4, EC_END, slave_4_syncs))
	{
		printf("Failed to configure slave 4 PDOs\n");
		return -1;
	}

	if (ecrt_slave_config_pdos(drive5, EC_END, slave_5_syncs))
	{
		printf("Failed to configure slave 5 PDOs\n");
		return -1;
	}

	if (ecrt_slave_config_pdos(drive6, EC_END, slave_6_syncs))
	{
		printf("Failed to configure slave 6 PDOs\n");
		return -1;
	}

	uint controlword0, statusword0 ,
	target_position0,actual_position0,
	target_velocity0,actual_velocity0,
	target_torque0,actual_torque0, digital_output0, digital_input0;

	uint controlword1, statusword1 ,
	target_position1,actual_position1,
	target_velocity1,actual_velocity1,
	target_torque1,actual_torque1, digital_output1, digital_input1;

	uint controlword2, statusword2 ,
	target_position2,actual_position2,
	target_velocity2,actual_velocity2,
	target_torque2,actual_torque2, digital_output2, digital_input2;

	uint controlword3, statusword3 ,
	target_position3,actual_position3,
	target_velocity3,actual_velocity3,
	target_torque3,actual_torque3, digital_output3, digital_input3;

	uint controlword4, statusword4 ,
	target_position4,actual_position4,
	target_velocity4,actual_velocity4,
	target_torque4,actual_torque4, digital_output4, digital_input4;

	uint controlword5, statusword5 ,
	target_position5,actual_position5,
	target_velocity5,actual_velocity5,
	target_torque5,actual_torque5, digital_output5, digital_input5;

	uint controlword6, statusword6 ,
	target_position6,actual_position6,
	target_velocity6,actual_velocity6,
	target_torque6,actual_torque6, digital_output6, digital_input6;

	ec_pdo_entry_reg_t domain1_regs[] =
	{
	// 
	{0, 0, vendor_id, product_code, 0x607a, 0x00, &target_position0	},
	{0, 0, vendor_id, product_code, 0x60fe, 0x00, &digital_output0 	},
	{0, 0, vendor_id, product_code, 0x6040, 0x00, &controlword0    	},
	{0, 0, vendor_id, product_code, 0x6064, 0x00, &actual_position0	},
	{0, 0, vendor_id, product_code, 0x60fd, 0x00, &digital_input0  	},
	{0, 0, vendor_id, product_code, 0x6041, 0x00, &statusword0     	},
	{0, 1, vendor_id, product_code, 0x607a, 0x00, &target_position1	},
	{0, 1, vendor_id, product_code, 0x60fe, 0x00, &digital_output1 	},
	{0, 1, vendor_id, product_code, 0x6040, 0x00, &controlword1    	},
	{0, 1, vendor_id, product_code, 0x6064, 0x00, &actual_position1	},
	{0, 1, vendor_id, product_code, 0x60fd, 0x00, &digital_input1  	},
	{0, 1, vendor_id, product_code, 0x6041, 0x00, &statusword1     	},
	{0, 2, vendor_id, product_code, 0x607a, 0x00, &target_position2	},
	{0, 2, vendor_id, product_code, 0x60fe, 0x00, &digital_output2 	},
	{0, 2, vendor_id, product_code, 0x6040, 0x00, &controlword2    	},
	{0, 2, vendor_id, product_code, 0x6064, 0x00, &actual_position2	},
	{0, 2, vendor_id, product_code, 0x60fd, 0x00, &digital_input2  	},
	{0, 2, vendor_id, product_code, 0x6041, 0x00, &statusword2    	},
	{0, 3, vendor_id, product_code, 0x607a, 0x00, &target_position3	},
	{0, 3, vendor_id, product_code, 0x60fe, 0x00, &digital_output3 	},
	{0, 3, vendor_id, product_code, 0x6040, 0x00, &controlword3    	},
	{0, 3, vendor_id, product_code, 0x6064, 0x00, &actual_position3	},
	{0, 3, vendor_id, product_code, 0x60fd, 0x00, &digital_input3  	},
	{0, 3, vendor_id, product_code, 0x6041, 0x00, &statusword3    	},
	{0, 4, vendor_id, product_code, 0x607a, 0x00, &target_position4	},
	{0, 4, vendor_id, product_code, 0x60fe, 0x00, &digital_output4 	},
	{0, 4, vendor_id, product_code, 0x6040, 0x00, &controlword4    	},
	{0, 4, vendor_id, product_code, 0x6064, 0x00, &actual_position4	},
	{0, 4, vendor_id, product_code, 0x60fd, 0x00, &digital_input4  	},
	{0, 4, vendor_id, product_code, 0x6041, 0x00, &statusword4    	},
	{0, 5, vendor_id, product_code, 0x607a, 0x00, &target_position5	},
	{0, 5, vendor_id, product_code, 0x60fe, 0x00, &digital_output5 	},
	{0, 5, vendor_id, product_code, 0x6040, 0x00, &controlword5    	},
	{0, 5, vendor_id, product_code, 0x6064, 0x00, &actual_position5	},
	{0, 5, vendor_id, product_code, 0x60fd, 0x00, &digital_input5  	},
	{0, 5, vendor_id, product_code, 0x6041, 0x00, &statusword5    	},
	{0, 6, vendor_id, product_code, 0x607a, 0x00, &target_position6	},
	{0, 6, vendor_id, product_code, 0x60fe, 0x00, &digital_output6 	},
	{0, 6, vendor_id, product_code, 0x6040, 0x00, &controlword6    	},
	{0, 6, vendor_id, product_code, 0x6064, 0x00, &actual_position6	},
	{0, 6, vendor_id, product_code, 0x60fd, 0x00, &digital_input6  	},
	{0, 6, vendor_id, product_code, 0x6041, 0x00, &statusword6    	},
	{}
	};
	/* Creates a new process data domain. */
	/* For process data exchange, at least one process data domain is needed. */
	ec_domain_t* domain1 = ecrt_master_create_domain(master);

	//(master, 1);
	if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs))
	{
		printf("PDO entry registration failed\n");
		return -1;
	}

	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	ecrt_master_application_time(master, EC_NEWTIMEVAL2NANO(t));

	ecrt_slave_config_dc(drive0, 0x0300, PERIOD_NS,0, 0, 0);
	ecrt_slave_config_dc(drive1, 0x0300, PERIOD_NS,0, 0, 0);
	ecrt_slave_config_dc(drive2, 0x0300, PERIOD_NS,0, 0, 0);
	ecrt_slave_config_dc(drive3, 0x0300, PERIOD_NS,0, 0, 0);
	ecrt_slave_config_dc(drive4, 0x0300, PERIOD_NS,0, 0, 0);
	ecrt_slave_config_dc(drive5, 0x0300, PERIOD_NS,0, 0, 0);
	ecrt_slave_config_dc(drive6, 0x0300, PERIOD_NS,0, 0, 0);

	/* Initialize master application time. */
	struct timespec masterInitTime;
	clock_gettime(CLOCK_MONOTONIC, &masterInitTime);
	ecrt_master_application_time(master, TIMESPEC2NS(masterInitTime));

	/* Up to this point, we have only requested the master. See log messages */
	printf("Activating master...\n");

	if (ecrt_master_activate(master))
		return -1;

	uint8_t* domain1_pd;
	/* Returns a pointer to (I think) the first byte of PDO data of the domain */
	if (!(domain1_pd = ecrt_domain_data(domain1)))
		return -1;

	struct timespec wakeupTime;

	#ifdef DC
	struct timespec	time;
	#endif

	struct timespec cycleTime = {0, PERIOD_NS};
	clock_gettime(CLOCK_MONOTONIC, &wakeupTime);

	/* The slaves (drives) enter OP mode after exchanging a few frames. */
	/* We exchange frames with no RPDOs (targetPos) untill all slaves have
	   reached OP state, and then we break out of the loop.
	*/
	while (1)
	{

		timespec_add(&wakeupTime, &wakeupTime, &cycleTime);
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);

		ecrt_master_receive(master);

		ecrt_slave_config_state(drive0, &slaveState0);
		ecrt_slave_config_state(drive1, &slaveState1);
		ecrt_slave_config_state(drive2, &slaveState2);
		ecrt_slave_config_state(drive3, &slaveState3);
		ecrt_slave_config_state(drive4, &slaveState4);
		ecrt_slave_config_state(drive5, &slaveState5);
		ecrt_slave_config_state(drive6, &slaveState6);

		if (slaveState0.operational && 
			slaveState1.operational && 
			slaveState2.operational && 
			slaveState3.operational && 
			slaveState4.operational && 
			slaveState5.operational && 
			slaveState6.operational)
		{
			printf("All slaves have reached OP state\n");
			//initDrive(master, 0);
			break;
		}

		ecrt_domain_queue(domain1);

		/* Syncing reference slave to master:
        	1- The master's (PC) clock is the reference.
		   	2- Sync the reference slave's clock to the master's.
		   	3- Sync the other slave clocks to the reference slave's.
		*/

		clock_gettime(CLOCK_MONOTONIC, &time);
		ecrt_master_application_time(master, TIMESPEC2NS(time));
		/* Queues the DC reference clock drift compensation datagram for sending.
		   The reference clock will by synchronized to the **application (PC)** time provided
		   by the last call off ecrt_master_application_time().
		*/
		ecrt_master_sync_reference_clock(master);
		/* Queues the DC clock drift compensation datagram for sending.
		   All slave clocks will be synchronized to the reference slave clock.
		*/
		ecrt_master_sync_slave_clocks(master);

		ecrt_master_send(master);
	}

	/* The slave time received in the current and the previous cycle */
	uint32_t t_cur, t_prev;

	/* Sleep is how long we should sleep each loop to keep the cycle's frequency as close to cycleTime as possible. */
	struct timespec sleepTime;

	struct timespec execTime, endTime;

	/* Wake up 1 msec after the start of the previous loop. */
	sleepTime = cycleTime;

	/* Update wakeupTime = current time */
	clock_gettime(CLOCK_MONOTONIC, &wakeupTime);

	while (1)
	{
		clock_gettime(CLOCK_MONOTONIC, &endTime);
		/* wakeupTime is also start time of the loop. */
		/* execTime = endTime - wakeupTime */
		timespec_sub(&execTime, &endTime, &wakeupTime);
		// printf("Execution time: %lu ns\n", execTime.tv_nsec);

		/* wakeupTime = wakeupTime + sleepTime */
		timespec_add(&wakeupTime, &wakeupTime, &sleepTime);
		/* Sleep to adjust the update frequency */
		/* Note: TIMER_ABSTIME flag is key in ensuring the execution with the desired frequency.
		 *
		 *
		   We don't have to conider the loop's execution time (as long as it doesn't get too close to 1 ms),
		   as the sleep ends cycleTime (=1 msecs) *after the start of the previous loop*.
		*/
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);

		/* Fetches received frames from the newtork device and processes the datagrams. */
		ecrt_master_receive(master);

		/* Evaluates the working counters of the received datagrams and outputs statistics,
		   if necessary.
		   This function is NOT essential to the receive/process/send procedure and can be
		   commented out
		*/
		ecrt_domain_process(domain1);

		ecrt_master_reference_clock_time(master, &t_cur);

		/********************************************************************************/

		/* Read PDOs from the datagram */
		actPos0 = EC_READ_S32(domain1_pd + actual_position0);
		actPos1 = EC_READ_S32(domain1_pd + actual_position1);
		actPos2 = EC_READ_S32(domain1_pd + actual_position2);
		actPos3 = EC_READ_S32(domain1_pd + actual_position3);
		actPos4 = EC_READ_S32(domain1_pd + actual_position4);
		actPos5 = EC_READ_S32(domain1_pd + actual_position5);
		actPos6 = EC_READ_S32(domain1_pd + actual_position6);

		std::cout << "\nactual position[0]: " << actPos0 << std::endl;
		std::cout << "actual position[1]: " << actPos1 << std::endl;
		std::cout << "actual position[2]: " << actPos2 << std::endl;
		std::cout << "actual position[3]: " << actPos3 << std::endl;
		std::cout << "actual position[4]: " << actPos4 << std::endl;
		std::cout << "actual position[5]: " << actPos5 << std::endl;
		std::cout << "actual position[6]: " << actPos6 << std::endl;

		/* Process the received data */
		targetPos0 = actPos0 + desAccumulatedPos0;
		targetPos1 = actPos1 + desAccumulatedPos1;
		targetPos2 = actPos2 + desAccumulatedPos2;
		targetPos3 = actPos3 + desAccumulatedPos3;
		targetPos4 = actPos4 + desAccumulatedPos4;
		targetPos5 = actPos5 + desAccumulatedPos5;
		targetPos6 = actPos6 + desAccumulatedPos6;

		// Read status word
		uint16_t statusWord0 = EC_READ_U16(domain1_pd + statusword0);
		uint16_t state0 = getDriveState(statusWord0);
		uint16_t cw0 = 0;

		uint16_t statusWord1 = EC_READ_U16(domain1_pd + statusword1);
		uint16_t state1 = getDriveState(statusWord1);
		uint16_t cw1 = 0;

		uint16_t statusWord2 = EC_READ_U16(domain1_pd + statusword2);
		uint16_t state2 = getDriveState(statusWord2);
		uint16_t cw2 = 0;

		uint16_t statusWord3 = EC_READ_U16(domain1_pd + statusword3);
		uint16_t state3 = getDriveState(statusWord3);
		uint16_t cw3 = 0;

		uint16_t statusWord4 = EC_READ_U16(domain1_pd + statusword4);
		uint16_t state4 = getDriveState(statusWord4);
		uint16_t cw4 = 0;

		uint16_t statusWord5 = EC_READ_U16(domain1_pd + statusword5);
		uint16_t state5 = getDriveState(statusWord5);
		uint16_t cw5 = 0;

		uint16_t statusWord6 = EC_READ_U16(domain1_pd + statusword6);
		uint16_t state6 = getDriveState(statusWord6);
		uint16_t cw6 = 0;

		std::cout << "state[0]: " << state0 << std::endl;
		std::cout << "state[1]: " << state1 << std::endl;
		std::cout << "state[2]: " << state2 << std::endl;
		std::cout << "state[3]: " << state3 << std::endl;
		std::cout << "state[4]: " << state4 << std::endl;
		std::cout << "state[5]: " << state5 << std::endl;
		std::cout << "state[6]: " << state6 << std::endl;

		// Slave 0
		switch(state0) {
			case STATE_FAULT:
				cw0 = CONTROL_WORD_FAULT_RESET;
				printf("Fault detected! Status word: 0x%04x\n", statusWord0);
				break;
				
			case STATE_SWITCH_ON_DISABLED:
				cw0 = CONTROL_WORD_SHUTDOWN;
				printf("Switch on disabled! Status word: 0x%04x\n", statusWord0);
				break;
				
			case STATE_READY_TO_SWITCH_ON:
				cw0 = CONTROL_WORD_SWITCH_ON;
				printf("Ready to switch on, sending switch on command\n");
				break;
				
			case STATE_SWITCHED_ON:
				cw0 = CONTROL_WORD_ENABLE_OPERATION;
				printf("Switched on, sending enable operation command\n");
				break;
				
			case STATE_OPERATION_ENABLED:

				// Set constant target position
				EC_WRITE_S32(domain1_pd + target_position0, targetPos0);
			
				// Keep operation enabled
				cw0 = CONTROL_WORD_ENABLE_OPERATION;
				
				break;
				
			default:
				cw0 = CONTROL_WORD_SHUTDOWN;
				printf("Unknown state (0x%04x), trying shutdown\n", state0);
				break;
		}

		// Write control word after switch statement
		EC_WRITE_U16(domain1_pd + controlword0, cw0);
		std::cout << "cw0 ==> " << cw0 << std::endl;
		
		// Slave 1
		switch(state1) {
			case STATE_FAULT:
				cw1 = CONTROL_WORD_FAULT_RESET;
				printf("Fault detected! Status word: 0x%04x\n", statusWord1);
				break;
				
			case STATE_SWITCH_ON_DISABLED:
				cw1 = CONTROL_WORD_SHUTDOWN;
				printf("Switch on disabled! Status word: 0x%04x\n", statusWord1);
				break;
				
			case STATE_READY_TO_SWITCH_ON:
				cw1 = CONTROL_WORD_SWITCH_ON;
				printf("Ready to switch on, sending switch on command\n");
				break;
				
			case STATE_SWITCHED_ON:
				cw1 = CONTROL_WORD_ENABLE_OPERATION;
				printf("Switched on, sending enable operation command\n");
				break;
				
			case STATE_OPERATION_ENABLED:

				// Set constant target position
				EC_WRITE_S32(domain1_pd + target_position1, targetPos1);
			
				// Keep operation enabled
				cw1 = CONTROL_WORD_ENABLE_OPERATION;
				
				break;
				
			default:
				cw1 = CONTROL_WORD_SHUTDOWN;
				printf("Unknown state (0x%04x), trying shutdown\n", state1);
				break;
		}

		// Write control word after switch statement
		EC_WRITE_U16(domain1_pd + controlword1, cw1);
		std::cout << "cw1 ==> " << cw1 << std::endl;

		// Slave 2
		switch(state2) {
			case STATE_FAULT:
				cw2 = CONTROL_WORD_FAULT_RESET;
				printf("Fault detected! Status word: 0x%04x\n", statusWord2);
				break;
				
			case STATE_SWITCH_ON_DISABLED:
				cw2 = CONTROL_WORD_SHUTDOWN;
				printf("Switch on disabled! Status word: 0x%04x\n", statusWord2);
				break;
				
			case STATE_READY_TO_SWITCH_ON:
				cw2 = CONTROL_WORD_SWITCH_ON;
				printf("Ready to switch on, sending switch on command\n");
				break;
				
			case STATE_SWITCHED_ON:
				cw2 = CONTROL_WORD_ENABLE_OPERATION;
				printf("Switched on, sending enable operation command\n");
				break;
				
			case STATE_OPERATION_ENABLED:

				// Set constant target position
				EC_WRITE_S32(domain1_pd + target_position2, targetPos2);
			
				// Keep operation enabled
				cw2 = CONTROL_WORD_ENABLE_OPERATION;
				
				break;
				
			default:
				cw2 = CONTROL_WORD_SHUTDOWN;
				printf("Unknown state (0x%04x), trying shutdown\n", state2);
				break;
		}

		// Write control word after switch statement
		EC_WRITE_U16(domain1_pd + controlword2, cw2);
		std::cout << "cw2 ==> " << cw2 << std::endl;

		// Slave 3
		switch(state3) {
			case STATE_FAULT:
				cw3 = CONTROL_WORD_FAULT_RESET;
				printf("Fault detected! Status word: 0x%04x\n", statusWord3);
				break;
				
			case STATE_SWITCH_ON_DISABLED:
				cw3 = CONTROL_WORD_SHUTDOWN;
				printf("Switch on disabled! Status word: 0x%04x\n", statusWord3);
				break;
				
			case STATE_READY_TO_SWITCH_ON:
				cw3 = CONTROL_WORD_SWITCH_ON;
				printf("Ready to switch on, sending switch on command\n");
				break;
				
			case STATE_SWITCHED_ON:
				cw3 = CONTROL_WORD_ENABLE_OPERATION;
				printf("Switched on, sending enable operation command\n");
				break;
				
			case STATE_OPERATION_ENABLED:

				// Set constant target position
				EC_WRITE_S32(domain1_pd + target_position3, targetPos3);
			
				// Keep operation enabled
				cw3 = CONTROL_WORD_ENABLE_OPERATION;
				
				break;
				
			default:
				cw3 = CONTROL_WORD_SHUTDOWN;
				printf("Unknown state (0x%04x), trying shutdown\n", state3);
				break;
		}

		// Write control word after switch statement
		EC_WRITE_U16(domain1_pd + controlword3, cw3);
		std::cout << "cw3 ==> " << cw3 << std::endl;

		// Slave 4
		switch(state4) {
			case STATE_FAULT:
				cw4 = CONTROL_WORD_FAULT_RESET;
				printf("Fault detected! Status word: 0x%04x\n", statusWord4);
				break;
				
			case STATE_SWITCH_ON_DISABLED:
				cw4 = CONTROL_WORD_SHUTDOWN;
				printf("Switch on disabled! Status word: 0x%04x\n", statusWord4);
				break;
				
			case STATE_READY_TO_SWITCH_ON:
				cw4 = CONTROL_WORD_SWITCH_ON;
				printf("Ready to switch on, sending switch on command\n");
				break;
				
			case STATE_SWITCHED_ON:
				cw4 = CONTROL_WORD_ENABLE_OPERATION;
				printf("Switched on, sending enable operation command\n");
				break;
				
			case STATE_OPERATION_ENABLED:

				// Set constant target position
				EC_WRITE_S32(domain1_pd + target_position4, targetPos4);
			
				// Keep operation enabled
				cw4 = CONTROL_WORD_ENABLE_OPERATION;
				
				break;
				
			default:
				cw4 = CONTROL_WORD_SHUTDOWN;
				printf("Unknown state (0x%04x), trying shutdown\n", state4);
				break;
		}

		// Write control word after switch statement
		EC_WRITE_U16(domain1_pd + controlword4, cw4);
		std::cout << "cw4 ==> " << cw4 << std::endl;

		// Slave 5
		switch(state5) {
			case STATE_FAULT:
				cw5 = CONTROL_WORD_FAULT_RESET;
				printf("Fault detected! Status word: 0x%04x\n", statusWord5);
				break;
				
			case STATE_SWITCH_ON_DISABLED:
				cw5 = CONTROL_WORD_SHUTDOWN;
				printf("Switch on disabled! Status word: 0x%04x\n", statusWord5);
				break;
				
			case STATE_READY_TO_SWITCH_ON:
				cw5 = CONTROL_WORD_SWITCH_ON;
				printf("Ready to switch on, sending switch on command\n");
				break;
				
			case STATE_SWITCHED_ON:
				cw5 = CONTROL_WORD_ENABLE_OPERATION;
				printf("Switched on, sending enable operation command\n");
				break;
				
			case STATE_OPERATION_ENABLED:

				// Set constant target position
				EC_WRITE_S32(domain1_pd + target_position5, targetPos5);
			
				// Keep operation enabled
				cw5 = CONTROL_WORD_ENABLE_OPERATION;
				
				break;
				
			default:
				cw5 = CONTROL_WORD_SHUTDOWN;
				printf("Unknown state (0x%04x), trying shutdown\n", state5);
				break;
		}

		// Write control word after switch statement
		EC_WRITE_U16(domain1_pd + controlword5, cw5);
		std::cout << "cw5 ==> " << cw5 << std::endl;

		// Slave 6
		switch(state6) {
			case STATE_FAULT:
				cw6 = CONTROL_WORD_FAULT_RESET;
				printf("Fault detected! Status word: 0x%04x\n", statusWord6);
				break;
				
			case STATE_SWITCH_ON_DISABLED:
				cw6 = CONTROL_WORD_SHUTDOWN;
				printf("Switch on disabled! Status word: 0x%04x\n", statusWord6);
				break;
				
			case STATE_READY_TO_SWITCH_ON:
				cw6 = CONTROL_WORD_SWITCH_ON;
				printf("Ready to switch on, sending switch on command\n");
				break;
				
			case STATE_SWITCHED_ON:
				cw6 = CONTROL_WORD_ENABLE_OPERATION;
				printf("Switched on, sending enable operation command\n");
				break;
				
			case STATE_OPERATION_ENABLED:

				// Set constant target position
				EC_WRITE_S32(domain1_pd + target_position6, targetPos6);
			
				// Keep operation enabled
				cw6 = CONTROL_WORD_ENABLE_OPERATION;
				
				break;
				
			default:
				cw6 = CONTROL_WORD_SHUTDOWN;
				printf("Unknown state (0x%04x), trying shutdown\n", state6);
				break;
		}

		// Write control word after switch statement
		EC_WRITE_U16(domain1_pd + controlword6, cw6);
		std::cout << "cw6 ==> " << cw6 << std::endl;

		/********************************************************************************/

		/* Queues all domain datagrams in the master's datagram queue.
		   Call this function to mark the domain's datagrams for exchanging at the
		   next call of ecrt_master_send()
		*/
		ecrt_domain_queue(domain1);

		/* Distributed clocks */
		clock_gettime(CLOCK_MONOTONIC, &time);
		ecrt_master_application_time(master, TIMESPEC2NS(time));
		ecrt_master_sync_reference_clock(master);
		ecrt_master_sync_slave_clocks(master);
	
		/* Sends all datagrams in the queue.
		   This method takes all datagrams that have been queued for transmission,
		   puts them into frames, and passes them to the Ethernet device for sending.
		*/
		ecrt_master_send(master);

		// printf("\nTimestamp diff: %" PRIu32 " ns\n\n", t_cur - t_prev);
		t_prev = t_cur;

	}

	return 0;
}
