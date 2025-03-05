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

// 0 --> Position control
// 1 --> Velocity control
// 2 --> Torque control

int32_t control_mode_pos_vel_trq = 0;

/* The actual and target values of the drive */
int32_t actPos0, targetPos0, desAccumulatedPos0 = 0;
int32_t actPos1, targetPos1, desAccumulatedPos1 = 0;
int32_t actPos2, targetPos2, desAccumulatedPos2 = 0;
int32_t actPos3, targetPos3, desAccumulatedPos3 = 0;
int32_t actPos4, targetPos4, desAccumulatedPos4 = 0;
int32_t actPos5, targetPos5, desAccumulatedPos5 = 0;
int32_t actPos6, targetPos6, desAccumulatedPos6 = 0;
// int32_t actVel0, targetVel0 = 0;
// int32_t actTrq0, targetTrq0 = 0;

/*****************************************************************************/
/* Comment to disable PDO configuration (i.e. in case the PDO configuration saved in EEPROM is our
   desired configuration.)
*/
#define CONFIG_PDOS

/* Comment to disable distributed clocks. */
#define DC

/* Choose the syncronization method: The reference clock can be either master's, or the reference slave's (slave 0 by default) */
#ifdef DC

/* Slave0's clock is the reference: no drift. Algorithm from rtai_rtdm_dc example. Work in progress. */
//#define SYNC_MASTER_TO_REF
/* Master's clock (CPU) is the reference: lower overhead. */
#define SYNC_REF_TO_MASTER

#endif

#ifdef DC

/* Comment to disable configuring slave's DC specification (shift time & cycle time) */
#define CONFIG_DC

#endif

/*****************************************************************************/

/* One motor revolution increments the encoder by 2^19 -1. */
#define ENCODER_RES 524287
/* The maximum stack size which is guranteed safe to access without faulting. */
#define MAX_SAFE_STACK (8 * 1024)

/* Uncomment to enable performance measurement. */
/* Measure the difference in reference slave's clock timstamp each cycle, and print the result,
   which should be as close to cycleTime as possible. */
/* Note: Only works with DC enabled. */
#define MEASURE_PERF

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

#ifdef CONFIG_DC

/* SYNC0 event happens halfway through the cycle */
#define SHIFT0 (PERIOD_NS/2)

#endif

/*****************************************************************************/
/* Note: Anything relying on definition of SYNC_MASTER_TO_REF is essentially copy-pasted from /rtdm_rtai_dc/main.c */

#ifdef SYNC_MASTER_TO_REF

/* First used in system_time_ns() */
static int64_t  system_time_base = 0LL;
/* First used in sync_distributed_clocks() */
static uint64_t dc_time_ns = 0;
static int32_t  prev_dc_diff_ns = 0;
/* First used in update_master_clock() */
static int32_t  dc_diff_ns = 0;
static unsigned int cycle_ns = PERIOD_NS;
static uint8_t  dc_started = 0;
static int64_t  dc_diff_total_ns = 0LL;
static int64_t  dc_delta_total_ns = 0LL;
static int      dc_filter_idx = 0;
static int64_t  dc_adjust_ns;
#define DC_FILTER_CNT          1024
/** Return the sign of a number
 *
 * ie -1 for -ve value, 0 for 0, +1 for +ve value
 *
 * \retval the sign of the value
 */
#define sign(val) \
    ({ typeof (val) _val = (val); \
    ((_val > 0) - (_val < 0)); })

static uint64_t dc_start_time_ns = 0LL;

#endif

ec_master_t* master;

/*****************************************************************************/

#ifdef SYNC_MASTER_TO_REF

/** Get the time in ns for the current cpu, adjusted by system_time_base.
 *
 * \attention Rather than calling rt_get_time_ns() directly, all application
 * time calls should use this method instead.
 *
 * \ret The time in ns.
 */
uint64_t system_time_ns(void)
{
	struct timespec time;
	int64_t time_ns;
	clock_gettime(CLOCK_MONOTONIC, &time);
	time_ns = TIMESPEC2NS(time);

	if (system_time_base > time_ns)
	{
		printf("%s() error: system_time_base greater than"
		       " system time (system_time_base: %ld, time: %lu\n",
			__func__, system_time_base, time_ns);
		return time_ns;
	}
	else
	{
		return time_ns - system_time_base;
	}
}


/** Synchronise the distributed clocks
 */
void sync_distributed_clocks(void)
{

	uint32_t ref_time = 0;
	uint64_t prev_app_time = dc_time_ns;

	dc_time_ns = system_time_ns();

	// set master time in nano-seconds
	ecrt_master_application_time(master, dc_time_ns);

	// get reference clock time to synchronize master cycle
	ecrt_master_reference_clock_time(master, &ref_time);
	dc_diff_ns = (uint32_t) prev_app_time - ref_time;

	// call to sync slaves to ref slave
	ecrt_master_sync_slave_clocks(master);
}


/** Update the master time based on ref slaves time diff
 *
 * called after the ethercat frame is sent to avoid time jitter in
 * sync_distributed_clocks()
 */
void update_master_clock(void)
{

	// calc drift (via un-normalised time diff)
	int32_t delta = dc_diff_ns - prev_dc_diff_ns;
	//printf("%d\n", (int) delta);
	prev_dc_diff_ns = dc_diff_ns;

	// normalise the time diff
	dc_diff_ns = ((dc_diff_ns + (cycle_ns / 2)) % cycle_ns) - (cycle_ns / 2);

	// only update if primary master
	if (dc_started)
	{

		// add to totals
		dc_diff_total_ns += dc_diff_ns;
		dc_delta_total_ns += delta;
		dc_filter_idx++;

		if (dc_filter_idx >= DC_FILTER_CNT)
		{
			// add rounded delta average
			dc_adjust_ns += ((dc_delta_total_ns + (DC_FILTER_CNT / 2)) / DC_FILTER_CNT);

			// and add adjustment for general diff (to pull in drift)
			dc_adjust_ns += sign(dc_diff_total_ns / DC_FILTER_CNT);

			// limit crazy numbers (0.1% of std cycle time)
			if (dc_adjust_ns < -1000)
			{
				dc_adjust_ns = -1000;
			}
			if (dc_adjust_ns > 1000)
			{
				dc_adjust_ns =  1000;
			}

			// reset
			dc_diff_total_ns = 0LL;
			dc_delta_total_ns = 0LL;
			dc_filter_idx = 0;
		}

		// add cycles adjustment to time base (including a spot adjustment)
		system_time_base += dc_adjust_ns + sign(dc_diff_ns);
	}
	else
	{
		dc_started = (dc_diff_ns != 0);

		if (dc_started)
		{
			// output first diff
			printf("First master diff: %d.\n", dc_diff_ns);

			// record the time of this initial cycle
			dc_start_time_ns = dc_time_ns;
		}
	}
}

#endif

/*****************************************************************************/

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

#ifdef MEASURE_TIMING
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
#endif

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

	#ifdef CONFIG_PDOS
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

	#endif

	uint controlword, statusword ,
	target_position,actual_position,
	target_velocity,actual_velocity,
	target_torque,actual_torque, digital_output, digital_input;

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
	{0, 0, vendor_id, product_code, 0x607a, 0x00, &target_position     	},
	{0, 0, vendor_id, product_code, 0x60fe, 0x00, &digital_output      	},
	{0, 0, vendor_id, product_code, 0x6040, 0x00, &controlword         	},
	{0, 0, vendor_id, product_code, 0x6064, 0x00, &actual_position     	},
	{0, 0, vendor_id, product_code, 0x60fd, 0x00, &digital_input       	},
	{0, 0, vendor_id, product_code, 0x6041, 0x00, &statusword          	},
	{0, 1, vendor_id, product_code, 0x607a, 0x00, &target_position1     },
	{0, 1, vendor_id, product_code, 0x60fe, 0x00, &digital_output1      },
	{0, 1, vendor_id, product_code, 0x6040, 0x00, &controlword1         },
	{0, 1, vendor_id, product_code, 0x6064, 0x00, &actual_position1     },
	{0, 1, vendor_id, product_code, 0x60fd, 0x00, &digital_input1       },
	{0, 1, vendor_id, product_code, 0x6041, 0x00, &statusword1          },
	{0, 2, vendor_id, product_code, 0x607a, 0x00, &target_position2     },
	{0, 2, vendor_id, product_code, 0x60fe, 0x00, &digital_output2      },
	{0, 2, vendor_id, product_code, 0x6040, 0x00, &controlword2         },
	{0, 2, vendor_id, product_code, 0x6064, 0x00, &actual_position2     },
	{0, 2, vendor_id, product_code, 0x60fd, 0x00, &digital_input2       },
	{0, 2, vendor_id, product_code, 0x6041, 0x00, &statusword2        	},
	{0, 3, vendor_id, product_code, 0x607a, 0x00, &target_position3     },
	{0, 3, vendor_id, product_code, 0x60fe, 0x00, &digital_output3      },
	{0, 3, vendor_id, product_code, 0x6040, 0x00, &controlword3         },
	{0, 3, vendor_id, product_code, 0x6064, 0x00, &actual_position3     },
	{0, 3, vendor_id, product_code, 0x60fd, 0x00, &digital_input3       },
	{0, 3, vendor_id, product_code, 0x6041, 0x00, &statusword3        	},
	{0, 4, vendor_id, product_code, 0x607a, 0x00, &target_position4     },
	{0, 4, vendor_id, product_code, 0x60fe, 0x00, &digital_output4      },
	{0, 4, vendor_id, product_code, 0x6040, 0x00, &controlword4         },
	{0, 4, vendor_id, product_code, 0x6064, 0x00, &actual_position4     },
	{0, 4, vendor_id, product_code, 0x60fd, 0x00, &digital_input4       },
	{0, 4, vendor_id, product_code, 0x6041, 0x00, &statusword4        	},
	{0, 5, vendor_id, product_code, 0x607a, 0x00, &target_position5     },
	{0, 5, vendor_id, product_code, 0x60fe, 0x00, &digital_output5      },
	{0, 5, vendor_id, product_code, 0x6040, 0x00, &controlword5         },
	{0, 5, vendor_id, product_code, 0x6064, 0x00, &actual_position5     },
	{0, 5, vendor_id, product_code, 0x60fd, 0x00, &digital_input5       },
	{0, 5, vendor_id, product_code, 0x6041, 0x00, &statusword5        	},
	{0, 6, vendor_id, product_code, 0x607a, 0x00, &target_position6     },
	{0, 6, vendor_id, product_code, 0x60fe, 0x00, &digital_output6      },
	{0, 6, vendor_id, product_code, 0x6040, 0x00, &controlword6         },
	{0, 6, vendor_id, product_code, 0x6064, 0x00, &actual_position6     },
	{0, 6, vendor_id, product_code, 0x60fd, 0x00, &digital_input6       },
	{0, 6, vendor_id, product_code, 0x6041, 0x00, &statusword6        	},
	{}
	};
	/* Creates a new process data domain. */
	/* For process data exchange, at least one process data domain is needed. */
	ec_domain_t* domain1 = ecrt_master_create_domain(master);

	/* Registers PDOs for a domain. */
	/* Returns 0 on success. */
	printf("Activating master...\n");


	//(master, 1);
	if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs))
	{
		printf("PDO entry registration failed\n");
		return -1;
	}

	#ifdef CONFIG_DC

	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	ecrt_master_application_time(master, EC_NEWTIMEVAL2NANO(t));
	/* Do not enable Sync1 */
	ecrt_slave_config_dc(drive0, 0x0300, PERIOD_NS,0, 0, 0);
	ecrt_slave_config_dc(drive1, 0x0300, PERIOD_NS,0, 0, 0);
	ecrt_slave_config_dc(drive2, 0x0300, PERIOD_NS,0, 0, 0);
	ecrt_slave_config_dc(drive3, 0x0300, PERIOD_NS,0, 0, 0);
	ecrt_slave_config_dc(drive4, 0x0300, PERIOD_NS,0, 0, 0);
	ecrt_slave_config_dc(drive5, 0x0300, PERIOD_NS,0, 0, 0);
	ecrt_slave_config_dc(drive6, 0x0300, PERIOD_NS,0, 0, 0);

	#endif

	#ifdef SYNC_REF_TO_MASTER
	/* Initialize master application time. */
	struct timespec masterInitTime;
	clock_gettime(CLOCK_MONOTONIC, &masterInitTime);
	ecrt_master_application_time(master, TIMESPEC2NS(masterInitTime));
	#endif

	#ifdef SYNC_MASTER_TO_REF
	/* Initialize master application time. */
	dc_start_time_ns = system_time_ns();
	dc_time_ns = dc_start_time_ns;
	ecrt_master_application_time(master, dc_start_time_ns);

	if (ecrt_master_select_reference_clock(master, drive0))
	{
		printf("Selecting slave 0 as reference clock failed!\n");
		return -1;
	}
	#endif

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

		if (slaveState0.operational)
		{
			printf("All slaves have reached OP state\n");
			//initDrive(master, 0);
			break;
		}

		ecrt_slave_config_state(drive1, &slaveState1);

		if (slaveState1.operational)
		{
			printf("All slaves have reached OP state\n");
			//initDrive(master, 0);
			break;
		}

		ecrt_slave_config_state(drive2, &slaveState2);

		if (slaveState2.operational)
		{
			printf("All slaves have reached OP state\n");
			//initDrive(master, 0);
			break;
		}

		ecrt_slave_config_state(drive3, &slaveState3);

		if (slaveState3.operational)
		{
			printf("All slaves have reached OP state\n");
			//initDrive(master, 0);
			break;
		}

		ecrt_slave_config_state(drive4, &slaveState4);

		if (slaveState4.operational)
		{
			printf("All slaves have reached OP state\n");
			//initDrive(master, 0);
			break;
		}

		ecrt_slave_config_state(drive5, &slaveState5);

		if (slaveState5.operational)
		{
			printf("All slaves have reached OP state\n");
			//initDrive(master, 0);
			break;
		}

		ecrt_slave_config_state(drive6, &slaveState6);

		if (slaveState6.operational)
		{
			printf("All slaves have reached OP state\n");
			//initDrive(master, 0);
			break;
		}

		ecrt_domain_queue(domain1);

		#ifdef SYNC_REF_TO_MASTER
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
		#endif

		#ifdef SYNC_MASTER_TO_REF
		// sync distributed clock just before master_send to set
     	        // most accurate master clock time
                sync_distributed_clocks();
		#endif

		ecrt_master_send(master);

		#ifdef SYNC_MASTER_TO_REF
		// update the master clock
     		// Note: called after ecrt_master_send() to reduce time
                // jitter in the sync_distributed_clocks() call
                update_master_clock();
		#endif

	}

	#ifdef MEASURE_PERF
	/* The slave time received in the current and the previous cycle */
	uint32_t t_cur, t_prev;
	#endif

	/* Sleep is how long we should sleep each loop to keep the cycle's frequency as close to cycleTime as possible. */
	struct timespec sleepTime;
	#ifdef MEASURE_TIMING
	struct timespec execTime, endTime;
	#endif

	/* Wake up 1 msec after the start of the previous loop. */
	sleepTime = cycleTime;
	/* Update wakeupTime = current time */
	clock_gettime(CLOCK_MONOTONIC, &wakeupTime);


	while (1)
	{
		#ifdef MEASURE_TIMING
		clock_gettime(CLOCK_MONOTONIC, &endTime);
		/* wakeupTime is also start time of the loop. */
		/* execTime = endTime - wakeupTime */
		timespec_sub(&execTime, &endTime, &wakeupTime);
		// printf("Execution time: %lu ns\n", execTime.tv_nsec);
		#endif

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

		#ifdef MEASURE_PERF
		ecrt_master_reference_clock_time(master, &t_cur);
		#endif

		/********************************************************************************/

		/* Read PDOs from the datagram */
		actPos0 = EC_READ_S32(domain1_pd + actual_position);
		actPos1 = EC_READ_S32(domain1_pd + actual_position1);
		actPos2 = EC_READ_S32(domain1_pd + actual_position2);
		actPos3 = EC_READ_S32(domain1_pd + actual_position3);
		actPos4 = EC_READ_S32(domain1_pd + actual_position4);
		actPos5 = EC_READ_S32(domain1_pd + actual_position5);
		actPos6 = EC_READ_S32(domain1_pd + actual_position6);
		// actVel0 = EC_READ_S32(domain1_pd + actual_velocity);
		// actTrq0 = EC_READ_S32(domain1_pd + actual_torque);

		std::cout << "\nactual position[0]: " << actPos0 << std::endl;
		std::cout << "actual position[1]: " << actPos1 << std::endl;
		std::cout << "actual position[2]: " << actPos2 << std::endl;
		std::cout << "actual position[3]: " << actPos3 << std::endl;
		std::cout << "actual position[4]: " << actPos4 << std::endl;
		std::cout << "actual position[5]: " << actPos5 << std::endl;
		std::cout << "actual position[6]: " << actPos6 << std::endl;
		// std::cout << "Actual velocity: " << actVel0 << std::endl;
		// std::cout << "Actual torque: " << actTrq0 << std::endl;

		/* Process the received data */
		targetPos0 = actPos0 + desAccumulatedPos0;
		// targetVel0 = actVel0 + desAccumulatedVel0;
		// targetTrq0 = actTrq0 + desAccumulatedTrq0;

		// Read status word
		uint16_t statusWord = EC_READ_U16(domain1_pd + statusword);
		uint16_t state = getDriveState(statusWord);
		uint16_t cw = 0; // 将变量声明移到switch语句之前
		
		// State machine for enabling the drive

		// printf("Current state: %x\n ", state);
		switch(state) {
			case STATE_FAULT:
				cw = CONTROL_WORD_FAULT_RESET;
				printf("Fault detected! Status word: 0x%04x\n", statusWord);
				break;
				
			case STATE_SWITCH_ON_DISABLED:
				cw = CONTROL_WORD_SHUTDOWN;
				printf("Switch on disabled! Status word: 0x%04x\n", statusWord);
				break;
				
			case STATE_READY_TO_SWITCH_ON:
				cw = CONTROL_WORD_SWITCH_ON;
				printf("Ready to switch on, sending switch on command\n");
				break;
				
			case STATE_SWITCHED_ON:
				cw = CONTROL_WORD_ENABLE_OPERATION;
				printf("Switched on, sending enable operation command\n");
				break;
				
			case STATE_OPERATION_ENABLED:

				// Set constant target position
				if (control_mode_pos_vel_trq == 0){

					EC_WRITE_S32(domain1_pd + target_position, targetPos0);
				}else if (control_mode_pos_vel_trq == 1){

					// Set constant target velocity
					// EC_WRITE_S32(domain1_pd + target_velocity, targetVel0);
				}else if (control_mode_pos_vel_trq == 2){
				
					// printf("Target torque %d\n", targetTrq0);
					// Set constant target torque
					// EC_WRITE_S32(domain1_pd + target_torque, targetTrq0);
				}
								
				// Keep operation enabled
				cw = CONTROL_WORD_ENABLE_OPERATION;
				
				break;
				
			default:
				cw = CONTROL_WORD_SHUTDOWN;
				printf("Unknown state (0x%04x), trying shutdown\n", state);
				break;
		}

		// Write control word after switch statement
		// EC_WRITE_U16(domain1_pd + controlword, cw);

		/********************************************************************************/

		/* Queues all domain datagrams in the master's datagram queue.
		   Call this function to mark the domain's datagrams for exchanging at the
		   next call of ecrt_master_send()
		*/
		ecrt_domain_queue(domain1);

		#ifdef SYNC_REF_TO_MASTER
		/* Distributed clocks */
		clock_gettime(CLOCK_MONOTONIC, &time);
		ecrt_master_application_time(master, TIMESPEC2NS(time));
		ecrt_master_sync_reference_clock(master);
		ecrt_master_sync_slave_clocks(master);
		#endif

		#ifdef SYNC_MASTER_TO_REF
		// sync distributed clock just before master_send to set
     	        // most accurate master clock time
                sync_distributed_clocks();
		#endif

		/* Sends all datagrams in the queue.
		   This method takes all datagrams that have been queued for transmission,
		   puts them into frames, and passes them to the Ethernet device for sending.
		*/
		ecrt_master_send(master);

		#ifdef SYNC_MASTER_TO_REF
		// update the master clock
     		// Note: called after ecrt_master_send() to reduce time
                // jitter in the sync_distributed_clocks() call
                update_master_clock();
		#endif

		#ifdef MEASURE_PERF
		// printf("\nTimestamp diff: %" PRIu32 " ns\n\n", t_cur - t_prev);
		t_prev = t_cur;
		#endif

	}

	return 0;
}
