#include <stdio.h>
#include <stdlib.h>
#include "utlist.h"
#include "utils.h"
#include "memory_controller.h"
#include "params.h"
#define MARKINGCAP 100
#define HI_WM (WQ_CAPACITY - NUM_BANKS)
#define LO_WM (HI_WM - 8)
#define MIN_WRITES_ONCE_WRITING_HAS_BEGUN 1
#define MAX_DISTANCE 13
#define M2C_INTERVAL 970
#define C2C_INTERVAL 220
#define MAX_ROWS 32768
long int count_col_hits[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];

long int CAPN;

typedef struct state {
	int marked;
	int incoming; 
} State; 

extern int WQ_CAPACITY;
extern int NUM_BANKS;
extern int NUM_RANKS;
extern long long int CYCLE_VAL;
extern int NUMCORES;
int drain_writes[MAX_NUM_CHANNELS];
int writes_done_this_drain[MAX_NUM_CHANNELS];
int draining_writes_due_to_rq_empty[MAX_NUM_CHANNELS];
int distance [];
int interval [];
int phase [];

int *load_bank, *load_max, *load_all, marked_num;
void
init_scheduler_vars1 ()
{
  for (int i = 0; i < MAX_NUM_CHANNELS; i++)
  {
    for (int j = 0; j < MAX_NUM_RANKS; j++)
    {
      for (int k = 0; k < MAX_NUM_BANKS; k++)
      {
        count_col_hits[i][j][k] = 0;
      }
    }
  }

  return;
}

// row buffer locality between thread
int *localityCounter;

// Cleaning the loading
void init_all_banks(){
	int bank_count = NUMCORES * MAX_NUM_CHANNELS * MAX_NUM_RANKS * MAX_NUM_BANKS;
	for(int index = 0; index < bank_count; index++)
		load_bank[index] = 0;
}

void init_distance_interval() {

	for (int i = 0; i < NUMCORES; i++) {
		distance[i] = 0;
		interval[i] = 0;
		phase[i] = 0;
	}
}
/* A data structure to see if a bank is a candidate for precharge. */
int recent_colacc[MAX_NUM_CHANNELS][MAX_NUM_RANKS][MAX_NUM_BANKS];

void init_scheduler_vars()
{
	// initialize all scheduler variables here
	int bank_count = NUMCORES * MAX_NUM_CHANNELS * MAX_NUM_RANKS * MAX_NUM_BANKS;
	load_bank = (int*)malloc( bank_count * sizeof(int) ); 
	load_max = (int*)malloc( NUMCORES * sizeof(int));
	load_all = (int*)malloc( NUMCORES * sizeof(int));
	int distance1 = (int*)malloc( NUMCORES * sizeof(int));
	int interval1 = (int*)malloc( NUMCORES * sizeof(int));
	int phase1 = (int*)malloc( NUMCORES * sizeof(int));
	localityCounter = (int*)malloc( MAX_NUM_RANKS * MAX_NUM_BANKS * MAX_ROWS * sizeof(int));
	init_all_banks();
	init_distance_interval();
	marked_num = 0;
	// Aggressive precharge
	int i, j, k;
	for (i=0; i<MAX_NUM_CHANNELS; i++) {
		for (j=0; j<MAX_NUM_RANKS; j++) {
			for (k=0; k<MAX_NUM_BANKS; k++) {
				recent_colacc[i][j][k] = 0;
			}
		}
	}
	return;
}

void updateAggressiveTable(int channel, request_t * req) {
	/* Before issuing the command, see if this bank is now a candidate for closure (if it just did a column-rd/wr).
	   If the bank just did an activate or precharge, it is not a candidate for closure. */
	if (req->next_command == COL_READ_CMD) {
		recent_colacc[channel][req->dram_addr.rank][req->dram_addr.bank] = 1;
	}
	if (req->next_command == ACT_CMD) {
		recent_colacc[channel][req->dram_addr.rank][req->dram_addr.bank] = 0;
	}
	if (req->next_command == PRE_CMD) {
		recent_colacc[channel][req->dram_addr.rank][req->dram_addr.bank] = 0;
	}
}

void aggressivePrecharge(int channel) {
	/* If a command hasn't yet been issued to this channel in this cycle, issue a precharge. */
	if (!command_issued_current_cycle[channel]) {
		for (int i=0; i<NUM_RANKS; i++) {
			for (int j=0; j<NUM_BANKS; j++) {  /* For all banks on the channel.. */
        		int chit = dram_state[channel][i][j].state == ROW_ACTIVE;
				if (chit && recent_colacc[channel][i][j]) {  /* See if this bank is a candidate. */
					if (is_precharge_allowed(channel,i,j)) {  /* See if precharge is doable. */
						if (issue_precharge_command(channel,i,j)) {
							recent_colacc[channel][i][j] = 0;
						}
					}
				}
			}
		}
	}
}
int higher(request_t *req_a, request_t *req_b){
	if( req_b == NULL ) return 1;                                                       // req_b = NULL -> always return req_a
	State * s_a = (State*)(req_a->user_ptr);
	State * s_b = (State*)(req_b->user_ptr);
	if (((s_a->marked)) && !(s_b->marked)) return 1;                              // if req_a marked and req_b didn't then return req_a
	int req_a_hit = req_a->command_issuable && (req_a->next_command == COL_READ_CMD), 
		req_b_hit = req_b->command_issuable && (req_b->next_command == COL_READ_CMD);   // row hit status
	int req_a_core  = req_a->thread_id, 
		req_b_core  = req_b->thread_id;
	if (!(s_a->marked)){
		if (s_b->marked) return 0;                                              
		else 
			if (req_b_hit)      return 0;                                               // hit   
			else if (req_a_hit) return 1;
			else                {
				if (phase[req_a_core] && !phase[req_b_core]) return 1;
				if (!phase[req_a_core] && phase[req_b_core]) return 0;
				if (load_max[req_a_core] < load_max[req_b_core]) return 1;                          // all and max load ranking
				else if (load_max[req_a_core] > load_max[req_b_core]) return 0;
				else if (load_all[req_a_core] < load_all[req_b_core]) return 1;
				return 0;
			};
	}
	if (req_a_hit && !req_b_hit) return 1;
	if (!req_a_hit && req_b_hit) return 0;
	if (phase[req_a_core] && !phase[req_b_core]) return 1;
	if (!phase[req_a_core] && phase[req_b_core]) return 0;
	if (load_max[req_a_core] < load_max[req_b_core]) return 1;                          // all and max load ranking
	else if (load_max[req_a_core] > load_max[req_b_core]) return 0;
	else if (load_all[req_a_core] < load_all[req_b_core]) return 1;
	return 0;
}

void stateAssign(int channel) {
	request_t * rd_ptr = NULL;
	request_t * wr_ptr = NULL;
	LL_FOREACH(write_queue_head[channel], wr_ptr)
	{
		if (wr_ptr->user_ptr == NULL) {
			State * st = (State*) malloc (sizeof(State));
			st->marked = 0;
			st->incoming = 1;
			wr_ptr->user_ptr = st;
		}
	}
	LL_FOREACH(read_queue_head[channel], rd_ptr)
	{
		if (rd_ptr->user_ptr == NULL) {
			State * st = (State*) malloc (sizeof(State));
			st->marked = 0;
			st->incoming = 1;
			rd_ptr->user_ptr = st;
		}
	}
}
void schedule(int channel)
{


	stateAssign(channel);
	//predictThreadPhase(channel);
	request_t * rd_ptr = NULL;
	request_t * wr_ptr = NULL;
	request_t * auto_ptr = NULL;
	int read_issued = 0;
	int write_issued = 0;

	// begin write drain if we're above the high water mark
	if((write_queue_length[channel] > HI_WM) && (!drain_writes[channel]))
	{
		drain_writes[channel] = 1;
		writes_done_this_drain[channel] = 0;
	}

	// also begin write drain if read queue is empty
	if((read_queue_length[channel] < 1) && (write_queue_length[channel] > 0) && (!drain_writes[channel]))
	{
		drain_writes[channel] = 1;
		writes_done_this_drain[channel] = 0;
		draining_writes_due_to_rq_empty[channel] = 1;
	}

	// end write drain if we're below the low water mark
	if((drain_writes[channel]) && (write_queue_length[channel] <= LO_WM) && (!draining_writes_due_to_rq_empty[channel]))
	{
		drain_writes[channel] = 0;
	}

	// end write drain that was due to read_queue emptiness only if at least one write has completed
	if((drain_writes[channel]) && (read_queue_length[channel] > 0) && (draining_writes_due_to_rq_empty[channel]) && (writes_done_this_drain[channel] > MIN_WRITES_ONCE_WRITING_HAS_BEGUN))
	{
		drain_writes[channel] = 0;
		draining_writes_due_to_rq_empty[channel] = 0;
	}

	// make sure we don't try to drain writes if there aren't any
	if(write_queue_length[channel] == 0)
	{
		drain_writes[channel] = 0;
	}

	// drain from write queue now
	if(drain_writes[channel])
	{
		// prioritize open row hits
		LL_FOREACH(write_queue_head[channel], wr_ptr)
		{
			if(wr_ptr->command_issuable && (wr_ptr->next_command == COL_WRITE_CMD))
			{
				writes_done_this_drain[channel]++;
updateAggressiveTable(channel, wr_ptr);
				issue_request_command(wr_ptr);
				write_issued = 1;
				break;
			}
		}
		request_t * issue = NULL;
		if(!write_issued){
			// if no write row hit, check read queue
			LL_FOREACH(read_queue_head[channel], rd_ptr)
			{
				// if COL_WRITE_CMD is the next command, then that means the appropriate row must already be open && batch first
				if(rd_ptr->command_issuable && (rd_ptr->next_command == COL_READ_CMD))
				{
					if (higher(rd_ptr, issue)) {
						issue = rd_ptr;
						read_issued = 1;
					}
				}
			}
		}

		if(!write_issued && !read_issued){
			// if no open rows, just issue any other available commands
			LL_FOREACH(write_queue_head[channel], wr_ptr)
			{
				if(wr_ptr->command_issuable)
				{
updateAggressiveTable(channel, wr_ptr);
					issue_request_command(wr_ptr);
					write_issued = 1;
					break;
				}
			}
		}

		if (!write_issued && !read_issued) {
			LL_FOREACH(read_queue_head[channel], rd_ptr)
			{
				// Random one
				if(rd_ptr->command_issuable)
				{
					if (higher(rd_ptr, issue)) {
						issue = rd_ptr;
						read_issued = 1;
					}
				}
			}
		}

		if(issue){
updateAggressiveTable(channel, issue);
			issue_request_command(issue);
			read_issued = 1;
			State * st = (State*)(issue->user_ptr);
			if ((st->marked) && issue->next_command == COL_READ_CMD)
				marked_num--;
		}
		rd_ptr = issue;
		// try auto-precharge
		if (!write_issued && !read_issued) {
aggressivePrecharge(channel);
			return; // no request issued, quit
		}

		if (!write_issued && read_issued) {
			wr_ptr = rd_ptr;
		}

		if (is_autoprecharge_allowed(channel, wr_ptr->dram_addr.rank, wr_ptr->dram_addr.bank)) {
			LL_FOREACH(write_queue_head[channel], auto_ptr) {
				if (!auto_ptr->request_served
						&& auto_ptr->dram_addr.rank == wr_ptr->dram_addr.rank
						&& auto_ptr->dram_addr.bank == wr_ptr->dram_addr.bank
						&& auto_ptr->dram_addr.row == wr_ptr->dram_addr.row) {
					return; // has hit, no auto precharge
				}
			}
			LL_FOREACH(read_queue_head[channel], auto_ptr) {
				if (!auto_ptr->request_served
						&& auto_ptr->dram_addr.rank == wr_ptr->dram_addr.rank
						&& auto_ptr->dram_addr.bank == wr_ptr->dram_addr.bank
						&& auto_ptr->dram_addr.row == wr_ptr->dram_addr.row) {
					return; // has hit, no auto precharge
				}
			}
			issue_autoprecharge(channel, wr_ptr->dram_addr.rank, wr_ptr->dram_addr.bank);
			// nothing issuable this cycle
			return;
		}
		return;
	}

	if( marked_num == 0 ){          // if last batch finish
		init_all_banks();
		for(int ch = 0; ch < MAX_NUM_CHANNELS; ch++){
			LL_FOREACH(read_queue_head[ch], rd_ptr){
				int index = 
					rd_ptr->thread_id * MAX_NUM_CHANNELS * MAX_NUM_RANKS * MAX_NUM_BANKS + 
					ch * MAX_NUM_RANKS * MAX_NUM_BANKS +
					(rd_ptr->dram_addr).rank * MAX_NUM_BANKS + 
					(rd_ptr->dram_addr).bank; 
				if ( load_bank[index] < MARKINGCAP ){                   // Mark the Req if load <  MARKINGCAP (Rule 1)
					load_bank[index]++;
					marked_num++;
					State * st = (State*)(rd_ptr->user_ptr);
					if (st == NULL) { 
						st = (State*) malloc (sizeof(State));
						st->marked = 0;
						st->incoming = 1;
						rd_ptr->user_ptr = st;
					}
					st->marked = 1;
				}
			}
		}
		int max_load = 0, all_load = 0;
		for(int coreid = 0; coreid < NUMCORES; coreid++){               // Compute all & max loading in each core
			for(int i = coreid * MAX_NUM_CHANNELS * MAX_NUM_RANKS * MAX_NUM_BANKS; 
					i < (coreid + 1) * MAX_NUM_CHANNELS * MAX_NUM_RANKS * MAX_NUM_BANKS;
					i++ ){
				all_load += load_bank[i];
				if ( max_load < load_bank[i] )
					max_load = load_bank[i];
			}
			load_max[coreid] = max_load;
			load_all[coreid] = all_load;
			max_load = 0;
			all_load = 0;
		}
	}
	request_t *issue = NULL;
	LL_FOREACH(read_queue_head[channel], rd_ptr) {                       // Start batch
		if(rd_ptr->command_issuable)
			if(higher(rd_ptr, issue))
				issue = rd_ptr;
	}
	if(issue){  		// Start issue
		updateAggressiveTable(channel, issue);
		issue_request_command(issue);
		read_issued = 1;
		State * st = (State*)(issue->user_ptr);
		if ((st->marked) && issue->next_command == COL_READ_CMD)
			marked_num--;
	}
	else {
		// prioritize open row hits
		LL_FOREACH(write_queue_head[channel], wr_ptr)
		{
			// if COL_WRITE_CMD is the next command, then that means the appropriate row must already be open
			if(wr_ptr->command_issuable && (wr_ptr->next_command == COL_WRITE_CMD))
			{
				//writes_done_this_drain[channel]++;
				updateAggressiveTable(channel, wr_ptr);
 				issue_request_command(wr_ptr);
				write_issued = 1;
				break;
			}
		}

		if (!write_issued) {
			// No read can issue, issue a random write
			LL_FOREACH(write_queue_head[channel], wr_ptr)
			{
				if(wr_ptr->command_issuable)
				{
					updateAggressiveTable(channel, wr_ptr);
 					issue_request_command(wr_ptr);
					write_issued = 1;
					break;
				}
			}
		}
	}
	rd_ptr = issue;
	// try auto-precharge
	if (!write_issued && !read_issued) {
		aggressivePrecharge (channel);
		return; // no request issued, quit
	}

	if (write_issued && !read_issued) {
		rd_ptr = wr_ptr;
	}


	if (is_autoprecharge_allowed(channel, rd_ptr->dram_addr.rank, rd_ptr->dram_addr.bank)) {
		LL_FOREACH(read_queue_head[channel], auto_ptr) {
			if (!auto_ptr->request_served
					&& auto_ptr->dram_addr.rank == rd_ptr->dram_addr.rank
					&& auto_ptr->dram_addr.bank == rd_ptr->dram_addr.bank
					&& auto_ptr->dram_addr.row == rd_ptr->dram_addr.row) {
				return; // has hit, no auto precharge
			}
		}
		LL_FOREACH(write_queue_head[channel], auto_ptr) {
			if (!auto_ptr->request_served
					&& auto_ptr->dram_addr.rank == rd_ptr->dram_addr.rank
					&& auto_ptr->dram_addr.bank == rd_ptr->dram_addr.bank
					&& auto_ptr->dram_addr.row == rd_ptr->dram_addr.row) {
				return; // has hit, no auto precharge
			}
		}
		// no hit pending, auto precharge
		issue_autoprecharge(channel, rd_ptr->dram_addr.rank, rd_ptr->dram_addr.bank);
		return;

	}
}
void scheduler_stats()
{
	/* Nothing to print for now. */
}


