#include <stdio.h>
#include "utlist.h"
#include "utils.h"
#include "memory_controller.h"
#include "params.h"
#include <string.h>
#include <stdbool.h>
#define MAX_THREADS  64
// #define clusterThresh 0.5
// #define NUM_CHANNELS_HERE NUM_CHANNELS
extern long long int CYCLE_VAL;
struct Node  
{ 
  int data; 
  struct Node *next; 
}; 
int quantCounter;
int shuffleCycleInterval=0;
int bwClusterCount=0;
int bwShuffleIteration = 0;
int performDecSort = 0;
long int latency[MAX_NUM_CHANNELS][MAX_THREADS];
long int instructionCt[MAX_NUM_CHANNELS][MAX_THREADS];
long double hitCt[MAX_NUM_CHANNELS][MAX_THREADS];
double MPKI[MAX_NUM_CHANNELS][MAX_THREADS];
int bWCluster [MAX_NUM_CHANNELS][MAX_THREADS];
double bWClusterNiceness[MAX_NUM_CHANNELS][MAX_THREADS];
int latencyCluster [MAX_NUM_CHANNELS][MAX_THREADS];
double bankAccess[MAX_NUM_CHANNELS][MAX_THREADS][MAX_NUM_BANKS];
int threadCalls[MAX_NUM_CHANNELS][MAX_THREADS];
int threadCallCount[MAX_NUM_CHANNELS][MAX_THREADS];
double bankLevelParallelism[MAX_NUM_CHANNELS][MAX_THREADS];
long long int rowBufferArr [MAX_NUM_CHANNELS][MAX_THREADS][MAX_NUM_RANKS][MAX_NUM_BANKS];
double rowBufferHit [MAX_NUM_CHANNELS][MAX_THREADS];
double rowBufferLocality [MAX_NUM_CHANNELS][MAX_THREADS];
double clusterThresh = 0.7;
int threadCount = MAX_THREADS;
int firstQuantOut=0;

int selectedTheThread(int arr[],int n,int channel)
{

	for(int ch = 0; ch< threadCount-bwClusterCount;ch++) {
		int latencyThreadId =latencyCluster[channel][ch];
		// for (int i = 0; i< n;i++){
			if(arr[latencyThreadId] > 0)
				return latencyThreadId;
		// }
	}
	for(int ch = 0; ch< bwClusterCount;ch++) {
		int bwThreadId =bWCluster[channel][ch];
		// for (int i = 0; i< n;i++){
			if(arr[bwThreadId]>0)
				return bwThreadId;
		// }
	}
	return -1;
}
void incSort(double arr[],int ch[], int n) 
{ 
    for(int i=1; i<(n); i++)
    {
        int j=i;
        while(j>0 && arr[j]<arr[j-1])
        {
            double temp=arr[j];
            int temp2 = ch[j];
            arr[j]=arr[j-1];
            ch[j]=ch[j-1];
            arr[j-1]=temp;
            ch[j-1]=temp2;
            j--;
        }
    }
} 
void incSortSubArray(double arr[],int ch[], int start, int n) 
{ 
    int len = n-start;
    double arr2[len];
    int ch2[len];
    for (int i=start; i< n;i++){
        arr2[i-start]=arr[i];
        ch2[i-start]=ch[i];
    }
    incSort(arr2,ch2,len);
    for (int i=start; i< n;i++)
    {
        arr[i]=arr2[i-start];
        ch[i]=ch2[i-start];
    }    
} 
void decSort(double arr[],int ch[], int n) 
{ 
    for(int i=1; i<(n); i++)
    {
        int j=i;
        while(j>0 && arr[j]>arr[j-1])
        {
            double temp=arr[j];
            int temp2 = ch[j];
            arr[j]=arr[j-1];
            ch[j]=ch[j-1];
            arr[j-1]=temp;
            ch[j-1]=temp2;
            j--;
        }
    }
} 
void resetBandwidth()
{	
	memset(latency, 0, sizeof(latency[0][0]) * MAX_NUM_CHANNELS * MAX_THREADS);	  
}
void initializeInstructionCt()
{	
	memset(instructionCt, 0, sizeof(instructionCt[0][0]) * MAX_NUM_CHANNELS * MAX_THREADS);	  
}

void resetHitCt(){
	for (int i = 0; i < NUM_CHANNELS; i++)
	{
	    for (int j = 0; j < MAX_THREADS; j++)
	    {
	    	hitCt[i][j]=0.0;
	    }
	}
}
void resetbWCluster()
{
  	memset(bWCluster, -1, sizeof(bWCluster[0][0]) * MAX_NUM_CHANNELS * MAX_THREADS);	  
}
void resetbWClusterNiceness()
{
	for (int i = 0; i < NUM_CHANNELS; i++)
	{
	    for (int j = 0; j < MAX_THREADS; j++)
	    {
	    	bWClusterNiceness[i][j]=-1.0;
	    }
	}
}
void resetLatencyCluster()
{
	memset(latencyCluster, -1, sizeof(latencyCluster[0][0]) * MAX_NUM_CHANNELS * MAX_THREADS);	  
}
double findSum(double arr[],int n) 
{ 
    double sum = 0; 
    for (int i = 0; i < n; i++) 
    sum += arr[i]; 
    return sum; 
} 
void resetBankParrallelism(){	
	for (int i = 0; i < NUM_CHANNELS; i++)
	{
	    for (int j = 0; j < MAX_THREADS; j++)
	    {
	    	bankLevelParallelism[i][j]=-1.0;
	    }
	}
}

void resetBankAccess(){	
	for (int i = 0; i < NUM_CHANNELS; i++)
	{
	    for (int j = 0; j < MAX_THREADS; j++)
	    {
		    for (int k = 0; k < MAX_NUM_BANKS; k++)
		    {
		      	bankAccess[i][j][k] = 0;
		    }

	    }
	}
}
void resetThreadAccess(){	
	 // memset(threadCalls, 0.0, sizeof(threadCalls[0][0]) * MAX_NUM_CHANNELS * MAX_THREADS);
	 for (int i = 0; i < NUM_CHANNELS; i++)
	{
	    for (int j = 0; j < MAX_THREADS; j++)
	    {
	    	threadCalls[i][j]=0.0;
	    }
	}
}
void resetRowBufferArr(){
	for (int i = 0; i < NUM_CHANNELS; i++)
	{
	    for (int j = 0; j < MAX_THREADS; j++)
	    {
		    for (int k = 0; k < MAX_NUM_RANKS; k++)
		    {
		      	for (int l = 0; l < MAX_NUM_BANKS; l++)
			    {
			      	rowBufferArr[i][j][k][l] = -1;
			    }
		    }
	    }
	}
}

void updateRowBuffer(int channel,int threadId,int bank,long long int rowBuffer){

}

void checkRowHit(int channel,int threadId,int rank,int bank,long long int rowBuffer){
	if(rowBufferArr[channel][threadId][rank][bank]==rowBuffer)
	{
		rowBufferHit[channel][threadId]++;
	}
}

void resetRowBufferHit(){
	// memset(rowBufferHit, 0.0, sizeof(rowBufferHit[0][0]) * MAX_NUM_CHANNELS * MAX_THREADS);	  
	for (int i = 0; i < NUM_CHANNELS; i++)
	{
	    for (int j = 0; j < MAX_THREADS; j++)
	    {
	    	rowBufferHit[i][j]=0.0;
	    }
	}
}

void resetRowBufferLocality(){
	//memset(rowBufferLocality, -1.0, sizeof(rowBufferLocality[0][0]) * MAX_NUM_CHANNELS * MAX_THREADS);	  
	for (int i = 0; i < NUM_CHANNELS; i++)
	{
	    for (int j = 0; j < MAX_THREADS; j++)
	    {
	    	rowBufferLocality[i][j]=-1.0;
	    }
	}
}
void resetThreadCallCount(){
	memset(threadCallCount, 0, sizeof(threadCallCount[0][0]) * MAX_NUM_CHANNELS * MAX_THREADS);	  			
}
void init_scheduler_vars()
{
	// initialize all scheduler variables here
	quantCounter = 0;
	initializeInstructionCt();
	resetBandwidth();
	resetHitCt();
	resetbWCluster();
	resetLatencyCluster();
	return;
}

int oneTimeCheck = 0;

// write queue high water mark; begin draining writes if write queue exceeds this value
#define HI_WM 40

// end write queue drain once write queue has this many writes in it
#define LO_WM 20

// 1 means we are in write-drain mode for that channel
int drain_writes[MAX_NUM_CHANNELS];

/* Each cycle it is possible to issue a valid command from the read or write queues
   OR
   a valid precharge command to any bank (issue_precharge_command())
   OR 
   a valid precharge_all bank command to a rank (issue_all_bank_precharge_command())
   OR
   a power_down command (issue_powerdown_command()), programmed either for fast or slow exit mode
   OR
   a refresh command (issue_refresh_command())
   OR
   a power_up command (issue_powerup_command())
   OR
   an activate to a specific row (issue_activate_command()).

   If a COL-RD or COL-WR is picked for issue, the scheduler also has the
   option to issue an auto-precharge in this cycle (issue_autoprecharge()).

   Before issuing a command it is important to check if it is issuable. For the RD/WR queue resident commands, checking the "command_issuable" flag is necessary. To check if the other commands (mentioned above) can be issued, it is important to check one of the following functions: is_precharge_allowed, is_all_bank_precharge_allowed, is_powerdown_fast_allowed, is_powerdown_slow_allowed, is_powerup_allowed, is_refresh_allowed, is_autoprecharge_allowed, is_activate_allowed.
   */

void schedule(int channel)
{
	request_t * rd_ptr = NULL;
	request_t * wr_ptr = NULL;
	// if in write drain mode, keep draining writes until the
	// write queue occupancy drops to LO_WM
	if (drain_writes[channel] && (write_queue_length[channel] > LO_WM)) {
	  drain_writes[channel] = 1; // Keep draining.
	}
	else {
	  drain_writes[channel] = 0; // No need to drain.
	}

	// initiate write drain if either the write queue occupancy
	// has reached the HI_WM , OR, if there are no pending read
	// requests
	if(write_queue_length[channel] > HI_WM)
	{
		drain_writes[channel] = 1;
	}
	else {
	  if (!read_queue_length[channel])
	    drain_writes[channel] = 1;
	}


	// If in write drain mode, look through all the write queue
	// elements (already arranged in the order of arrival), and
	// issue the command for the first request that is ready
	if(drain_writes[channel])
	{

		LL_FOREACH(write_queue_head[channel], wr_ptr)
		{
			if(wr_ptr->command_issuable)
			{
				issue_request_command(wr_ptr);
				break;
			}
		}
		return;
	}
	// Draining ReadsthreadCallCount
	// look through the queue and find the first request whose
	// command can be issued in this cycle and issue it 
	// Simple FCFS 
	if(!drain_writes[channel])
	{
		if(CYCLE_VAL/10000000 > quantCounter)
		{ // valid quant 
			firstQuantOut = 1;
			printf("got a quant %d\n",++quantCounter);
			resetbWCluster();
			resetLatencyCluster();
			resetRowBufferLocality();
			resetBankParrallelism();
			resetbWClusterNiceness();
			for (int i = 0; i < NUM_CHANNELS; i++) // MPKI calculation
			{
			    for (int j = 0; j < MAX_THREADS; j++)
			    {
			    	if(instructionCt[i][j]!=0){
				      MPKI[i][j] = (1 - hitCt[i][j]/instructionCt[i][j])*1000; 			    // MPKI =(1-(hitcount/total instruction count))*1000	
			    	}
			    }
			}

			for (int i = 0; i < NUM_CHANNELS; i++)
			{
			    int ctr= 0;
			    int jVal = 0;
			    int sumBW = 0;
			    int totalBW = 0;
			    for (int j = 0; j < MAX_THREADS; j++)
			    {
			    	totalBW = totalBW+latency[i][j];
			    }
			    int ptr=0;
			    printf("total BW is: %d\n ", totalBW);
			    while(ptr < MAX_THREADS) // bandwidth cluster creaation
			    {
			    	jVal =-1;
				    int minVal = 99999;
			    	for (int j = 0; j < MAX_THREADS; j++)
				    {
				    	if(MPKI[i][j]!=-1 && MPKI[i][j]< minVal)
				    	{
					      	minVal = MPKI[i][j];
					      	jVal = j;
				    	}
				    }
				    if(jVal== -1)
				    {
				    	break;
				    }

				    if(sumBW+latency[i][jVal]< totalBW*clusterThresh)
				    {
					    sumBW += latency[i][jVal];
				    	bWCluster[i][ctr] = jVal;
				    	ctr=ctr+1;
				      	MPKI[i][jVal]=-1;
				    }
				    else
				    {
				    	MPKI[i][jVal] = minVal; 
				    	jVal=-1;
				    	break;
				    }
				    ptr++;
			    }
			    ctr = 0;
			    for (int j = 0; j < MAX_THREADS; j++) // latency cluster creation
			    {
			    	if(MPKI[i][j]!=-1 )
			    	{
				      	latencyCluster[i][ctr] = j;
				      	ctr++;
			    	}
			    }

			    if(threadCount == MAX_THREADS)
			    {
			    	threadCount=0;
			    	for (int j = 0; j < MAX_THREADS; j++)
				    {
				    	if(latencyCluster[i][j]!=-1 )
				    	{
					      	threadCount++;
				    	}
				    	if(bWCluster[i][j]!=-1){
					      	threadCount++;
				    	}
				    }
			    }
			    
			    printf("Thread count is: %d\n", threadCount);
			    printf("sum BW is: %d\n ", sumBW);
			    // clustering algo complete
			   	int readInstructionCount = 0;
			    LL_FOREACH(read_queue_head[i],rd_ptr)
				{
					if(rd_ptr->command_issuable)
					{						
						if (rd_ptr->next_command == COL_READ_CMD)
						{
							int threadId = rd_ptr->thread_id;
							int bank = rd_ptr->dram_addr.bank;
							bankAccess[i][threadId][bank] =1.0;
							threadCalls[i][threadId]++;
							readInstructionCount++;
						}
					}
				}

				for (int j=0;j<MAX_THREADS;j++) // calculation of bank level parralelism
				{ 
					if(threadCalls[i][j]>0)
					{
					    int n = sizeof(bankAccess[i][j]) / sizeof(bankAccess[i][j][0]); 
						bankLevelParallelism[i][j]=findSum(bankAccess[i][j],n)/threadCalls[i][j];
					}
				}
				for (int j=0;j<MAX_THREADS;j++) // calculation of bank level parralelism
				{ 
					if(rowBufferHit[i][j]>-1.0 && threadCallCount[i][j]>0)
					{
						rowBufferLocality[i][j]=rowBufferHit[i][j]/threadCallCount[i][j];
					}
				}

			}

			bwClusterCount=0;
			for (int ctr=0;ctr<threadCount;ctr++){
				if(bWCluster[channel][ctr]!=-1){
					bWClusterNiceness[channel][ctr] = bankLevelParallelism[channel][ctr]-rowBufferLocality[channel][ctr];
					bwClusterCount++;
				}
			}
			bwShuffleIteration = bwClusterCount;
			decSort(bWClusterNiceness[channel],bWCluster[channel],bwClusterCount);
			incSort(MPKI[channel],latencyCluster[channel],threadCount-bwClusterCount);

			printf("\n");
			resetBandwidth(); // reset bandwidth array at the end of the quantum
			resetBankAccess();
			resetBankParrallelism();
			resetRowBufferArr();
			resetRowBufferHit();
			resetThreadCallCount();
		}

		if(shuffleCycleInterval==800){
			shuffleCycleInterval=1;
			if(performDecSort==1){
				incSortSubArray(bWClusterNiceness[channel],bWCluster[channel],bwShuffleIteration-1,bwClusterCount);				
				if(--bwShuffleIteration==0){
					performDecSort = 0;
					bwShuffleIteration = 1;
				}
			}else{
				decSort(bWClusterNiceness[channel],bWCluster[channel],bwShuffleIteration);
				if(++bwShuffleIteration>bwClusterCount){
					performDecSort = 1;
					bwShuffleIteration = bwClusterCount;
				}
			}
		}
		shuffleCycleInterval++;
		int threadCallArr [threadCount];
		for (int i = 0; i< threadCount;i++){
			threadCallArr[i]=0;	
		}
		int chk = 0;
		LL_FOREACH(read_queue_head[channel],rd_ptr)
		{
			if(rd_ptr->command_issuable)
			{
				int threadId = rd_ptr->thread_id;
			//	printf("%d->", threadId);
				threadCallArr[threadId]=1;
				chk =1;
			}
		}
		//printf("\n");
		int selectedThread=-1;
		if(firstQuantOut==1 && chk==1)
			selectedThread = selectedTheThread(threadCallArr,threadCount,channel);
		// firstQuantOut=0;
		LL_FOREACH(read_queue_head[channel],rd_ptr)
		{
			if(rd_ptr->command_issuable)
			{
				int threadId = rd_ptr->thread_id;
				if(firstQuantOut==0||selectedThread == threadId||selectedThread==-1)
				{
					instructionCt[channel][rd_ptr->thread_id] ++;
					issue_request_command(rd_ptr);
					if(rd_ptr->next_command == ACT_CMD)
					{
						rowBufferArr[channel][threadId][rd_ptr->dram_addr.rank][rd_ptr->dram_addr.bank]= rd_ptr->dram_addr.row;
					}
					if (rd_ptr->next_command == COL_READ_CMD)
					{
						hitCt[channel][threadId]++;
						latency[channel][threadId] +=  CYCLE_VAL+ T_CAS + T_DATA_TRANS - rd_ptr->arrival_time;
						checkRowHit(channel,threadId,rd_ptr->dram_addr.rank,rd_ptr->dram_addr.bank,rd_ptr->dram_addr.row);
						threadCallCount[channel][threadId]++;
					}
					break;
				}
				
			}
		}
		return;
	}
}

void scheduler_stats()
{
  /* Nothing to print for now. */
	// printf("latency stat starts here 2 \n");
	// for (int i = 0; i < NUM_CHANNELS; i++)
	//   {
	//   	printf("Channel Num: %d\n", i);
	//     for (int j = 0; j < MAX_THREADS; j++)
	//     {
	//       printf("| %d : %ld ", j ,latency[i][j] );
	//     }
	//     printf("\n");
	//   }
	// printf("latency stats end here\n");
	printf("MPKI stat starts here \n");
	for (int i = 0; i < NUM_CHANNELS; i++)
	  {
	  	printf("Channel Num: %d\n", i);
	    for (int j = 0; j < MAX_THREADS; j++)
	    {
	    	if(MPKI[i][j]!=-1.0)
	      		printf("| %d : %f ", j ,MPKI[i][j] );
	    }
	    printf("\n");
	  }
	printf("MPKI stats end here\n");
	printf("BW CLUSTER stat starts here \n");
	for (int i = 0; i < NUM_CHANNELS; i++)
	  {
	  	printf("Channel Num: %d\n", i);
	    for (int j = 0; j < MAX_THREADS; j++)
	    {
	    	if(bWCluster[i][j]!=-1){
	    		printf("| %d : %d ", j ,bWCluster[i][j] );
	    	}
	    }
	    printf("\n");
	  }
	printf("BW CLuster stat end here\n");
	printf("Latency CLUSTER stat starts here \n");
	for (int i = 0; i < NUM_CHANNELS; i++)
	  {
	  	printf("Channel Num: %d\n", i);
	    for (int j = 0; j < MAX_THREADS; j++)
	    {
	    	if(latencyCluster[i][j]!=-1){
	    		printf("| %d : %d ", j ,latencyCluster[i][j] );
	    	}
	    }
	    printf("\n");
	  }

	// printf("Latency CLuster stat end here\n");
	printf("Bank Level Paralleism CLUSTER stat starts here \n");

	for (int i = 0; i < NUM_CHANNELS; i++)
	  {
	  	printf("Channel Num: %d\n", i);
	    for (int j = 0; j < MAX_THREADS; j++)
	    {
	    	if(bankLevelParallelism[i][j]!= -1.0){
	    		printf("| %d : %f ", j ,bankLevelParallelism[i][j] );
	    	}
	    }
	    printf("\n");
	  }
	
	printf("Bank Level Paralleism CLuster stat end here\n");

	// printf("threadCalls stat starts here \n");

	// for (int i = 0; i < NUM_CHANNELS; i++)
	//   {
	//   	printf("Channel Num: %d\n", i);
	//     for (int j = 0; j < MAX_THREADS; j++)
	//     {
 //    		printf("| %d : %d ", j ,threadCalls[i][j] );
	//     }
	//     printf("\n");
	//   }
	
	// printf("threadCalls stat end here\n");

	// printf("Row buffer hit stats start \n");
	// for (int i = 0; i < NUM_CHANNELS; i++)
	// {
	//   	printf("Channel Num: %d\n", i);
	//     for (int j = 0; j < MAX_THREADS; j++)
	//     {
	//     	if(rowBufferHit[i][j]!= 0.0){
	//     		printf("| %d : %f ", j ,rowBufferHit[i][j] );
	//     	}
	//     }
	//     printf("\n");
	// }
	// printf("Row buffer hit stats end \n");
	// printf("threadCalls count stat starts here \n");

	// for (int i = 0; i < NUM_CHANNELS; i++)
	//   {
	//   	printf("Channel Num: %d\n", i);
	//     for (int j = 0; j < MAX_THREADS; j++)
	//     {
	//     	if(threadCallCount[i][j]!=0)
 //    			printf("| %d : %d ", j ,threadCallCount[i][j] );
	//     }
	//     printf("\n");
	//   }
	// printf("threadCalls count stat end here \n");

	// printf("Row buffer locality stats start here\n");
	// for (int i = 0; i < NUM_CHANNELS; i++)
	// {
	//   	printf("Channel Num: %d\n", i);
	//     for (int j = 0; j < MAX_THREADS; j++)
	//     {
	//     	if(rowBufferLocality[i][j]!=-1.0){
	//     		printf("| %d : %f ", j ,rowBufferLocality[i][j] );
	//     	}
	//     }
	//     printf("\n");
	// }
	// printf("Row buffer locality stats end here\n");

	printf("bWClusterNiceness stats start here\n");
	for (int i = 0; i < NUM_CHANNELS; i++)
	{
	  	printf("Channel Num: %d\n", i);
	    for (int j = 0; j < MAX_THREADS; j++)
	    {
	    	if(bWClusterNiceness[i][j]!=-1.0){
	    		printf("| %d : %f ", j ,bWClusterNiceness[i][j] );
	    	}
	    }
	    printf("\n");
	}
	printf("bWClusterNiceness stats end here\n");
}
void updateThreadBandwidth(request_t * request)
{
	// int channel = request->dram_addr.channel;
	// int threadId = request->thread_id;
	// latency[channel][threadId] = latency[channel][threadId]+ request->latency;
	// latency[channel][threadId] = latency[channel][threadId] + CYCLE_VAL+ T_CAS + T_DATA_TRANS - request->arrival_time;
}