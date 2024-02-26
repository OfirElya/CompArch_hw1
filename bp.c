/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */

#define _CRT_SECURE_NO_WARNINGS

#include "bp_api.h"
#include "math.h"
#include <stdio.h>
#include <malloc.h>

#define FAILURE -1
#define PC_SIZE 32

typedef enum {
	SNT = 0,
	WNT,
	WT,
	ST
} FSM_state;

/* A structure to return information about the current BTB block */
typedef struct {
	uint32_t tag;
	uint32_t target_pc;
	uint32_t* history;
	bool valid;

} BTB_block;

/* A structure to return information about the BTB */
typedef struct {
	BTB_block* btb_blocks;
	uint32_t tag_size;
	uint32_t history_size;
	uint32_t btb_size;
	uint32_t blocks_in_btb;
	int shared;
	int updates;
	int flushes;
	FSM_state initial_state;
	uint32_t** fsm_array;
	bool global_table;
	bool global_history;

} BTB_t;

// global variable - the BTB
BTB_t* BTB;

// helper functions
FSM_state update_state(FSM_state current_state, bool taken);
uint32_t calc_tag(uint32_t pc);
int find_block(uint32_t pc);
void update_history(uint32_t* history, bool taken, uint32_t mask);
int calcSharedEntry(int shared, uint32_t history, uint32_t pc, uint32_t history_size);
void is_flush(bool prediction, bool taken, bool wrongDst);
void free_space(bool global_history, bool global_table);

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
	bool isGlobalHist, bool isGlobalTable, int Shared) {

	BTB = (BTB_t*)malloc(sizeof(BTB_t));
	if (BTB == NULL)
		return FAILURE;

	BTB_block* btb_array = (BTB_block*)malloc(sizeof(BTB_block) * btbSize);
	if (btb_array == NULL)
		return FAILURE;

	// initialize BTB
	BTB->btb_blocks = btb_array;
	BTB->initial_state = fsmState;
	BTB->global_table = isGlobalTable;
	BTB->global_history = isGlobalHist;
	BTB->tag_size = tagSize;
	BTB->history_size = historySize;
	BTB->btb_size = btbSize;
	BTB->shared = Shared;
	BTB->blocks_in_btb = 0;
	BTB->flushes = 0;
	BTB->updates = 0;

	// initialize btb blocks array
	for (int i = 0; i < btbSize; i++) {
		BTB->btb_blocks[i].valid = false;
		BTB->btb_blocks[i].target_pc = 0;
		BTB->btb_blocks[i].tag = -1;
	}

	// initialize histories
	if (isGlobalHist) {
		uint32_t* shared_history = (uint32_t*)malloc(sizeof(uint32_t));
		if (shared_history == NULL)
			return FAILURE;
		*shared_history = 0;
		for (int i = 0; i < btbSize; i++) {
			BTB_block* current_block = &btb_array[i];
			current_block->history = shared_history;
		}
	}
	else {
		for (int i = 0; i < btbSize; i++) {
			uint32_t* local_history = (uint32_t*)malloc(sizeof(uint32_t));
			if (local_history == NULL)
				return FAILURE;
			*local_history = 0;
			BTB_block* current_block = &btb_array[i];
			current_block->history = local_history;
		}
	}

	// create fsm state array
	uint32_t** fsm_array = (uint32_t**)malloc(sizeof(uint32_t*) * btbSize);
	if (fsm_array == NULL)
		return FAILURE;
	BTB->fsm_array = fsm_array;

	uint32_t fsm_size = pow(2, historySize);

	// initialize fsm state array
	if (isGlobalTable) {
		uint32_t* global_state = (uint32_t*)malloc(sizeof(uint32_t) * fsm_size);
		if (global_state == NULL)
			return FAILURE;
		for (int j = 0; j < fsm_size; j++)
			global_state[j] = fsmState;
		for (int i = 0; i < btbSize; i++)
			fsm_array[i] = global_state;
	}
	else {
		for (int i = 0; i < btbSize; i++) {
			fsm_array[i] = (uint32_t*)malloc(sizeof(uint32_t) * fsm_size);
			if (fsm_array[i] == NULL)
				return FAILURE;
			for (int j = 0; j < fsm_size; j++) {
				fsm_array[i][j] = fsmState;
			}
		}
	}

	return 0;
}

bool BP_predict(uint32_t pc, uint32_t* dst) {
	
	*dst = pc + 4;
	
	// if BTB is empty predict NT
	if (!BTB->blocks_in_btb)
		return false;

	bool prediction = false; // default value

	// location points to the entry in fsm array that correlates to current history and share status
	uint32_t location;    

	int block_entry = find_block(pc);
	BTB_block* block = &(BTB->btb_blocks[block_entry]);
	uint32_t tag = calc_tag(pc);

	// local tables
	if (!BTB->global_table) { 
		// if wanted tag is not in BTB, predict NT
		if (tag != block->tag || !block->valid)
			prediction = false;
		// if tag exists in BTB, predict by state and history
		else 
			prediction = BTB->fsm_array[block_entry][*(block->history)] >> 1;
	}
	// global table
	else { 
		// if wanted tag is not in BTB, predict NT
		if (tag != block->tag || !block->valid)
			prediction = false;
		// tag exists in BTB, predict by state and history
		else { 
			location = calcSharedEntry(BTB->shared, *(block->history), pc, BTB->history_size);
			prediction = (BTB->fsm_array[block_entry][location]) >> 1;
		}
	}

	// if prediction is TAKEN, update dst address
	if (prediction)
		*dst = block->target_pc;

	return prediction;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {

	BTB->updates++;

	uint32_t tag = calc_tag(pc);
	uint32_t entry = find_block(pc);

	uint32_t history_mask = 1;
	for (int i = 1; i < BTB->history_size; i++)
		history_mask = (history_mask << 1) | 1;

	BTB_block* block = &(BTB->btb_blocks[entry]);
	// calc location in fsm array
	int location = calcSharedEntry(BTB->shared, *(block->history), pc, BTB->history_size);
    if(!BTB->global_table)
        location = *(block->history);
	// get current state of specified entry
	FSM_state current_state = BTB->fsm_array[entry][location];
	bool prediction = current_state >> 1;
    bool wrongDst = false;

	// if BTB entry is empty
	if (!block->valid) {
		BTB->blocks_in_btb++;
		block->tag = tag;
		block->target_pc = targetPc;
		block->valid = true;

		prediction = false; // for unfamiliar branch we predict NT

		// update FSM state
		// local tables
		if (!BTB->global_table) {
			BTB->fsm_array[entry][*(block->history)] = update_state(BTB->initial_state, taken);;
		}
		// global table
		else {
			BTB->fsm_array[entry][location] = update_state(current_state, taken);
		}

        update_history(block->history, taken, history_mask);
	}
	// A block exists in wanted entry
	else {

		// if block is the wanted block
		if (block->tag == tag ) {

            if(targetPc != block->target_pc)
                wrongDst = true;
			block->target_pc = targetPc;

			// update FSM state
			BTB->fsm_array[entry][location] = update_state(current_state, taken);

			// update history
			update_history(block->history, taken, history_mask);
		}
		// block not in BTB - need to replace entry
		else {
			block->target_pc = targetPc;
			block->tag = tag;

			prediction = false;

			if (!BTB->global_history)
				*block->history = 0;

			// update FSM state
			// local tables
			if (!BTB->global_table) {
				// initialize new block fsm states
                for(int i = 0; i < (int)pow(2,BTB->history_size); i++)
                    BTB->fsm_array[entry][i] = BTB->initial_state;
				// update fsm according to history
                if(BTB->global_history)
                    BTB->fsm_array[entry][(*block->history)] = update_state(BTB->initial_state, taken);
                else
                    BTB->fsm_array[entry][0] = update_state(BTB->initial_state, taken);
			}
			// global table
			else {
				location = calcSharedEntry(BTB->shared, *(block->history), pc, BTB->history_size);
				current_state = BTB->fsm_array[entry][location];
				BTB->fsm_array[entry][location] = update_state(current_state, taken);
			}

            update_history(block->history, taken, history_mask);
		}
	}

	// check for flushes
	is_flush(prediction, taken, wrongDst);

	return;
}

void BP_GetStats(SIM_stats* curStats) {
	curStats->br_num = BTB->updates;
	curStats->flush_num = BTB->flushes;

	int size = 0;
	int target_pc_size = PC_SIZE - 2;
	int valid_size = 1;
	int state_size = 2*pow(2, BTB->history_size);
	int blocks_num = BTB->btb_size;

	size = blocks_num * (BTB->tag_size + target_pc_size + valid_size);

	if (BTB->global_history) {
		size += BTB->history_size;
	}
	else {
		size += (BTB->history_size * blocks_num);
	}

	if (BTB->global_table) {
		size += state_size;
	}
	else {
		size += (state_size * blocks_num);
	}

	curStats->size = size;

	free_space(BTB->global_history, BTB->global_table);

	return;
}

// update the state of the FSM according the current state and TAKEN/NOT TAKEN
FSM_state update_state(FSM_state current_state, bool taken) {
	FSM_state new_state = WNT;
	switch (current_state) {
	case WT:
		if (taken)
			new_state = ST;
		else
			new_state = WNT;
		break;
	case ST:
		if (taken)
			new_state = ST;
		else
			new_state = WT;
		break;
	case WNT:
		if (taken)
			new_state = WT;
		else
			new_state = SNT;
		break;
	case SNT:
		if (taken)
			new_state = WNT;
		else
			new_state = SNT;
		break;
	}
	return new_state;
}

// calculate the tag from the pc according to btb size and tag size
uint32_t calc_tag(uint32_t pc) {
	if (BTB->tag_size == 0)
		return -1;

	uint32_t tmp = pc >> 2;
    tmp = tmp >> (int)log2(BTB->btb_size);
    return (tmp % (int)pow(2, BTB->tag_size));
}

// calculate the entry in btb array according to btb size bits in pc
int find_block(uint32_t pc) {
	return ((pc >> 2) % BTB->btb_size);
}

void update_history(uint32_t* history, bool taken, uint32_t mask) {
	uint32_t tmp = *history;
	if (taken)
		*(history) = ((tmp << 1) | 1) & mask;
	else {
		*(history) = (tmp << 1) & mask;
	}
	return;
}

// return the location in the fsm array of wanted entry according to pc and share policy
int calcSharedEntry(int shared, uint32_t history, uint32_t pc, uint32_t history_size) {
	uint32_t tmp_pc;
	int location = history;
	switch (shared) {
	case 0: //not shared
		location = history;
		break;
	case 1: //shared lsb
		tmp_pc = (pc << (PC_SIZE - 2 - history_size)) >> (PC_SIZE - 2 - history_size);
		location = (history ^ (tmp_pc >> 2));
		break;
	case 2: //shared mid
		tmp_pc = (pc << (PC_SIZE - 16 - history_size)) >> (PC_SIZE - 16 - history_size);
		location = (history ^ (tmp_pc >> 16));
		break;
	}
	return location;
}

// count the number of flushes
void is_flush(bool prediction, bool taken, bool wrongDst) {
	if (prediction != taken || (wrongDst && prediction))
		BTB->flushes++;
	return;
}

// free all allocated space
void free_space(bool global_history, bool global_table) {
	uint32_t** fsm_array = BTB->fsm_array;
	BTB_block* btb_array = BTB->btb_blocks;

	if (global_table) {
		free(fsm_array[0]);
	}
	else {
		for (int i = 0; i < BTB->btb_size; i++)
			free(fsm_array[i]);
	}
	free(fsm_array);

	if (global_history) {
		free((&btb_array[0])->history);
	}
	else {
		for (int i = 0; i < BTB->btb_size; i++)
			free((&btb_array[i])->history);
	}
	free(btb_array);

	free(BTB);

	return;
}
