/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */

#define _CRT_SECURE_NO_WARNINGS

#include "bp_api.h"
#include "math.h"

typedef enum {
	SNT = 0,
	WNT,
	WT,
	ST
} FSM_state;

/* A structure to return information about the current BTB block */
typedef struct {
	uint32_t tag;
	uint32_t branch_pc;
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

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){

	BTB = (BTB_t*)malloc(sizeof(BTB_t));
	if (BTB == NULL)
		return -1;

	BTB_block* btb_array = (BTB_block*)malloc(sizeof(BTB_block) * btbSize);
	if (btb_array == NULL)
		return -1;

	BTB->btb_blocks = btb_array;
	BTB->initial_state = fsmState;
	BTB->global_table = isGlobalTable;
	BTB->global_history = isGlobalHist;
	BTB->tag_size = tagSize;
	BTB->history_size = historySize;
	BTB->btb_size = btbSize;
	BTB->shared = Shared;
	BTB->blocks_in_btb = 0;

	for (int i = 0; i < btbSize; i++) {
		BTB->btb_blocks[i].valid = false;
	}

	if (isGlobalHist) {
		uint32_t* shared_history = (uint32_t*)malloc(sizeof(uint32_t));
		if (shared_history == NULL)
			return -1;
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
				return -1;
			*local_history = 0;
			BTB_block* current_block = &btb_array[i];
			current_block->history = local_history;
		}
	}

	uint32_t** fsm_array = (uint32_t**)malloc(sizeof(uint32_t*) * btbSize);
	if (fsm_array == NULL)
		return -1;
	BTB->fsm_array = fsm_array;

	uint32_t fsm_size = pow(2, historySize);
	fsmState = (uint32_t)fsmState;

	if (isGlobalTable) {
		uint32_t* global_state = (uint32_t*)malloc(sizeof(uint32_t) * fsm_size);
		if (global_state == NULL)
			return -1;
		for (int j = 0; j < fsm_size; j++)
			global_state[j] = fsmState;
		for (int i = 0; i < btbSize; i++)
			fsm_array[i] = global_state;
	}
	else {
		for (int i = 0; i < btbSize; i++) {
			fsm_array[i] = (uint32_t*)malloc(sizeof(uint32_t) * fsm_size);
			if (fsm_array[i] == NULL)
				return -1;
			for (int j = 0; j < fsm_size; j++) {
				fsm_array[i][j] = fsmState;
			}
		}
	}
	
	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
	
	*dst = pc + 4;
	if (!BTB->blocks_in_btb)
		return false;

	bool prediction = false;
	uint32_t history, tmp_pc, location;

	int block_entry = find_block(pc);
	BTB_block* block =&(BTB->btb_blocks[block_entry]);

	if (!BTB->global_table) {
		prediction = BTB->fsm_array[block_entry][*(block->history)] >> 1;
	}
	else {
		switch (BTB->shared) {
		case 0: //not shared
			prediction = BTB->fsm_array[0][*(block->history)] >> 1;
			break;
		case 1: //shared lsb
			history = *(block->history);
			tmp_pc = (pc << (32 - 2 - BTB->history_size)) >> (32 - 2 - BTB->history_size);
			location = (history ^ (tmp_pc >> 2));
			prediction = (BTB->fsm_array[0][location]) >> 1;
			break;
		case 2: //shared mid
			history = *(block->history);
			tmp_pc = (pc << (32 - 16 - BTB->history_size)) >> (32 - 16 - BTB->history_size);
			location = (history ^ (tmp_pc >> 16));
			prediction = (BTB->fsm_array[0][location]) >> 1;
			break;
		}
	}

	if (prediction)
		*dst = block->target_pc;
	return prediction;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {

	uint32_t tag = calc_tag(pc);
	uint32_t entry = find_block(pc);
	uint32_t history_mask = 1;
	for (int i = 1; i < BTB->history_size; i++)
		history_mask = (history_mask << 1) | 1;

	BTB_block* block = &(BTB->btb_blocks[entry]);

	// if block not in BTB
	if (!block->valid) {
		BTB->blocks_in_btb++;
		block->tag = tag;
		block->branch_pc = pc;
		block->target_pc = targetPc;
		block->valid = true;

		// update FSM state
		if (!BTB->global_table)
			BTB->fsm_array[entry][*(block->history)] = BTB->initial_state;
		else {
			FSM_state current_state = BTB->fsm_array[entry][*(block->history)];
			BTB->fsm_array[entry][*(block->history)] = update_state(current_state, taken);
		}

		// update history
		if (!BTB->global_history)
			block->history = 0;
		else {
			update_history(block->history, taken, history_mask);
		}
	}
	//BTB is not empty
	else {
		
		// if block already in BTB
		if (block->tag == tag) {

			// update FSM state
			FSM_state current_state = BTB->fsm_array[entry][*(block->history)];
			BTB->fsm_array[entry][*(block->history)] = update_state(current_state, taken);

			// update history
			update_history(block->history, taken, history_mask);
		}
		// block not in BTB - need to replace entry
		else {
			block->branch_pc = pc;
			block->target_pc = targetPc;
			block->tag = tag;
			block->valid = true;

			// update FSM state
			if (!BTB->global_table)
				BTB->fsm_array[entry][*(block->history)] = BTB->initial_state;
			else {
				FSM_state current_state = BTB->fsm_array[entry][*(block->history)];
				BTB->fsm_array[entry][*(block->history)] = update_state(current_state, taken);
			}

			// update history
			if (!BTB->global_history)
				block->history = 0;
			else {
				update_history(block->history, taken, history_mask);
			}
		}
	}
	
	return;
}

void BP_GetStats(SIM_stats *curStats){
	return;
}

FSM_state update_state(FSM_state current_state, bool taken) {
	switch (current_state) {
		case WT:
			if (taken)
				return ST;
			else
				return WNT;
		case ST:
			if (taken)
				return ST;
			else
				return WT;
		case WNT:
			if (taken)
				return WT;
			else
				return SNT;
		case SNT:
			if (taken)
				return WNT;
			else
				return SNT;
		}
}

uint32_t calc_tag(uint32_t pc) {
	uint32_t tmp = pc >> 2;
	int tag_size = BTB->tag_size;
	int tag_start = log(BTB->btb_size);
	return (((tmp >> tag_start) << (32 - tag_size)) >> (32 - tag_size));
}

int find_block(uint32_t pc) {
	return ((pc >> 2) % BTB->btb_size);
}

void update_history(uint32_t* history, bool taken, uint32_t mask) {
	if (taken)
		*(history) = ((*(history) << 1) | 1) & mask;
	else {
		*(history) = (*(history) << 1) & mask;
	}
	return;
}

