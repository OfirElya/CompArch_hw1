/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */

#define _CRT_SECURE_NO_WARNINGS

#include "bp_api.h"
#include "math.h"
#include <stdio.h>
#include <malloc.h>

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
int calcSharedEntry(int shared, uint32_t history,uint32_t pc, uint32_t history_size);
void is_flush(FSM_state current_state, bool taken);
void free_space(bool global_history, bool global_table);

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
	BTB->flushes = 0;
	BTB->updates = 0;

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
	uint32_t location;

	int block_entry = find_block(pc);
	BTB_block* block = &(BTB->btb_blocks[block_entry]);
    uint32_t tag = calc_tag(pc);

	if (!BTB->global_table) { // local histories
		if (tag != block->tag) // if wanted tag is not in BTB, predict NT
			prediction = false;
		else // tag exists in BTB
			prediction = BTB->fsm_array[block_entry][*(block->history)] >> 1;
	}
	else { // global history
		if (tag != block->tag) // if wanted tag is not in BTB, predict NT
			prediction = BTB->initial_state >> 1;
		else { // tag exists in BTB
			location = calcSharedEntry(BTB->shared, *(block->history), pc, BTB->history_size);
			prediction = (BTB->fsm_array[0][location]) >> 1;
		}
	}

	// if prediction is TAKEN, update dst address
	if (prediction)
		*dst = block->target_pc;
	return prediction;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
//    printf("got here with history %d, will update according to %d, fsm_array : ", *BTB->btb_blocks[0].history, taken);
	
	BTB->updates++;

	uint32_t tag = calc_tag(pc);
	uint32_t entry = find_block(pc);

	uint32_t history_mask = 1;
	for (int i = 1; i < BTB->history_size; i++)
		history_mask = (history_mask << 1) | 1;

	BTB_block* block = &(BTB->btb_blocks[entry]);
	// calc location in fsm array
	int location = calcSharedEntry(BTB->shared, *(block->history), block->branch_pc, BTB->history_size);
	FSM_state current_state = BTB->fsm_array[entry][location];
	printf("tag 0x%x entry %d location %d state %d history %d\n", tag, entry, location, current_state, *block->history);

	// if block not in BTB
	if (!block->valid) {
		BTB->blocks_in_btb++;
		block->tag = tag;
		block->branch_pc = pc;
		block->target_pc = targetPc;
		block->valid = true;

		// update FSM state
		if (!BTB->global_table) {
			BTB->fsm_array[entry][*(block->history)] = BTB->initial_state;
			current_state = BTB->initial_state;
		}
		else
			BTB->fsm_array[entry][location] = update_state(current_state, taken);
		
		// update history
		if (!BTB->global_history)
			*block->history = 0;
		else
			update_history(block->history, taken, history_mask);
	}
	// A block exists in wanted entry
	else {

		// if block is the wanted block
		if (block->tag == tag) {

			// update FSM state
			BTB->fsm_array[entry][location] = update_state(current_state, taken);

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
			if (!BTB->global_table){
				BTB->fsm_array[entry][*(block->history)] = BTB->initial_state;
				current_state = BTB->initial_state;
			}
			else {
				printf("BTB->fsm_array[entry][0] = %d\n", BTB->fsm_array[entry][0]);
				printf("BTB->fsm_array[entry][1] = %d\n", BTB->fsm_array[entry][1]);
				BTB->fsm_array[entry][location] = update_state(current_state, taken);
			}

			// update history
			if (!BTB->global_history)
				*block->history = 0;
			else {
				uint32_t old_history = *block->history;
				update_history(block->history, taken, history_mask);
				printf("block->history = %d --> %d\n", old_history, *block->history);
			}
		}
	}

	is_flush(current_state, taken);

	return;
}

void BP_GetStats(SIM_stats *curStats){
	curStats->br_num = BTB->updates;
	curStats->flush_num = BTB->flushes;

	int size = 0;
	int target_pc_size = 32;
	int valid_size = 1;
	int state_size = 2;

	int blocks_num = pow(2, BTB->btb_size);
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
	uint32_t tmp = pc >> 2;
	int tag_size = BTB->tag_size;
	int tag_start = log(BTB->btb_size);
	return (((tmp >> tag_start) << (32 - tag_size)) >> (32 - tag_size));
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
int calcSharedEntry(int shared, uint32_t history ,uint32_t pc, uint32_t history_size){
	uint32_t tmp_pc;
	int location = history;
    switch (shared) {
        case 0: //not shared
            location = history;
            break;
        case 1: //shared lsb
            tmp_pc = (pc << (32 - 2 - history_size)) >> (32 - 2 - history_size);
            location = (history ^ (tmp_pc >> 2));
            break;
        case 2: //shared mid
            tmp_pc = (pc << (32 - 16 - history_size)) >> (32 - 16 - history_size);
            location = (history ^ (tmp_pc >> 16));
            break;
    }
    return location;
}

// count the number of flushes
void is_flush(FSM_state current_state, bool taken) {
	bool prediction = current_state >> 1;
	if (prediction != taken)
		BTB->flushes++;

	printf("current state is %d, taken is %d, num flushes is %d\n\n", current_state, taken, BTB->flushes);
	return;
}

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
