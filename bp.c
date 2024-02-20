/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"

BTB_t* BTB;

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){

	BTB = (BTB_t*)malloc(sizeof(BTB_t) * btbSize);
	if (BTB == NULL)
		return -1;

	BTB->initial_state = fsmState;
	BTB->global = isGlobalTable;
	BTB->tag_size = tagSize;
	BTB->size = btbSize;

	if (isGlobalHist) {
		BTB->historys_p = (char*)malloc(historySize);
	}
	else {
		BTB->historys_p = (char*)malloc(historySize * btbSize);
	}

	if (BTB->historys_p == NULL)
		return -1;
	
	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
	int i;
	*dst = pc + 4;
	bool prediction = false;
    if(BTB->btb_blocks == NULL)
        return false;
	for (i = 0; i < BTB->size; i++) {
		BTB_block* current_block = BTB->btb_blocks[i];
		if (!current_block->valid || current_block->branch_pc != pc)
			continue;
		if (current_block->current_state > 1  )
			prediction = true;
		if (prediction)
			*dst = current_block->target_pc;
	}
	return prediction;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	
	
	
	
	
	return;
}

void BP_GetStats(SIM_stats *curStats){
	return;
}

