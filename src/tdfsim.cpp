/**********************************************************************/
/*  Parallel-Fault Event-Driven Transition Delay Fault Simulator      */
/*                                                                    */
/*           Author: Tsai-Chieh Chen                                  */
/*           last update : 10/22/2018                                 */
/**********************************************************************/

#include "atpg.h"

/* pack 16 faults into one packet.  simulate 16 faults together. 
 * the following variable name is somewhat misleading */
#define num_of_pattern 16
#define num_of_tdfaults_in_parallel  16

/* The faulty_wire contains a list of wires that 
 * change their values in the fault simulation for a particular packet.
 * (that is wire_value1 != wire_value2) 
 * Note that the wire themselves are not necessarily a fault site.
 * The list is linked by the pnext pointers */

/* fault simulate a set of test vectors */
void ATPG::transition_delay_fault_simulation(int &total_detect_num) {
    int current_detect_num = 0;
    //for every test vector
    for(int i = vectors.size() - 1; i >= 0; --i){
        tdfault_sim_a_vector(vectors[i], current_detect_num);
        total_detect_num += current_detect_num;
        fprintf(stdout, "vector[%d] detects %d faults (%d)\n", i, current_detect_num, total_detect_num);
    }
}

void ATPG::tdfault_sim_a_vector(const string &vec, int &num_of_current_detect) {
    const int numI = cktin.size();
    const int numCkt = sort_wlist.size();

    //for every input, set value to the current vector value V1
    for(int i = 0; i < numI; ++i){
        cktin[i]->value = ctoi(vec[i]);
    }

    //init the circuit: mark all inputs as changed and all other node as unknown
    for(int i = 0; i < numCkt; ++i){
        if(i < numI) sort_wlist[i]->set_changed();
        else sort_wlist[i]->value = U; 
    }

    //do a fault free sim
    sim();

    //expand the fault-free value into 32 bits, store it in wire_value1 and wire_value2
    for(int i = 0; i < numCkt; ++i){
        switch (sort_wlist[i]->value) {
            case 2:
                sort_wlist[i]->wire_value1 = 0x55555555; // 01 represents unknown
                sort_wlist[i]->wire_value2 = 0x55555555;
                break;
	        case 1:
		        sort_wlist[i]->wire_value1 = ALL_ONE;  // 11 represents logic one
		        sort_wlist[i]->wire_value2 = ALL_ONE;
		        break;
		    case 0:
		        sort_wlist[i]->wire_value1 = ALL_ZERO; // 00 represents logic zero
		        sort_wlist[i]->wire_value2 = ALL_ZERO;
		        break;
        }
    }

    //some declare for parallel fault sim
    fptr simulated_fault_list[num_of_tdfaults_in_parallel];
    int num_of_fault = 0;
    int start_wire_index = 10000;
    
    //for every undetected fault, check which faults are activated
    for (auto pos = flist_undetect.cbegin(); pos != flist_undetect.cend(); ++pos) {
        fptr f = *pos;
        //consider only active fault
        if (f->fault_type == sort_wlist[f->to_swlist]->value) {
            
            /*
            //if f is gate output fault
            if(f->io == GO){
                //if this wire is not yet marked as faulty
                if (!(sort_wlist[f->to_swlist]->is_faulty())) {
                    //mark as faulty
                    sort_wlist[f->to_swlist]->set_faulty();
                    //insert the corresponding wire to the list of faulty wires
                    wlist_faulty.push_front(sort_wlist[f->to_swlist]);
                }

                //add the fault to the simulated fault list and inject the fault
                simulated_fault_list[num_of_fault] = f;
                inject_fault_value(sort_wlist[f->to_swlist], num_of_fault, f->fault_type);

                //mark the wire as having a fault injected
                sort_wlist[f->to_swlist]->set_fault_injected();
                //schedule the outputs of this gate 
                for (auto pos_n : sort_wlist[f->to_swlist]->onode) {
                    pos_n->owire.front()->set_scheduled();
                }

                //increment the number of simulated faults in this packet
                num_of_fault++;
                //start_wire_index keeps track of the smallest level of fault in this packet.
                start_wire_index = min(start_wire_index, f->to_swlist);
            }//end f is gate output fault

            //if f is gate input fault
            else{
                //check if f is propagated to gate output
                faulty_wire = get_faulty_wire(f, fault_type);
                if (faulty_wire != nullptr) {
	                //if faulty_wire is not already marked as faulty
	                if (!(faulty_wire->is_faulty())) {
	                    //mark it as faulty
	                    faulty_wire->set_faulty();
	                    //add the wire to the list of faulty wires.
	                    wlist_faulty.push_front(faulty_wire);
	                }
	
	                //add the fault to the simulated list and inject it
	                simulated_fault_list[num_of_fault] = f;
	                inject_fault_value(faulty_wire, num_of_fault, fault_type);
	
	                //mark the faulty_wire as having a fault injected
	                faulty_wire->set_fault_injected();
	                //schedule the outputs of this gate
	                for (auto pos_n : faulty_wire->onode) {
	                    pos_n->owire.front()->set_scheduled();
	                }
	                num_of_fault++;
	                start_wire_index = min(start_wire_index, f->to_swlist);
                }//end if faulty_wire != NULL
            }//end if f is gate input fault
            */
        }//end if fault is not activate

        //if packet is full or no more undetected fault => do fault sim
        if ((num_of_fault == num_of_faults_in_parallel) || (next(pos, 1) == flist_undetect.cend())) {
            //shift in a bit and obtain V2 pattern

            //do fault sim to check which faults are detected

        }//end fault sim
    }//end for every undetected fault

    //drop fault


}

void ATPG::generate_tdfault_list() {
    wptr w;
    nptr n;
    fptr_s f;
    //walk through every wire in the circuit
    for(auto pos: sort_wlist){
        //target wire
        w = pos;
        //target gate
        n = w->inode.front();


        //create a gate output SA0 for target gate
        //create a new fault
        f = move(fptr_s(new(nothrow) FAULT));
        if (f == nullptr) error("No more room!");
        //setting the fault
        f->eqv_fault_num = 1;
        f->node = n;
        f->io = GO;
        f->fault_type = STR;
        f->to_swlist = w->wlist_index;
        //setting the fault list
        ++num_of_gate_fault;
        flist_undetect.push_front(f.get());
        flist.push_front(move(f));

        //create a gate output SA1 for target gate
        //create a new fault
        f = move(fptr_s(new(nothrow) FAULT));
        if (f == nullptr) error("No more room!");
        //setting the fault
        f->eqv_fault_num = 1;
        f->node = n;
        f->io = GO;
        f->fault_type = STF;
        f->to_swlist = w->wlist_index;
        //setting the fault list
        ++num_of_gate_fault;
        flist_undetect.push_front(f.get());
        flist.push_front(move(f));

        //if this wire is stem
        if(w->onode.size() > 1){
            //for each branch
            for(nptr nptr_ele: w->onode){
                //create SA0 for gate input
                //create a new fault
		        f = move(fptr_s(new(nothrow) FAULT));
		        if (f == nullptr) error("No more room!");
		        //setting the fault
		        f->eqv_fault_num = 1;
		        f->node = nptr_ele;
		        f->io = GI;
		        f->fault_type = STR;
		        f->to_swlist = w->wlist_index;
                //set fualt index for GI fault
                for (int k = 0, m = nptr_ele->iwire.size(); k < m; ++k) {
                    if (nptr_ele->iwire[k] == w) f->index = k;
                }
		        //setting the fault list
		        ++num_of_gate_fault;
		        flist_undetect.push_front(f.get());
		        flist.push_front(move(f));
		
                //create SA1 for gate input
                //create a new fault
                f = move(fptr_s(new(nothrow) FAULT));
                if (f == nullptr) error("No more room!");
                //setting the fault
                f->eqv_fault_num = 1;
                f->node = nptr_ele;
                f->io = GI;
                f->fault_type = STF;
                f->to_swlist = w->wlist_index;
                //set fualt index for GI fault
                for (int k = 0, m = nptr_ele->iwire.size(); k < m; ++k) {
                    if (nptr_ele->iwire[k] == w) f->index = k;
                }
                //setting the fault list
                ++num_of_gate_fault;
                flist_undetect.push_front(f.get());
                flist.push_front(move(f));
            }//end for each branch
        }//end if w is branch
    }//end for loop

    flist.reverse();
    flist_undetect.reverse();
    /*walk through all faults, assign fault_no one by one  */
	int fault_num = 0;
	for (fptr f: flist_undetect) {
	    f->fault_no = fault_num;
	    ++fault_num;
	    //cout << f->fault_no << f->node->name << ":" << (f->io?"O":"I") << (f->io?9:(f->index)) << "SA" << f->fault_type << endl;
	}
    fprintf(stdout, "#number of equivalent faults = %d\n", fault_num);
}//end gen fault list
