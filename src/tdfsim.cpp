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

/* The faulty_wire contains a list of wires that 
 * change their values in the fault simulation for a particular packet.
 * (that is wire_value1 != wire_value2) 
 * Note that the wire themselves are not necessarily a fault site.
 * The list is linked by the pnext pointers */

/* fault simulate a set of test vectors */
void ATPG::transition_delay_fault_simulation(int &total_detect_num) {

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
        f->fault_type = STUCK0;
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
        f->fault_type = STUCK1;
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
		        f->fault_type = STUCK0;
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
                f->fault_type = STUCK1;
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
	    cout << f->fault_no << f->node->name << ":" << (f->io?"O":"I") << (f->io?9:(f->index)) << "SA" << f->fault_type << endl;
	}
}//end gen fault list
