// main.cpp
#include "systemc.h"

int sc_main(int argc, char *argv[]) {
  sc_signal<int> A, B, R_M, R_T1, R_T2, R_T3, R_C;
  sc_set_time_resolution(1, SC_NS); // V2.0
  sc_set_default_time_unit(1, SC_NS);
  sc_clock clock("clock", 20, SC_NS, 0.5, 15, SC_NS);



  // trace file creation
  sc_trace_file *tf = sc_create_vcd_trace_file("wave");
  sc_trace(tf, clock, "clock");
  sc_trace(tf, A,     "A");
  sc_trace(tf, B,     "B");
  sc_trace(tf, R_M,   "R_M");
  sc_trace(tf, R_T1,  "R_T1");
  sc_trace(tf, R_T2,  "R_T2");
  sc_trace(tf, R_T3,  "R_T3");
  sc_trace(tf, R_C,   "R_C");
  sc_start(80, SC_NS);
  return(0);
}


