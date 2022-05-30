// main.cpp
#include "systemc.h"

SC_MODULE(fibonacci) {
  sc_in<short> num;
  sc_fifo<int> f1, f2;

  void do_it(void) {
    int sum = 0;

    // Put initial value of f(n-1) loop
    f1.write(0);
    // Put initial values of f(n-2) loop
    f2.write(0); f2.write(1);

    // Go through the sequence based on the number of iterations
    for(sc_int<8> i=0; i < num; i++) {
      // Consume the FIFOs and find the sum of the loops
      sum = f1.read() + f2.read();
      // Send sum to console
      cout << "Fn(" << i << ") = " << sum << endl;
      // Write the sum to the loop buffers
      f1.write(sum); f2.write(sum);
    }
  }

  SC_CTOR(fibonacci) : f1(2), f2(3) {
    SC_THREAD(do_it);
    sensitive << num;
    dont_initialize();
  }
};

// Module to generate the number of iterations
SC_MODULE (generator) {
  sc_out<short>  sig;
 
  void do_it(void) {
    sig.write(20);
  }
 
  SC_CTOR(generator) {
    SC_THREAD(do_it);
    sig.initialize(0);
  }
};

int sc_main(int argc, char *argv[]) {
  sc_signal<short> A;
  sc_set_time_resolution(1, SC_NS); // V2.0
  sc_set_default_time_unit(1, SC_NS);
  //sc_clock clock("clock", 20, SC_NS, 0.5, 15, SC_NS);

  generator GEN("GEN");
  GEN.sig(A);

  fibonacci f_inst("f_inst");
  f_inst.num(A);

  // trace file creation
  sc_trace_file *tf = sc_create_vcd_trace_file("wave");
  sc_start(80, SC_NS);
  return(0);
}


