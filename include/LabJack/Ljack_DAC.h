#ifndef __LJACK_DAC__
#define __LJACK_DAC__

#include "macros.h"
#include "msleep.h"

#include "Ljack.h"
// provisional
#include "Ljack_DAC_plugin.h"



using namespace std;




class Ljack_DAC {

 private:

  u3TdacCalibrationInfo caliInfo;
  Ljack *host_labjack;
  HANDLE host_handle;
  // true if connected to free slot and calibrated OK
  bool connected;
  Ljack_DAC_plugin DAC_internal;

  
  // outputs a (coerced to) proper voltage on current exit
  void set_DAC_output(float voltage);


  // simulation
  bool simulate_p;
  size_t latency_microsecs; // useful for simulating labjack wait


 public:

  
  Ljack_DAC(Ljack &host_labjack)
    {
      connected = false;
      this -> host_labjack = &host_labjack;
      host_handle = host_labjack.get_handle();
      
      simulate_p = host_labjack.simulate_p; 
      if (! simulate_p)
	{
	  MSG_ASSERT(configIO_DAC(host_handle) == 0, \
		     "DAC config failure");
	}
      else
	latency_microsecs = host_labjack.latency_microsecs;
    }

  // connect current DAC expansion to specified slot of the labjack host
  // returns false if slot occupied or initialization failed
  bool connect(enum LjackSlot freeslot);

  // requires established connection
  // voltage will be coerced to [-10; +10v]
  void set_A_output(float voltage);
  void set_B_output(float voltage);

  // Failed writes so far (USB transfer or an I2C error code): the library counts them here and otherwise only prints.
  // A write failed when this rose across it. Inline and no new member: the compiled library's layout is unchanged.
  // The simulation never calls the library, which then may not initialize the count: 0.
  unsigned int communication_errors() const { return simulate_p ? 0u : DAC_internal.communication_errors; }

  // Whether the LJTick-DAC acknowledged every byte of the last output (B206). The library compares the I2C ack bits with
  // the expected 2^(bytes sent + 1) - 1 and only prints "expected an ack"; it does not count a NACK (e.g. a loose tick on
  // a working U3). Read after each output; stale after a failed USB transfer, which the count above covers. True in
  // simulation (no output).
  bool last_output_acknowledged() const {
    if (simulate_p) return true;
    const unsigned ack = unsigned(DAC_internal.ackArray[0]) | (unsigned(DAC_internal.ackArray[1]) << 8) |
                         (unsigned(DAC_internal.ackArray[2]) << 16) | (unsigned(DAC_internal.ackArray[3]) << 24);
    return ack == (1u << (DAC_internal.numBytesToSend + 1)) - 1u;
  }

};


#endif // __LJACK_DAC__
