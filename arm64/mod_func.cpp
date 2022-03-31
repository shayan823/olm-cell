#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;
#if defined(__cplusplus)
extern "C" {
#endif

extern void _HCNolm_reg(void);
extern void _Kdrfast_reg(void);
extern void _KvAolm_reg(void);
extern void _Nav_reg(void);
extern void _leak_chan_reg(void);
extern void _pg_olm_reg(void);

void modl_reg(){
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");

    fprintf(stderr," \"HCNolm.mod\"");
    fprintf(stderr," \"Kdrfast.mod\"");
    fprintf(stderr," \"KvAolm.mod\"");
    fprintf(stderr," \"Nav.mod\"");
    fprintf(stderr," \"leak_chan.mod\"");
    fprintf(stderr," \"pg_olm.mod\"");
    fprintf(stderr, "\n");
  }
  _HCNolm_reg();
  _Kdrfast_reg();
  _KvAolm_reg();
  _Nav_reg();
  _leak_chan_reg();
  _pg_olm_reg();
}

#if defined(__cplusplus)
}
#endif
