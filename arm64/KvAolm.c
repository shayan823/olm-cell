/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__KvAolm
#define _nrn_initial _nrn_initial__KvAolm
#define nrn_cur _nrn_cur__KvAolm
#define _nrn_current _nrn_current__KvAolm
#define nrn_jacob _nrn_jacob__KvAolm
#define nrn_state _nrn_state__KvAolm
#define _net_receive _net_receive__KvAolm 
#define rates rates__KvAolm 
#define states states__KvAolm 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define gmax _p[0]
#define conductance _p[1]
#define a_instances _p[2]
#define a_timeCourse_tau _p[3]
#define a_steadyState_rate _p[4]
#define a_steadyState_midpoint _p[5]
#define a_steadyState_scale _p[6]
#define b_instances _p[7]
#define b_timeCourse_TIME_SCALE _p[8]
#define b_timeCourse_VOLT_SCALE _p[9]
#define b_steadyState_rate _p[10]
#define b_steadyState_midpoint _p[11]
#define b_steadyState_scale _p[12]
#define gion _p[13]
#define a_timeCourse_t _p[14]
#define a_steadyState_x _p[15]
#define a_rateScale _p[16]
#define a_fcond _p[17]
#define a_inf _p[18]
#define a_tauUnscaled _p[19]
#define a_tau _p[20]
#define b_timeCourse_V _p[21]
#define b_timeCourse_alpha _p[22]
#define b_timeCourse_beta _p[23]
#define b_timeCourse_t _p[24]
#define b_steadyState_x _p[25]
#define b_rateScale _p[26]
#define b_fcond _p[27]
#define b_inf _p[28]
#define b_tauUnscaled _p[29]
#define b_tau _p[30]
#define conductanceScale _p[31]
#define fopen0 _p[32]
#define fopen _p[33]
#define g _p[34]
#define a_q _p[35]
#define b_q _p[36]
#define temperature _p[37]
#define ek _p[38]
#define ik _p[39]
#define rate_a_q _p[40]
#define rate_b_q _p[41]
#define Da_q _p[42]
#define Db_q _p[43]
#define v _p[44]
#define _g _p[45]
#define _ion_ik	*_ppvar[0]._pval
#define _ion_dikdv	*_ppvar[1]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_rates(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_KvAolm", _hoc_setdata,
 "rates_KvAolm", _hoc_rates,
 0, 0
};
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "gmax_KvAolm", "S/cm2",
 "conductance_KvAolm", "uS",
 "a_timeCourse_tau_KvAolm", "ms",
 "a_steadyState_midpoint_KvAolm", "mV",
 "a_steadyState_scale_KvAolm", "mV",
 "b_timeCourse_TIME_SCALE_KvAolm", "ms",
 "b_timeCourse_VOLT_SCALE_KvAolm", "mV",
 "b_steadyState_midpoint_KvAolm", "mV",
 "b_steadyState_scale_KvAolm", "mV",
 "gion_KvAolm", "S/cm2",
 "a_timeCourse_t_KvAolm", "ms",
 "a_tauUnscaled_KvAolm", "ms",
 "a_tau_KvAolm", "ms",
 "b_timeCourse_t_KvAolm", "ms",
 "b_tauUnscaled_KvAolm", "ms",
 "b_tau_KvAolm", "ms",
 "g_KvAolm", "uS",
 0,0
};
 static double a_q0 = 0;
 static double b_q0 = 0;
 static double delta_t = 0.01;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[2]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"KvAolm",
 "gmax_KvAolm",
 "conductance_KvAolm",
 "a_instances_KvAolm",
 "a_timeCourse_tau_KvAolm",
 "a_steadyState_rate_KvAolm",
 "a_steadyState_midpoint_KvAolm",
 "a_steadyState_scale_KvAolm",
 "b_instances_KvAolm",
 "b_timeCourse_TIME_SCALE_KvAolm",
 "b_timeCourse_VOLT_SCALE_KvAolm",
 "b_steadyState_rate_KvAolm",
 "b_steadyState_midpoint_KvAolm",
 "b_steadyState_scale_KvAolm",
 0,
 "gion_KvAolm",
 "a_timeCourse_t_KvAolm",
 "a_steadyState_x_KvAolm",
 "a_rateScale_KvAolm",
 "a_fcond_KvAolm",
 "a_inf_KvAolm",
 "a_tauUnscaled_KvAolm",
 "a_tau_KvAolm",
 "b_timeCourse_V_KvAolm",
 "b_timeCourse_alpha_KvAolm",
 "b_timeCourse_beta_KvAolm",
 "b_timeCourse_t_KvAolm",
 "b_steadyState_x_KvAolm",
 "b_rateScale_KvAolm",
 "b_fcond_KvAolm",
 "b_inf_KvAolm",
 "b_tauUnscaled_KvAolm",
 "b_tau_KvAolm",
 "conductanceScale_KvAolm",
 "fopen0_KvAolm",
 "fopen_KvAolm",
 "g_KvAolm",
 0,
 "a_q_KvAolm",
 "b_q_KvAolm",
 0,
 0};
 static Symbol* _k_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 46, _prop);
 	/*initialize range parameters*/
 	gmax = 0;
 	conductance = 1e-06;
 	a_instances = 1;
 	a_timeCourse_tau = 5;
 	a_steadyState_rate = 1;
 	a_steadyState_midpoint = -14;
 	a_steadyState_scale = 16.6;
 	b_instances = 1;
 	b_timeCourse_TIME_SCALE = 1;
 	b_timeCourse_VOLT_SCALE = 1;
 	b_steadyState_rate = 1;
 	b_steadyState_midpoint = -71;
 	b_steadyState_scale = -7.3;
 	_prop->param = _p;
 	_prop->param_size = 46;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 3, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_k_sym);
 	_ppvar[0]._pval = &prop_ion->param[3]; /* ik */
 	_ppvar[1]._pval = &prop_ion->param[4]; /* _ion_dikdv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _KvAolm_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("k", 1.0);
 	_k_sym = hoc_lookup("k_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 46, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 KvAolm /Users/shayan/Desktop/CN/gsoc_cn/olm_multicompartmental/KvAolm.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "Mod file for component: Component(id=KvAolm type=ionChannelHH)";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rates(_threadargsproto_);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[2], _dlist1[2];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   rates ( _threadargs_ ) ;
   Da_q = rate_a_q ;
   Db_q = rate_b_q ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 rates ( _threadargs_ ) ;
 Da_q = Da_q  / (1. - dt*( 0.0 )) ;
 Db_q = Db_q  / (1. - dt*( 0.0 )) ;
  return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   rates ( _threadargs_ ) ;
    a_q = a_q - dt*(- ( rate_a_q ) ) ;
    b_q = b_q - dt*(- ( rate_b_q ) ) ;
   }
  return 0;
}
 
static int  rates ( _threadargsproto_ ) {
   a_timeCourse_t = a_timeCourse_tau ;
   a_steadyState_x = a_steadyState_rate / ( 1.0 + exp ( 0.0 - ( v - a_steadyState_midpoint ) / a_steadyState_scale ) ) ;
   a_rateScale = 1.0 ;
   a_fcond = pow( a_q , a_instances ) ;
   a_inf = a_steadyState_x ;
   a_tauUnscaled = a_timeCourse_t ;
   a_tau = a_tauUnscaled / a_rateScale ;
   b_timeCourse_V = v / b_timeCourse_VOLT_SCALE ;
   b_timeCourse_alpha = 0.000009 / exp ( ( b_timeCourse_V - 26.0 ) / 18.5 ) ;
   b_timeCourse_beta = 0.014 / ( exp ( ( b_timeCourse_V + 70.0 ) / - 11.0 ) + 0.2 ) ;
   b_timeCourse_t = ( 1.0 / ( b_timeCourse_alpha + b_timeCourse_beta ) ) * b_timeCourse_TIME_SCALE ;
   b_steadyState_x = b_steadyState_rate / ( 1.0 + exp ( 0.0 - ( v - b_steadyState_midpoint ) / b_steadyState_scale ) ) ;
   b_rateScale = 1.0 ;
   b_fcond = pow( b_q , b_instances ) ;
   b_inf = b_steadyState_x ;
   b_tauUnscaled = b_timeCourse_t ;
   b_tau = b_tauUnscaled / b_rateScale ;
   rate_a_q = ( a_inf - a_q ) / a_tau ;
   rate_b_q = ( b_inf - b_q ) / b_tau ;
    return 0; }
 
static void _hoc_rates(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r = 1.;
 rates ( _p, _ppvar, _thread, _nt );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 2;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
  }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 2; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_k_sym, _ppvar, 0, 3);
   nrn_update_ion_pointer(_k_sym, _ppvar, 1, 4);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  a_q = a_q0;
  b_q = b_q0;
 {
   ek = - 77.0 ;
   temperature = celsius + 273.15 ;
   rates ( _threadargs_ ) ;
   rates ( _threadargs_ ) ;
   a_q = a_inf ;
   b_q = b_inf ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
 }
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   conductanceScale = 1.0 ;
   fopen0 = a_fcond * b_fcond ;
   fopen = conductanceScale * fopen0 ;
   g = conductance * fopen ;
   gion = gmax * fopen ;
   ik = gion * ( v - ek ) ;
   }
 _current += ik;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ double _dik;
  _dik = ik;
 _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
  _ion_dikdv += (_dik - ik)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ik += ik ;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 {   states(_p, _ppvar, _thread, _nt);
  } }}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(a_q) - _p;  _dlist1[0] = &(Da_q) - _p;
 _slist1[1] = &(b_q) - _p;  _dlist1[1] = &(Db_q) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "/Users/shayan/Desktop/CN/gsoc_cn/olm_multicompartmental/KvAolm.mod";
static const char* nmodl_file_text = 
  "TITLE Mod file for component: Component(id=KvAolm type=ionChannelHH)\n"
  "\n"
  "COMMENT\n"
  "\n"
  "    This NEURON file has been generated by org.neuroml.export (see https://github.com/NeuroML/org.neuroml.export)\n"
  "         org.neuroml.export  v1.8.1\n"
  "         org.neuroml.model   v1.8.1\n"
  "         jLEMS               v0.10.6\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  "NEURON {\n"
  "    SUFFIX KvAolm\n"
  "    USEION k WRITE ik VALENCE 1 ? Assuming valence = 1; TODO check this!!\n"
  "    \n"
  "    RANGE gion                           \n"
  "    RANGE gmax                              : Will be changed when ion channel mechanism placed on cell!\n"
  "    RANGE conductance                       : parameter\n"
  "    \n"
  "    RANGE g                                 : exposure\n"
  "    \n"
  "    RANGE fopen                             : exposure\n"
  "    RANGE a_instances                       : parameter\n"
  "    \n"
  "    RANGE a_tau                             : exposure\n"
  "    \n"
  "    RANGE a_inf                             : exposure\n"
  "    \n"
  "    RANGE a_rateScale                       : exposure\n"
  "    \n"
  "    RANGE a_fcond                           : exposure\n"
  "    RANGE a_timeCourse_tau                  : parameter\n"
  "    \n"
  "    RANGE a_timeCourse_t                    : exposure\n"
  "    RANGE a_steadyState_rate                : parameter\n"
  "    RANGE a_steadyState_midpoint            : parameter\n"
  "    RANGE a_steadyState_scale               : parameter\n"
  "    \n"
  "    RANGE a_steadyState_x                   : exposure\n"
  "    RANGE b_instances                       : parameter\n"
  "    \n"
  "    RANGE b_tau                             : exposure\n"
  "    \n"
  "    RANGE b_inf                             : exposure\n"
  "    \n"
  "    RANGE b_rateScale                       : exposure\n"
  "    \n"
  "    RANGE b_fcond                           : exposure\n"
  "    RANGE b_timeCourse_TIME_SCALE           : parameter\n"
  "    RANGE b_timeCourse_VOLT_SCALE           : parameter\n"
  "    \n"
  "    RANGE b_timeCourse_t                    : exposure\n"
  "    RANGE b_steadyState_rate                : parameter\n"
  "    RANGE b_steadyState_midpoint            : parameter\n"
  "    RANGE b_steadyState_scale               : parameter\n"
  "    \n"
  "    RANGE b_steadyState_x                   : exposure\n"
  "    RANGE a_tauUnscaled                     : derived variable\n"
  "    RANGE b_timeCourse_V                    : derived variable\n"
  "    RANGE b_timeCourse_alpha                : derived variable\n"
  "    RANGE b_timeCourse_beta                 : derived variable\n"
  "    RANGE b_tauUnscaled                     : derived variable\n"
  "    RANGE conductanceScale                  : derived variable\n"
  "    RANGE fopen0                            : derived variable\n"
  "    \n"
  "}\n"
  "\n"
  "UNITS {\n"
  "    \n"
  "    (nA) = (nanoamp)\n"
  "    (uA) = (microamp)\n"
  "    (mA) = (milliamp)\n"
  "    (A) = (amp)\n"
  "    (mV) = (millivolt)\n"
  "    (mS) = (millisiemens)\n"
  "    (uS) = (microsiemens)\n"
  "    (molar) = (1/liter)\n"
  "    (kHz) = (kilohertz)\n"
  "    (mM) = (millimolar)\n"
  "    (um) = (micrometer)\n"
  "    (umol) = (micromole)\n"
  "    (S) = (siemens)\n"
  "    \n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "    \n"
  "    gmax = 0  (S/cm2)                       : Will be changed when ion channel mechanism placed on cell!\n"
  "    \n"
  "    conductance = 1.0E-6 (uS)\n"
  "    a_instances = 1 \n"
  "    a_timeCourse_tau = 5 (ms)\n"
  "    a_steadyState_rate = 1 \n"
  "    a_steadyState_midpoint = -14 (mV)\n"
  "    a_steadyState_scale = 16.6 (mV)\n"
  "    b_instances = 1 \n"
  "    b_timeCourse_TIME_SCALE = 1 (ms)\n"
  "    b_timeCourse_VOLT_SCALE = 1 (mV)\n"
  "    b_steadyState_rate = 1 \n"
  "    b_steadyState_midpoint = -71 (mV)\n"
  "    b_steadyState_scale = -7.3 (mV)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "    \n"
  "    gion   (S/cm2)                          : Transient conductance density of the channel? Standard Assigned variables with ionChannel\n"
  "    v (mV)\n"
  "    celsius (degC)\n"
  "    temperature (K)\n"
  "    ek (mV)\n"
  "    ik (mA/cm2)\n"
  "    \n"
  "    \n"
  "    a_timeCourse_t (ms)                    : derived variable\n"
  "    \n"
  "    a_steadyState_x                        : derived variable\n"
  "    \n"
  "    a_rateScale                            : derived variable\n"
  "    \n"
  "    a_fcond                                : derived variable\n"
  "    \n"
  "    a_inf                                  : derived variable\n"
  "    \n"
  "    a_tauUnscaled (ms)                     : derived variable\n"
  "    \n"
  "    a_tau (ms)                             : derived variable\n"
  "    \n"
  "    b_timeCourse_V                         : derived variable\n"
  "    \n"
  "    b_timeCourse_alpha                     : derived variable\n"
  "    \n"
  "    b_timeCourse_beta                      : derived variable\n"
  "    \n"
  "    b_timeCourse_t (ms)                    : derived variable\n"
  "    \n"
  "    b_steadyState_x                        : derived variable\n"
  "    \n"
  "    b_rateScale                            : derived variable\n"
  "    \n"
  "    b_fcond                                : derived variable\n"
  "    \n"
  "    b_inf                                  : derived variable\n"
  "    \n"
  "    b_tauUnscaled (ms)                     : derived variable\n"
  "    \n"
  "    b_tau (ms)                             : derived variable\n"
  "    \n"
  "    conductanceScale                       : derived variable\n"
  "    \n"
  "    fopen0                                 : derived variable\n"
  "    \n"
  "    fopen                                  : derived variable\n"
  "    \n"
  "    g (uS)                                 : derived variable\n"
  "    rate_a_q (/ms)\n"
  "    rate_b_q (/ms)\n"
  "    \n"
  "}\n"
  "\n"
  "STATE {\n"
  "    a_q  \n"
  "    b_q  \n"
  "    \n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "    ek = -77.0\n"
  "    \n"
  "    temperature = celsius + 273.15\n"
  "    \n"
  "    rates()\n"
  "    rates() ? To ensure correct initialisation.\n"
  "    \n"
  "    a_q = a_inf\n"
  "    \n"
  "    b_q = b_inf\n"
  "    \n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "    \n"
  "    SOLVE states METHOD cnexp\n"
  "    \n"
  "    ? DerivedVariable is based on path: conductanceScaling[*]/factor, on: Component(id=KvAolm type=ionChannelHH), from conductanceScaling; null\n"
  "    ? Path not present in component, using factor: 1\n"
  "    \n"
  "    conductanceScale = 1 \n"
  "    \n"
  "    ? DerivedVariable is based on path: gates[*]/fcond, on: Component(id=KvAolm type=ionChannelHH), from gates; Component(id=a type=gateHHtauInf)\n"
  "    ? multiply applied to all instances of fcond in: <gates> ([Component(id=a type=gateHHtauInf), Component(id=b type=gateHHtauInf)]))\n"
  "    fopen0 = a_fcond * b_fcond ? path based, prefix = \n"
  "    \n"
  "    fopen = conductanceScale  *  fopen0 ? evaluable\n"
  "    g = conductance  *  fopen ? evaluable\n"
  "    gion = gmax * fopen \n"
  "    \n"
  "    ik = gion * (v - ek)\n"
  "    \n"
  "}\n"
  "\n"
  "DERIVATIVE states {\n"
  "    rates()\n"
  "    a_q' = rate_a_q \n"
  "    b_q' = rate_b_q \n"
  "    \n"
  "}\n"
  "\n"
  "PROCEDURE rates() {\n"
  "    \n"
  "    a_timeCourse_t = a_timeCourse_tau ? evaluable\n"
  "    a_steadyState_x = a_steadyState_rate  / (1 + exp(0 - (v -  a_steadyState_midpoint )/ a_steadyState_scale )) ? evaluable\n"
  "    ? DerivedVariable is based on path: q10Settings[*]/q10, on: Component(id=a type=gateHHtauInf), from q10Settings; null\n"
  "    ? Path not present in component, using factor: 1\n"
  "    \n"
  "    a_rateScale = 1 \n"
  "    \n"
  "    a_fcond = a_q ^ a_instances ? evaluable\n"
  "    ? DerivedVariable is based on path: steadyState/x, on: Component(id=a type=gateHHtauInf), from steadyState; Component(id=null type=HHSigmoidVariable)\n"
  "    a_inf = a_steadyState_x ? path based, prefix = a_\n"
  "    \n"
  "    ? DerivedVariable is based on path: timeCourse/t, on: Component(id=a type=gateHHtauInf), from timeCourse; Component(id=null type=fixedTimeCourse)\n"
  "    a_tauUnscaled = a_timeCourse_t ? path based, prefix = a_\n"
  "    \n"
  "    a_tau = a_tauUnscaled  /  a_rateScale ? evaluable\n"
  "    b_timeCourse_V = v /  b_timeCourse_VOLT_SCALE ? evaluable\n"
  "    b_timeCourse_alpha = 0.000009 / exp(( b_timeCourse_V -26)/18.5) ? evaluable\n"
  "    b_timeCourse_beta = 0.014 / (exp(( b_timeCourse_V +70)/-11) + 0.2) ? evaluable\n"
  "    b_timeCourse_t = ( 1 / ( b_timeCourse_alpha  +  b_timeCourse_beta ) ) *  b_timeCourse_TIME_SCALE ? evaluable\n"
  "    b_steadyState_x = b_steadyState_rate  / (1 + exp(0 - (v -  b_steadyState_midpoint )/ b_steadyState_scale )) ? evaluable\n"
  "    ? DerivedVariable is based on path: q10Settings[*]/q10, on: Component(id=b type=gateHHtauInf), from q10Settings; null\n"
  "    ? Path not present in component, using factor: 1\n"
  "    \n"
  "    b_rateScale = 1 \n"
  "    \n"
  "    b_fcond = b_q ^ b_instances ? evaluable\n"
  "    ? DerivedVariable is based on path: steadyState/x, on: Component(id=b type=gateHHtauInf), from steadyState; Component(id=null type=HHSigmoidVariable)\n"
  "    b_inf = b_steadyState_x ? path based, prefix = b_\n"
  "    \n"
  "    ? DerivedVariable is based on path: timeCourse/t, on: Component(id=b type=gateHHtauInf), from timeCourse; Component(id=null type=Bezaire_KvAolm_taub)\n"
  "    b_tauUnscaled = b_timeCourse_t ? path based, prefix = b_\n"
  "    \n"
  "    b_tau = b_tauUnscaled  /  b_rateScale ? evaluable\n"
  "    \n"
  "     \n"
  "    rate_a_q = ( a_inf  -  a_q ) /  a_tau ? Note units of all quantities used here need to be consistent!\n"
  "    \n"
  "     \n"
  "    \n"
  "     \n"
  "    \n"
  "     \n"
  "    rate_b_q = ( b_inf  -  b_q ) /  b_tau ? Note units of all quantities used here need to be consistent!\n"
  "    \n"
  "     \n"
  "    \n"
  "     \n"
  "    \n"
  "     \n"
  "    \n"
  "}\n"
  "\n"
  ;
#endif
