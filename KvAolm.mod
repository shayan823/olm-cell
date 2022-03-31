TITLE Mod file for component: Component(id=KvAolm type=ionChannelHH)

COMMENT

    This NEURON file has been generated by org.neuroml.export (see https://github.com/NeuroML/org.neuroml.export)
         org.neuroml.export  v1.8.1
         org.neuroml.model   v1.8.1
         jLEMS               v0.10.6

ENDCOMMENT

NEURON {
    SUFFIX KvAolm
    USEION k WRITE ik VALENCE 1 ? Assuming valence = 1; TODO check this!!
    
    RANGE gion                           
    RANGE gmax                              : Will be changed when ion channel mechanism placed on cell!
    RANGE conductance                       : parameter
    
    RANGE g                                 : exposure
    
    RANGE fopen                             : exposure
    RANGE a_instances                       : parameter
    
    RANGE a_tau                             : exposure
    
    RANGE a_inf                             : exposure
    
    RANGE a_rateScale                       : exposure
    
    RANGE a_fcond                           : exposure
    RANGE a_timeCourse_tau                  : parameter
    
    RANGE a_timeCourse_t                    : exposure
    RANGE a_steadyState_rate                : parameter
    RANGE a_steadyState_midpoint            : parameter
    RANGE a_steadyState_scale               : parameter
    
    RANGE a_steadyState_x                   : exposure
    RANGE b_instances                       : parameter
    
    RANGE b_tau                             : exposure
    
    RANGE b_inf                             : exposure
    
    RANGE b_rateScale                       : exposure
    
    RANGE b_fcond                           : exposure
    RANGE b_timeCourse_TIME_SCALE           : parameter
    RANGE b_timeCourse_VOLT_SCALE           : parameter
    
    RANGE b_timeCourse_t                    : exposure
    RANGE b_steadyState_rate                : parameter
    RANGE b_steadyState_midpoint            : parameter
    RANGE b_steadyState_scale               : parameter
    
    RANGE b_steadyState_x                   : exposure
    RANGE a_tauUnscaled                     : derived variable
    RANGE b_timeCourse_V                    : derived variable
    RANGE b_timeCourse_alpha                : derived variable
    RANGE b_timeCourse_beta                 : derived variable
    RANGE b_tauUnscaled                     : derived variable
    RANGE conductanceScale                  : derived variable
    RANGE fopen0                            : derived variable
    
}

UNITS {
    
    (nA) = (nanoamp)
    (uA) = (microamp)
    (mA) = (milliamp)
    (A) = (amp)
    (mV) = (millivolt)
    (mS) = (millisiemens)
    (uS) = (microsiemens)
    (molar) = (1/liter)
    (kHz) = (kilohertz)
    (mM) = (millimolar)
    (um) = (micrometer)
    (umol) = (micromole)
    (S) = (siemens)
    
}

PARAMETER {
    
    gmax = 0  (S/cm2)                       : Will be changed when ion channel mechanism placed on cell!
    
    conductance = 1.0E-6 (uS)
    a_instances = 1 
    a_timeCourse_tau = 5 (ms)
    a_steadyState_rate = 1 
    a_steadyState_midpoint = -14 (mV)
    a_steadyState_scale = 16.6 (mV)
    b_instances = 1 
    b_timeCourse_TIME_SCALE = 1 (ms)
    b_timeCourse_VOLT_SCALE = 1 (mV)
    b_steadyState_rate = 1 
    b_steadyState_midpoint = -71 (mV)
    b_steadyState_scale = -7.3 (mV)
}

ASSIGNED {
    
    gion   (S/cm2)                          : Transient conductance density of the channel? Standard Assigned variables with ionChannel
    v (mV)
    celsius (degC)
    temperature (K)
    ek (mV)
    ik (mA/cm2)
    
    
    a_timeCourse_t (ms)                    : derived variable
    
    a_steadyState_x                        : derived variable
    
    a_rateScale                            : derived variable
    
    a_fcond                                : derived variable
    
    a_inf                                  : derived variable
    
    a_tauUnscaled (ms)                     : derived variable
    
    a_tau (ms)                             : derived variable
    
    b_timeCourse_V                         : derived variable
    
    b_timeCourse_alpha                     : derived variable
    
    b_timeCourse_beta                      : derived variable
    
    b_timeCourse_t (ms)                    : derived variable
    
    b_steadyState_x                        : derived variable
    
    b_rateScale                            : derived variable
    
    b_fcond                                : derived variable
    
    b_inf                                  : derived variable
    
    b_tauUnscaled (ms)                     : derived variable
    
    b_tau (ms)                             : derived variable
    
    conductanceScale                       : derived variable
    
    fopen0                                 : derived variable
    
    fopen                                  : derived variable
    
    g (uS)                                 : derived variable
    rate_a_q (/ms)
    rate_b_q (/ms)
    
}

STATE {
    a_q  
    b_q  
    
}

INITIAL {
    ek = -77.0
    
    temperature = celsius + 273.15
    
    rates()
    rates() ? To ensure correct initialisation.
    
    a_q = a_inf
    
    b_q = b_inf
    
}

BREAKPOINT {
    
    SOLVE states METHOD cnexp
    
    ? DerivedVariable is based on path: conductanceScaling[*]/factor, on: Component(id=KvAolm type=ionChannelHH), from conductanceScaling; null
    ? Path not present in component, using factor: 1
    
    conductanceScale = 1 
    
    ? DerivedVariable is based on path: gates[*]/fcond, on: Component(id=KvAolm type=ionChannelHH), from gates; Component(id=a type=gateHHtauInf)
    ? multiply applied to all instances of fcond in: <gates> ([Component(id=a type=gateHHtauInf), Component(id=b type=gateHHtauInf)]))
    fopen0 = a_fcond * b_fcond ? path based, prefix = 
    
    fopen = conductanceScale  *  fopen0 ? evaluable
    g = conductance  *  fopen ? evaluable
    gion = gmax * fopen 
    
    ik = gion * (v - ek)
    
}

DERIVATIVE states {
    rates()
    a_q' = rate_a_q 
    b_q' = rate_b_q 
    
}

PROCEDURE rates() {
    
    a_timeCourse_t = a_timeCourse_tau ? evaluable
    a_steadyState_x = a_steadyState_rate  / (1 + exp(0 - (v -  a_steadyState_midpoint )/ a_steadyState_scale )) ? evaluable
    ? DerivedVariable is based on path: q10Settings[*]/q10, on: Component(id=a type=gateHHtauInf), from q10Settings; null
    ? Path not present in component, using factor: 1
    
    a_rateScale = 1 
    
    a_fcond = a_q ^ a_instances ? evaluable
    ? DerivedVariable is based on path: steadyState/x, on: Component(id=a type=gateHHtauInf), from steadyState; Component(id=null type=HHSigmoidVariable)
    a_inf = a_steadyState_x ? path based, prefix = a_
    
    ? DerivedVariable is based on path: timeCourse/t, on: Component(id=a type=gateHHtauInf), from timeCourse; Component(id=null type=fixedTimeCourse)
    a_tauUnscaled = a_timeCourse_t ? path based, prefix = a_
    
    a_tau = a_tauUnscaled  /  a_rateScale ? evaluable
    b_timeCourse_V = v /  b_timeCourse_VOLT_SCALE ? evaluable
    b_timeCourse_alpha = 0.000009 / exp(( b_timeCourse_V -26)/18.5) ? evaluable
    b_timeCourse_beta = 0.014 / (exp(( b_timeCourse_V +70)/-11) + 0.2) ? evaluable
    b_timeCourse_t = ( 1 / ( b_timeCourse_alpha  +  b_timeCourse_beta ) ) *  b_timeCourse_TIME_SCALE ? evaluable
    b_steadyState_x = b_steadyState_rate  / (1 + exp(0 - (v -  b_steadyState_midpoint )/ b_steadyState_scale )) ? evaluable
    ? DerivedVariable is based on path: q10Settings[*]/q10, on: Component(id=b type=gateHHtauInf), from q10Settings; null
    ? Path not present in component, using factor: 1
    
    b_rateScale = 1 
    
    b_fcond = b_q ^ b_instances ? evaluable
    ? DerivedVariable is based on path: steadyState/x, on: Component(id=b type=gateHHtauInf), from steadyState; Component(id=null type=HHSigmoidVariable)
    b_inf = b_steadyState_x ? path based, prefix = b_
    
    ? DerivedVariable is based on path: timeCourse/t, on: Component(id=b type=gateHHtauInf), from timeCourse; Component(id=null type=Bezaire_KvAolm_taub)
    b_tauUnscaled = b_timeCourse_t ? path based, prefix = b_
    
    b_tau = b_tauUnscaled  /  b_rateScale ? evaluable
    
     
    rate_a_q = ( a_inf  -  a_q ) /  a_tau ? Note units of all quantities used here need to be consistent!
    
     
    
     
    
     
    rate_b_q = ( b_inf  -  b_q ) /  b_tau ? Note units of all quantities used here need to be consistent!
    
     
    
     
    
     
    
}

