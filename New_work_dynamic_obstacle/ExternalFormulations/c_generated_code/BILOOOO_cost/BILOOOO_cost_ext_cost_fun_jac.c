/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) BILOOOO_cost_ext_cost_fun_jac_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s4[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

/* BILOOOO_cost_ext_cost_fun_jac:(i0[10],i1[4],i2[],i3[10])->(o0,o1[14]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a5, a6, a7, a8, a9;
  a0=1000.;
  a1=arg[0]? arg[0][0] : 0;
  a2=arg[3]? arg[3][0] : 0;
  a3=(a1-a2);
  a3=(a0*a3);
  a2=(a1-a2);
  a4=(a3*a2);
  a5=arg[0]? arg[0][1] : 0;
  a6=arg[3]? arg[3][1] : 0;
  a7=(a5-a6);
  a7=(a0*a7);
  a6=(a5-a6);
  a8=(a7*a6);
  a4=(a4+a8);
  a8=1.0000000000000000e-002;
  a9=arg[0]? arg[0][2] : 0;
  a10=arg[3]? arg[3][2] : 0;
  a11=(a9-a10);
  a11=(a8*a11);
  a9=(a9-a10);
  a10=(a11*a9);
  a4=(a4+a10);
  a10=arg[0]? arg[0][3] : 0;
  a12=arg[3]? arg[3][3] : 0;
  a13=(a10-a12);
  a13=(a8*a13);
  a10=(a10-a12);
  a12=(a13*a10);
  a4=(a4+a12);
  a12=1.0000000000000001e-005;
  a14=arg[0]? arg[0][4] : 0;
  a15=arg[3]? arg[3][4] : 0;
  a16=(a14-a15);
  a16=(a12*a16);
  a14=(a14-a15);
  a15=(a16*a14);
  a4=(a4+a15);
  a15=arg[0]? arg[0][5] : 0;
  a17=arg[3]? arg[3][5] : 0;
  a18=(a15-a17);
  a18=(a0*a18);
  a17=(a15-a17);
  a19=(a18*a17);
  a4=(a4+a19);
  a19=10.;
  a20=arg[0]? arg[0][6] : 0;
  a21=arg[3]? arg[3][6] : 0;
  a22=(a20-a21);
  a22=(a19*a22);
  a21=(a20-a21);
  a23=(a22*a21);
  a4=(a4+a23);
  a23=arg[0]? arg[0][7] : 0;
  a24=arg[3]? arg[3][7] : 0;
  a25=(a23-a24);
  a25=(a8*a25);
  a23=(a23-a24);
  a24=(a25*a23);
  a4=(a4+a24);
  a24=arg[0]? arg[0][8] : 0;
  a26=arg[3]? arg[3][8] : 0;
  a27=(a24-a26);
  a27=(a8*a27);
  a24=(a24-a26);
  a26=(a27*a24);
  a4=(a4+a26);
  a26=1.0000000000000000e-010;
  a28=arg[0]? arg[0][9] : 0;
  a29=arg[3]? arg[3][9] : 0;
  a30=(a28-a29);
  a30=(a26*a30);
  a28=(a28-a29);
  a29=(a30*a28);
  a4=(a4+a29);
  a29=9.4999999999999996e-001;
  a1=(a1-a15);
  a15=casadi_sq(a1);
  a5=(a5-a20);
  a20=casadi_sq(a5);
  a15=(a15+a20);
  a15=(a29*a15);
  a20=(1./a15);
  a4=(a4+a20);
  a31=5.;
  a32=arg[1]? arg[1][0] : 0;
  a33=(a31*a32);
  a34=(a33*a32);
  a35=1.0000000000000001e-001;
  a36=arg[1]? arg[1][1] : 0;
  a37=(a35*a36);
  a38=(a37*a36);
  a34=(a34+a38);
  a38=arg[1]? arg[1][2] : 0;
  a39=(a35*a38);
  a40=(a39*a38);
  a34=(a34+a40);
  a40=arg[1]? arg[1][3] : 0;
  a41=(a35*a40);
  a42=(a41*a40);
  a34=(a34+a42);
  a4=(a4+a34);
  if (res[0]!=0) res[0][0]=a4;
  a31=(a31*a32);
  a33=(a33+a31);
  if (res[1]!=0) res[1][0]=a33;
  a36=(a35*a36);
  a37=(a37+a36);
  if (res[1]!=0) res[1][1]=a37;
  a38=(a35*a38);
  a39=(a39+a38);
  if (res[1]!=0) res[1][2]=a39;
  a35=(a35*a40);
  a41=(a41+a35);
  if (res[1]!=0) res[1][3]=a41;
  a1=(a1+a1);
  a20=(a20/a15);
  a29=(a29*a20);
  a1=(a1*a29);
  a3=(a3-a1);
  a2=(a0*a2);
  a3=(a3+a2);
  if (res[1]!=0) res[1][4]=a3;
  a5=(a5+a5);
  a5=(a5*a29);
  a7=(a7-a5);
  a6=(a0*a6);
  a7=(a7+a6);
  if (res[1]!=0) res[1][5]=a7;
  a9=(a8*a9);
  a11=(a11+a9);
  if (res[1]!=0) res[1][6]=a11;
  a10=(a8*a10);
  a13=(a13+a10);
  if (res[1]!=0) res[1][7]=a13;
  a12=(a12*a14);
  a16=(a16+a12);
  if (res[1]!=0) res[1][8]=a16;
  a1=(a1+a18);
  a0=(a0*a17);
  a1=(a1+a0);
  if (res[1]!=0) res[1][9]=a1;
  a5=(a5+a22);
  a19=(a19*a21);
  a5=(a5+a19);
  if (res[1]!=0) res[1][10]=a5;
  a23=(a8*a23);
  a25=(a25+a23);
  if (res[1]!=0) res[1][11]=a25;
  a8=(a8*a24);
  a27=(a27+a8);
  if (res[1]!=0) res[1][12]=a27;
  a26=(a26*a28);
  a30=(a30+a26);
  if (res[1]!=0) res[1][13]=a30;
  return 0;
}

CASADI_SYMBOL_EXPORT int BILOOOO_cost_ext_cost_fun_jac(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int BILOOOO_cost_ext_cost_fun_jac_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int BILOOOO_cost_ext_cost_fun_jac_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void BILOOOO_cost_ext_cost_fun_jac_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int BILOOOO_cost_ext_cost_fun_jac_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void BILOOOO_cost_ext_cost_fun_jac_release(int mem) {
}

CASADI_SYMBOL_EXPORT void BILOOOO_cost_ext_cost_fun_jac_incref(void) {
}

CASADI_SYMBOL_EXPORT void BILOOOO_cost_ext_cost_fun_jac_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int BILOOOO_cost_ext_cost_fun_jac_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int BILOOOO_cost_ext_cost_fun_jac_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real BILOOOO_cost_ext_cost_fun_jac_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* BILOOOO_cost_ext_cost_fun_jac_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* BILOOOO_cost_ext_cost_fun_jac_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* BILOOOO_cost_ext_cost_fun_jac_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* BILOOOO_cost_ext_cost_fun_jac_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int BILOOOO_cost_ext_cost_fun_jac_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif