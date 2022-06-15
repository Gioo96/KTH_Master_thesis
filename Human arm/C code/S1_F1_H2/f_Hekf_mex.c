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
  #define CASADI_PREFIX(ID) f_Hekf_mex_ ## ID
#endif

#include <math.h>
#include <stdio.h>
#include <string.h>
#ifdef MATLAB_MEX_FILE
#include <mex.h>
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_from_mex CASADI_PREFIX(from_mex)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_to_mex CASADI_PREFIX(to_mex)

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

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

#ifdef MATLAB_MEX_FILE
casadi_real* casadi_from_mex(const mxArray* p, casadi_real* y, const casadi_int* sp, casadi_real* w) {
  casadi_int nrow, ncol, is_sparse, c, k, p_nrow, p_ncol;
  const casadi_int *colind, *row;
  mwIndex *Jc, *Ir;
  const double* p_data;
  if (!mxIsDouble(p) || mxGetNumberOfDimensions(p)!=2)
    mexErrMsgIdAndTxt("Casadi:RuntimeError",
      "\"from_mex\" failed: Not a two-dimensional matrix of double precision.");
  nrow = *sp++;
  ncol = *sp++;
  colind = sp;
  row = sp+ncol+1;
  p_nrow = mxGetM(p);
  p_ncol = mxGetN(p);
  is_sparse = mxIsSparse(p);
  Jc = 0;
  Ir = 0;
  if (is_sparse) {
    Jc = mxGetJc(p);
    Ir = mxGetIr(p);
  }
  p_data = (const double*)mxGetData(p);
  if (p_nrow==1 && p_ncol==1) {
    casadi_int nnz;
    double v = is_sparse && Jc[1]==0 ? 0 : *p_data;
    nnz = sp[ncol];
    casadi_fill(y, nnz, v);
  } else {
    casadi_int tr = 0;
    if (nrow!=p_nrow || ncol!=p_ncol) {
      tr = nrow==p_ncol && ncol==p_nrow && (nrow==1 || ncol==1);
      if (!tr) mexErrMsgIdAndTxt("Casadi:RuntimeError",
                                 "\"from_mex\" failed: Dimension mismatch. "
                                 "Expected %d-by-%d, got %d-by-%d instead.",
                                 nrow, ncol, p_nrow, p_ncol);
    }
    if (is_sparse) {
      if (tr) {
        for (c=0; c<ncol; ++c)
          for (k=colind[c]; k<colind[c+1]; ++k) w[row[k]+c*nrow]=0;
        for (c=0; c<p_ncol; ++c)
          for (k=Jc[c]; k<(casadi_int) Jc[c+1]; ++k) w[c+Ir[k]*p_ncol] = p_data[k];
        for (c=0; c<ncol; ++c)
          for (k=colind[c]; k<colind[c+1]; ++k) y[k] = w[row[k]+c*nrow];
      } else {
        for (c=0; c<ncol; ++c) {
          for (k=colind[c]; k<colind[c+1]; ++k) w[row[k]]=0;
          for (k=Jc[c]; k<(casadi_int) Jc[c+1]; ++k) w[Ir[k]]=p_data[k];
          for (k=colind[c]; k<colind[c+1]; ++k) y[k]=w[row[k]];
        }
      }
    } else {
      for (c=0; c<ncol; ++c) {
        for (k=colind[c]; k<colind[c+1]; ++k) {
          y[k] = p_data[row[k]+c*nrow];
        }
      }
    }
  }
  return y;
}

#endif

#define casadi_to_double(x) ((double) x)

#ifdef MATLAB_MEX_FILE
mxArray* casadi_to_mex(const casadi_int* sp, const casadi_real* x) {
  casadi_int nrow, ncol, c, k;
#ifndef CASADI_MEX_NO_SPARSE
  casadi_int nnz;
#endif
  const casadi_int *colind, *row;
  mxArray *p;
  double *d;
#ifndef CASADI_MEX_NO_SPARSE
  casadi_int i;
  mwIndex *j;
#endif /* CASADI_MEX_NO_SPARSE */
  nrow = *sp++;
  ncol = *sp++;
  colind = sp;
  row = sp+ncol+1;
#ifndef CASADI_MEX_NO_SPARSE
  nnz = sp[ncol];
  if (nnz!=nrow*ncol) {
    p = mxCreateSparse(nrow, ncol, nnz, mxREAL);
    for (i=0, j=mxGetJc(p); i<=ncol; ++i) *j++ = *colind++;
    for (i=0, j=mxGetIr(p); i<nnz; ++i) *j++ = *row++;
    if (x) {
      d = (double*)mxGetData(p);
      for (i=0; i<nnz; ++i) *d++ = casadi_to_double(*x++);
    }
    return p;
  }
#endif /* CASADI_MEX_NO_SPARSE */
  p = mxCreateDoubleMatrix(nrow, ncol, mxREAL);
  if (x) {
    d = (double*)mxGetData(p);
    for (c=0; c<ncol; ++c) {
      for (k=colind[c]; k<colind[c+1]; ++k) {
        d[row[k]+c*nrow] = casadi_to_double(*x++);
      }
    }
  }
  return p;
}

#endif

#ifndef CASADI_PRINTF
#ifdef MATLAB_MEX_FILE
  #define CASADI_PRINTF mexPrintf
#else
  #define CASADI_PRINTF printf
#endif
#endif

static const casadi_int casadi_s0[11] = {7, 1, 0, 7, 0, 1, 2, 3, 4, 5, 6};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[11] = {3, 2, 0, 3, 6, 0, 1, 2, 0, 1, 2};
static const casadi_int casadi_s3[69] = {12, 7, 0, 8, 20, 32, 41, 47, 53, 59, 1, 2, 4, 5, 7, 8, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 3, 4, 5, 6, 7, 8, 9, 10, 11, 6, 7, 8, 9, 10, 11, 6, 7, 8, 9, 10, 11, 6, 7, 8, 9, 10, 11};

/* f_J:(i0[7],i1[3],i2[3],i3[3x2])->(o0[12x7,59nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96;
  a0=arg[1]? arg[1][0] : 0;
  a1=arg[0]? arg[0][2] : 0;
  a2=cos(a1);
  a3=arg[0]? arg[0][1] : 0;
  a4=sin(a3);
  a5=arg[0]? arg[0][0] : 0;
  a6=cos(a5);
  a7=(a4*a6);
  a8=(a2*a7);
  a9=sin(a1);
  a10=sin(a5);
  a11=(a9*a10);
  a8=(a8-a11);
  a8=(a0*a8);
  a11=arg[1]? arg[1][1] : 0;
  a12=cos(a1);
  a10=(a12*a10);
  a13=sin(a1);
  a7=(a13*a7);
  a10=(a10+a7);
  a10=(a11*a10);
  a8=(a8-a10);
  a10=arg[1]? arg[1][2] : 0;
  a7=cos(a3);
  a6=(a7*a6);
  a6=(a10*a6);
  a8=(a8-a6);
  if (res[0]!=0) res[0][0]=a8;
  a8=cos(a5);
  a9=(a9*a8);
  a6=sin(a5);
  a14=(a4*a6);
  a15=(a2*a14);
  a9=(a9+a15);
  a9=(a0*a9);
  a12=(a12*a8);
  a14=(a13*a14);
  a12=(a12-a14);
  a12=(a11*a12);
  a9=(a9+a12);
  a7=(a7*a6);
  a7=(a10*a7);
  a9=(a9-a7);
  if (res[0]!=0) res[0][1]=a9;
  a9=arg[0]? arg[0][3] : 0;
  a7=cos(a9);
  a6=arg[2]? arg[2][0] : 0;
  a7=(a7*a6);
  a12=sin(a9);
  a14=arg[2]? arg[2][1] : 0;
  a12=(a12*a14);
  a7=(a7-a12);
  a12=cos(a1);
  a8=sin(a3);
  a15=cos(a5);
  a16=(a8*a15);
  a17=(a12*a16);
  a18=sin(a1);
  a19=sin(a5);
  a20=(a18*a19);
  a17=(a17-a20);
  a17=(a7*a17);
  a20=sin(a9);
  a20=(a20*a6);
  a21=cos(a9);
  a21=(a21*a14);
  a20=(a20+a21);
  a21=cos(a1);
  a19=(a21*a19);
  a22=sin(a1);
  a16=(a22*a16);
  a19=(a19+a16);
  a16=(a20*a19);
  a17=(a17-a16);
  a16=arg[2]? arg[2][2] : 0;
  a23=cos(a3);
  a15=(a23*a15);
  a15=(a16*a15);
  a17=(a17-a15);
  a15=-2.9999999999999999e-01;
  a19=(a15*a19);
  a17=(a17-a19);
  if (res[0]!=0) res[0][2]=a17;
  a17=cos(a5);
  a19=(a18*a17);
  a24=sin(a5);
  a25=(a8*a24);
  a26=(a12*a25);
  a19=(a19+a26);
  a19=(a7*a19);
  a17=(a21*a17);
  a25=(a22*a25);
  a17=(a17-a25);
  a25=(a20*a17);
  a19=(a19+a25);
  a23=(a23*a24);
  a23=(a16*a23);
  a19=(a19-a23);
  a17=(a15*a17);
  a19=(a19+a17);
  if (res[0]!=0) res[0][3]=a19;
  a19=arg[0]? arg[0][5] : 0;
  a17=cos(a19);
  a23=arg[0]? arg[0][6] : 0;
  a24=cos(a23);
  a25=(a17*a24);
  a26=arg[3]? arg[3][0] : 0;
  a25=(a25*a26);
  a27=sin(a23);
  a28=(a17*a27);
  a29=arg[3]? arg[3][1] : 0;
  a28=(a28*a29);
  a25=(a25-a28);
  a28=sin(a19);
  a30=arg[3]? arg[3][2] : 0;
  a28=(a28*a30);
  a25=(a25+a28);
  a28=cos(a9);
  a31=cos(a1);
  a32=sin(a3);
  a33=cos(a5);
  a34=(a32*a33);
  a35=(a31*a34);
  a36=sin(a1);
  a37=sin(a5);
  a38=(a36*a37);
  a35=(a35-a38);
  a38=(a28*a35);
  a39=sin(a9);
  a40=cos(a1);
  a37=(a40*a37);
  a41=sin(a1);
  a34=(a41*a34);
  a37=(a37+a34);
  a34=(a39*a37);
  a38=(a38-a34);
  a38=(a25*a38);
  a34=arg[0]? arg[0][4] : 0;
  a42=sin(a34);
  a43=sin(a19);
  a44=(a42*a43);
  a45=(a44*a24);
  a46=cos(a34);
  a47=sin(a23);
  a48=(a46*a47);
  a45=(a45+a48);
  a45=(a45*a26);
  a48=cos(a23);
  a49=(a46*a48);
  a50=(a44*a27);
  a49=(a49-a50);
  a49=(a49*a29);
  a45=(a45+a49);
  a49=cos(a19);
  a50=(a42*a49);
  a50=(a50*a30);
  a45=(a45-a50);
  a50=cos(a9);
  a51=(a50*a37);
  a52=sin(a9);
  a53=(a52*a35);
  a51=(a51+a53);
  a51=(a45*a51);
  a38=(a38-a51);
  a51=sin(a34);
  a53=(a51*a47);
  a54=cos(a34);
  a55=(a54*a43);
  a56=(a55*a24);
  a53=(a53-a56);
  a53=(a53*a26);
  a56=(a55*a27);
  a57=(a51*a48);
  a56=(a56+a57);
  a56=(a56*a29);
  a53=(a53+a56);
  a56=(a54*a49);
  a56=(a56*a30);
  a53=(a53+a56);
  a56=cos(a3);
  a33=(a56*a33);
  a33=(a53*a33);
  a38=(a38-a33);
  a33=-2.6000000000000001e-01;
  a57=(a33*a50);
  a58=(a57*a37);
  a59=(a33*a52);
  a35=(a59*a35);
  a58=(a58+a35);
  a37=(a15*a37);
  a58=(a58+a37);
  a38=(a38-a58);
  if (res[0]!=0) res[0][4]=a38;
  a38=cos(a5);
  a58=(a36*a38);
  a37=sin(a5);
  a35=(a32*a37);
  a60=(a31*a35);
  a58=(a58+a60);
  a60=(a28*a58);
  a38=(a40*a38);
  a35=(a41*a35);
  a38=(a38-a35);
  a35=(a39*a38);
  a60=(a60+a35);
  a60=(a25*a60);
  a35=(a50*a38);
  a61=(a52*a58);
  a35=(a35-a61);
  a35=(a45*a35);
  a60=(a60+a35);
  a37=(a56*a37);
  a37=(a53*a37);
  a60=(a60-a37);
  a37=(a57*a38);
  a58=(a59*a58);
  a37=(a37-a58);
  a38=(a15*a38);
  a37=(a37+a38);
  a60=(a60+a37);
  if (res[0]!=0) res[0][5]=a60;
  a60=cos(a19);
  a37=cos(a23);
  a38=(a60*a37);
  a58=arg[3]? arg[3][3] : 0;
  a38=(a38*a58);
  a35=sin(a23);
  a61=(a60*a35);
  a62=arg[3]? arg[3][4] : 0;
  a61=(a61*a62);
  a38=(a38-a61);
  a61=sin(a19);
  a63=arg[3]? arg[3][5] : 0;
  a61=(a61*a63);
  a38=(a38+a61);
  a61=cos(a9);
  a64=cos(a1);
  a65=sin(a3);
  a66=cos(a5);
  a67=(a65*a66);
  a68=(a64*a67);
  a69=sin(a1);
  a70=sin(a5);
  a71=(a69*a70);
  a68=(a68-a71);
  a71=(a61*a68);
  a72=sin(a9);
  a73=cos(a1);
  a70=(a73*a70);
  a74=sin(a1);
  a67=(a74*a67);
  a70=(a70+a67);
  a67=(a72*a70);
  a71=(a71-a67);
  a71=(a38*a71);
  a67=sin(a34);
  a75=sin(a19);
  a76=(a67*a75);
  a77=(a76*a37);
  a78=cos(a34);
  a79=sin(a23);
  a80=(a78*a79);
  a77=(a77+a80);
  a77=(a77*a58);
  a80=cos(a23);
  a81=(a78*a80);
  a82=(a76*a35);
  a81=(a81-a82);
  a81=(a81*a62);
  a77=(a77+a81);
  a81=cos(a19);
  a82=(a67*a81);
  a82=(a82*a63);
  a77=(a77-a82);
  a82=cos(a9);
  a83=(a82*a70);
  a84=sin(a9);
  a85=(a84*a68);
  a83=(a83+a85);
  a83=(a77*a83);
  a71=(a71-a83);
  a83=sin(a34);
  a85=(a83*a79);
  a86=cos(a34);
  a87=(a86*a75);
  a88=(a87*a37);
  a85=(a85-a88);
  a85=(a85*a58);
  a88=(a87*a35);
  a89=(a83*a80);
  a88=(a88+a89);
  a88=(a88*a62);
  a85=(a85+a88);
  a88=(a86*a81);
  a88=(a88*a63);
  a85=(a85+a88);
  a88=cos(a3);
  a66=(a88*a66);
  a66=(a85*a66);
  a71=(a71-a66);
  a66=(a33*a82);
  a89=(a66*a70);
  a90=(a33*a84);
  a68=(a90*a68);
  a89=(a89+a68);
  a70=(a15*a70);
  a89=(a89+a70);
  a71=(a71-a89);
  if (res[0]!=0) res[0][6]=a71;
  a71=cos(a5);
  a89=(a69*a71);
  a70=sin(a5);
  a68=(a65*a70);
  a91=(a64*a68);
  a89=(a89+a91);
  a91=(a61*a89);
  a71=(a73*a71);
  a68=(a74*a68);
  a71=(a71-a68);
  a68=(a72*a71);
  a91=(a91+a68);
  a91=(a38*a91);
  a68=(a82*a71);
  a92=(a84*a89);
  a68=(a68-a92);
  a68=(a77*a68);
  a91=(a91+a68);
  a70=(a88*a70);
  a70=(a85*a70);
  a91=(a91-a70);
  a70=(a66*a71);
  a89=(a90*a89);
  a70=(a70-a89);
  a71=(a15*a71);
  a70=(a70+a71);
  a91=(a91+a70);
  if (res[0]!=0) res[0][7]=a91;
  a91=sin(a3);
  a70=(a13*a91);
  a70=(a11*a70);
  a91=(a2*a91);
  a91=(a0*a91);
  a70=(a70-a91);
  a91=cos(a3);
  a91=(a10*a91);
  a70=(a70+a91);
  if (res[0]!=0) res[0][8]=a70;
  a70=sin(a5);
  a91=cos(a3);
  a71=(a70*a91);
  a89=(a2*a71);
  a89=(a0*a89);
  a71=(a13*a71);
  a71=(a11*a71);
  a89=(a89-a71);
  a71=sin(a3);
  a68=(a70*a71);
  a68=(a10*a68);
  a89=(a89+a68);
  if (res[0]!=0) res[0][9]=a89;
  a89=cos(a5);
  a91=(a89*a91);
  a13=(a13*a91);
  a13=(a11*a13);
  a2=(a2*a91);
  a2=(a0*a2);
  a13=(a13-a2);
  a71=(a89*a71);
  a10=(a10*a71);
  a13=(a13-a10);
  if (res[0]!=0) res[0][10]=a13;
  a13=sin(a3);
  a10=(a22*a13);
  a71=(a20*a10);
  a13=(a12*a13);
  a13=(a7*a13);
  a71=(a71-a13);
  a13=cos(a3);
  a13=(a16*a13);
  a71=(a71+a13);
  a10=(a15*a10);
  a71=(a71+a10);
  if (res[0]!=0) res[0][11]=a71;
  a71=sin(a5);
  a10=cos(a3);
  a13=(a71*a10);
  a2=(a12*a13);
  a2=(a7*a2);
  a13=(a22*a13);
  a91=(a20*a13);
  a2=(a2-a91);
  a91=sin(a3);
  a68=(a71*a91);
  a68=(a16*a68);
  a2=(a2+a68);
  a13=(a15*a13);
  a2=(a2-a13);
  if (res[0]!=0) res[0][12]=a2;
  a2=cos(a5);
  a10=(a2*a10);
  a13=(a22*a10);
  a68=(a20*a13);
  a10=(a12*a10);
  a10=(a7*a10);
  a68=(a68-a10);
  a91=(a2*a91);
  a16=(a16*a91);
  a68=(a68-a16);
  a13=(a15*a13);
  a68=(a68+a13);
  if (res[0]!=0) res[0][13]=a68;
  a68=sin(a3);
  a13=(a41*a68);
  a16=(a39*a13);
  a68=(a31*a68);
  a91=(a28*a68);
  a16=(a16-a91);
  a16=(a25*a16);
  a91=(a52*a68);
  a10=(a50*a13);
  a91=(a91+a10);
  a91=(a45*a91);
  a16=(a16+a91);
  a91=cos(a3);
  a91=(a53*a91);
  a16=(a16+a91);
  a68=(a59*a68);
  a91=(a57*a13);
  a68=(a68+a91);
  a13=(a15*a13);
  a68=(a68+a13);
  a16=(a16+a68);
  if (res[0]!=0) res[0][14]=a16;
  a16=sin(a5);
  a68=cos(a3);
  a13=(a16*a68);
  a91=(a31*a13);
  a10=(a28*a91);
  a13=(a41*a13);
  a92=(a39*a13);
  a10=(a10-a92);
  a10=(a25*a10);
  a92=(a50*a13);
  a93=(a52*a91);
  a92=(a92+a93);
  a92=(a45*a92);
  a10=(a10-a92);
  a92=sin(a3);
  a93=(a16*a92);
  a93=(a53*a93);
  a10=(a10+a93);
  a93=(a57*a13);
  a91=(a59*a91);
  a93=(a93+a91);
  a13=(a15*a13);
  a93=(a93+a13);
  a10=(a10-a93);
  if (res[0]!=0) res[0][15]=a10;
  a10=cos(a5);
  a68=(a10*a68);
  a93=(a41*a68);
  a13=(a39*a93);
  a68=(a31*a68);
  a91=(a28*a68);
  a13=(a13-a91);
  a13=(a25*a13);
  a91=(a50*a93);
  a94=(a52*a68);
  a91=(a91+a94);
  a91=(a45*a91);
  a13=(a13+a91);
  a92=(a10*a92);
  a53=(a53*a92);
  a13=(a13-a53);
  a53=(a57*a93);
  a68=(a59*a68);
  a53=(a53+a68);
  a93=(a15*a93);
  a53=(a53+a93);
  a13=(a13+a53);
  if (res[0]!=0) res[0][16]=a13;
  a13=sin(a3);
  a53=(a74*a13);
  a93=(a72*a53);
  a13=(a64*a13);
  a68=(a61*a13);
  a93=(a93-a68);
  a93=(a38*a93);
  a68=(a84*a13);
  a92=(a82*a53);
  a68=(a68+a92);
  a68=(a77*a68);
  a93=(a93+a68);
  a68=cos(a3);
  a68=(a85*a68);
  a93=(a93+a68);
  a13=(a90*a13);
  a68=(a66*a53);
  a13=(a13+a68);
  a53=(a15*a53);
  a13=(a13+a53);
  a93=(a93+a13);
  if (res[0]!=0) res[0][17]=a93;
  a93=sin(a5);
  a13=cos(a3);
  a53=(a93*a13);
  a68=(a64*a53);
  a92=(a61*a68);
  a53=(a74*a53);
  a91=(a72*a53);
  a92=(a92-a91);
  a92=(a38*a92);
  a91=(a82*a53);
  a94=(a84*a68);
  a91=(a91+a94);
  a91=(a77*a91);
  a92=(a92-a91);
  a91=sin(a3);
  a94=(a93*a91);
  a94=(a85*a94);
  a92=(a92+a94);
  a94=(a66*a53);
  a68=(a90*a68);
  a94=(a94+a68);
  a53=(a15*a53);
  a94=(a94+a53);
  a92=(a92-a94);
  if (res[0]!=0) res[0][18]=a92;
  a92=cos(a5);
  a13=(a92*a13);
  a94=(a74*a13);
  a53=(a72*a94);
  a13=(a64*a13);
  a68=(a61*a13);
  a53=(a53-a68);
  a53=(a38*a53);
  a68=(a82*a94);
  a95=(a84*a13);
  a68=(a68+a95);
  a68=(a77*a68);
  a53=(a53+a68);
  a91=(a92*a91);
  a85=(a85*a91);
  a53=(a53-a85);
  a85=(a66*a94);
  a13=(a90*a13);
  a85=(a85+a13);
  a94=(a15*a94);
  a85=(a85+a94);
  a53=(a53+a85);
  if (res[0]!=0) res[0][19]=a53;
  a53=cos(a3);
  a85=sin(a1);
  a94=(a53*a85);
  a94=(a0*a94);
  a13=cos(a1);
  a53=(a53*a13);
  a53=(a11*a53);
  a94=(a94+a53);
  a94=(-a94);
  if (res[0]!=0) res[0][20]=a94;
  a94=cos(a5);
  a53=cos(a1);
  a91=(a94*a53);
  a70=(a70*a4);
  a68=(a70*a85);
  a91=(a91-a68);
  a91=(a0*a91);
  a68=sin(a1);
  a94=(a94*a68);
  a70=(a70*a13);
  a94=(a94+a70);
  a94=(a11*a94);
  a91=(a91-a94);
  if (res[0]!=0) res[0][21]=a91;
  a91=sin(a5);
  a53=(a91*a53);
  a89=(a89*a4);
  a85=(a89*a85);
  a53=(a53+a85);
  a0=(a0*a53);
  a89=(a89*a13);
  a91=(a91*a68);
  a89=(a89-a91);
  a11=(a11*a89);
  a0=(a0+a11);
  if (res[0]!=0) res[0][22]=a0;
  a0=cos(a3);
  a11=sin(a1);
  a89=(a0*a11);
  a89=(a7*a89);
  a91=cos(a1);
  a68=(a0*a91);
  a13=(a20*a68);
  a89=(a89+a13);
  a68=(a15*a68);
  a89=(a89+a68);
  a89=(-a89);
  if (res[0]!=0) res[0][23]=a89;
  a89=cos(a5);
  a68=cos(a1);
  a13=(a89*a68);
  a71=(a71*a8);
  a53=(a71*a11);
  a13=(a13-a53);
  a13=(a7*a13);
  a53=sin(a1);
  a85=(a89*a53);
  a4=(a71*a91);
  a85=(a85+a4);
  a4=(a20*a85);
  a13=(a13-a4);
  a85=(a15*a85);
  a13=(a13-a85);
  if (res[0]!=0) res[0][24]=a13;
  a13=sin(a5);
  a68=(a13*a68);
  a2=(a2*a8);
  a11=(a2*a11);
  a68=(a68+a11);
  a7=(a7*a68);
  a91=(a2*a91);
  a53=(a13*a53);
  a91=(a91-a53);
  a20=(a20*a91);
  a7=(a7+a20);
  a91=(a15*a91);
  a7=(a7+a91);
  if (res[0]!=0) res[0][25]=a7;
  a7=cos(a3);
  a91=sin(a1);
  a20=(a7*a91);
  a53=(a28*a20);
  a68=cos(a1);
  a11=(a7*a68);
  a8=(a39*a11);
  a53=(a53+a8);
  a53=(a25*a53);
  a8=(a50*a11);
  a85=(a52*a20);
  a8=(a8-a85);
  a8=(a45*a8);
  a53=(a53+a8);
  a8=(a57*a11);
  a20=(a59*a20);
  a8=(a8-a20);
  a11=(a15*a11);
  a8=(a8+a11);
  a53=(a53+a8);
  a53=(-a53);
  if (res[0]!=0) res[0][26]=a53;
  a53=cos(a5);
  a8=cos(a1);
  a11=(a53*a8);
  a20=(a16*a32);
  a85=(a20*a91);
  a11=(a11-a85);
  a85=(a28*a11);
  a4=sin(a1);
  a94=(a53*a4);
  a70=(a20*a68);
  a94=(a94+a70);
  a70=(a39*a94);
  a85=(a85-a70);
  a85=(a25*a85);
  a70=(a50*a94);
  a95=(a52*a11);
  a70=(a70+a95);
  a70=(a45*a70);
  a85=(a85-a70);
  a70=(a57*a94);
  a11=(a59*a11);
  a70=(a70+a11);
  a94=(a15*a94);
  a70=(a70+a94);
  a85=(a85-a70);
  if (res[0]!=0) res[0][27]=a85;
  a85=sin(a5);
  a8=(a85*a8);
  a32=(a10*a32);
  a91=(a32*a91);
  a8=(a8+a91);
  a91=(a28*a8);
  a68=(a32*a68);
  a4=(a85*a4);
  a68=(a68-a4);
  a4=(a39*a68);
  a91=(a91+a4);
  a91=(a25*a91);
  a4=(a50*a68);
  a70=(a52*a8);
  a4=(a4-a70);
  a4=(a45*a4);
  a91=(a91+a4);
  a57=(a57*a68);
  a59=(a59*a8);
  a57=(a57-a59);
  a68=(a15*a68);
  a57=(a57+a68);
  a91=(a91+a57);
  if (res[0]!=0) res[0][28]=a91;
  a91=cos(a3);
  a57=sin(a1);
  a68=(a91*a57);
  a59=(a61*a68);
  a8=cos(a1);
  a4=(a91*a8);
  a70=(a72*a4);
  a59=(a59+a70);
  a59=(a38*a59);
  a70=(a82*a4);
  a94=(a84*a68);
  a70=(a70-a94);
  a70=(a77*a70);
  a59=(a59+a70);
  a70=(a66*a4);
  a68=(a90*a68);
  a70=(a70-a68);
  a4=(a15*a4);
  a70=(a70+a4);
  a59=(a59+a70);
  a59=(-a59);
  if (res[0]!=0) res[0][29]=a59;
  a59=cos(a5);
  a70=cos(a1);
  a4=(a59*a70);
  a68=(a93*a65);
  a94=(a68*a57);
  a4=(a4-a94);
  a94=(a61*a4);
  a1=sin(a1);
  a11=(a59*a1);
  a95=(a68*a8);
  a11=(a11+a95);
  a95=(a72*a11);
  a94=(a94-a95);
  a94=(a38*a94);
  a95=(a82*a11);
  a96=(a84*a4);
  a95=(a95+a96);
  a95=(a77*a95);
  a94=(a94-a95);
  a95=(a66*a11);
  a4=(a90*a4);
  a95=(a95+a4);
  a11=(a15*a11);
  a95=(a95+a11);
  a94=(a94-a95);
  if (res[0]!=0) res[0][30]=a94;
  a5=sin(a5);
  a70=(a5*a70);
  a65=(a92*a65);
  a57=(a65*a57);
  a70=(a70+a57);
  a57=(a61*a70);
  a8=(a65*a8);
  a1=(a5*a1);
  a8=(a8-a1);
  a1=(a72*a8);
  a57=(a57+a1);
  a57=(a38*a57);
  a1=(a82*a8);
  a94=(a84*a70);
  a1=(a1-a94);
  a1=(a77*a1);
  a57=(a57+a1);
  a66=(a66*a8);
  a90=(a90*a70);
  a66=(a66-a90);
  a15=(a15*a8);
  a66=(a66+a15);
  a57=(a57+a66);
  if (res[0]!=0) res[0][31]=a57;
  a57=(a0*a12);
  a66=sin(a9);
  a66=(a6*a66);
  a15=cos(a9);
  a15=(a14*a15);
  a66=(a66+a15);
  a57=(a57*a66);
  a0=(a0*a22);
  a15=cos(a9);
  a6=(a6*a15);
  a15=sin(a9);
  a14=(a14*a15);
  a6=(a6-a14);
  a0=(a0*a6);
  a57=(a57+a0);
  a57=(-a57);
  if (res[0]!=0) res[0][32]=a57;
  a57=(a89*a21);
  a0=(a71*a22);
  a57=(a57-a0);
  a57=(a57*a6);
  a71=(a71*a12);
  a89=(a89*a18);
  a71=(a71+a89);
  a71=(a71*a66);
  a57=(a57-a71);
  if (res[0]!=0) res[0][33]=a57;
  a22=(a2*a22);
  a21=(a13*a21);
  a22=(a22+a21);
  a22=(a22*a6);
  a13=(a13*a18);
  a2=(a2*a12);
  a13=(a13-a2);
  a13=(a13*a66);
  a22=(a22-a13);
  if (res[0]!=0) res[0][34]=a22;
  a22=(a7*a31);
  a13=sin(a9);
  a66=(a22*a13);
  a7=(a7*a41);
  a2=cos(a9);
  a12=(a7*a2);
  a66=(a66+a12);
  a66=(a25*a66);
  a12=cos(a9);
  a18=(a22*a12);
  a6=sin(a9);
  a21=(a7*a6);
  a18=(a18-a21);
  a18=(a45*a18);
  a66=(a66+a18);
  a18=(a33*a12);
  a21=(a22*a18);
  a57=(a33*a6);
  a71=(a7*a57);
  a21=(a21-a71);
  a66=(a66+a21);
  a66=(-a66);
  if (res[0]!=0) res[0][35]=a66;
  a66=(a53*a40);
  a21=(a20*a41);
  a66=(a66-a21);
  a21=(a66*a2);
  a20=(a20*a31);
  a53=(a53*a36);
  a20=(a20+a53);
  a53=(a20*a13);
  a21=(a21-a53);
  a21=(a25*a21);
  a53=(a66*a6);
  a71=(a20*a12);
  a53=(a53+a71);
  a53=(a45*a53);
  a21=(a21-a53);
  a53=(a66*a57);
  a71=(a20*a18);
  a53=(a53+a71);
  a21=(a21-a53);
  if (res[0]!=0) res[0][36]=a21;
  a41=(a32*a41);
  a40=(a85*a40);
  a41=(a41+a40);
  a2=(a41*a2);
  a85=(a85*a36);
  a32=(a32*a31);
  a85=(a85-a32);
  a13=(a85*a13);
  a2=(a2-a13);
  a25=(a25*a2);
  a6=(a41*a6);
  a12=(a85*a12);
  a6=(a6+a12);
  a45=(a45*a6);
  a25=(a25-a45);
  a57=(a41*a57);
  a18=(a85*a18);
  a57=(a57+a18);
  a25=(a25-a57);
  if (res[0]!=0) res[0][37]=a25;
  a25=(a91*a64);
  a57=sin(a9);
  a18=(a25*a57);
  a91=(a91*a74);
  a45=cos(a9);
  a6=(a91*a45);
  a18=(a18+a6);
  a18=(a38*a18);
  a6=cos(a9);
  a12=(a25*a6);
  a9=sin(a9);
  a2=(a91*a9);
  a12=(a12-a2);
  a12=(a77*a12);
  a18=(a18+a12);
  a12=(a33*a6);
  a2=(a25*a12);
  a33=(a33*a9);
  a13=(a91*a33);
  a2=(a2-a13);
  a18=(a18+a2);
  a18=(-a18);
  if (res[0]!=0) res[0][38]=a18;
  a18=(a59*a73);
  a2=(a68*a74);
  a18=(a18-a2);
  a2=(a18*a45);
  a68=(a68*a64);
  a59=(a59*a69);
  a68=(a68+a59);
  a59=(a68*a57);
  a2=(a2-a59);
  a2=(a38*a2);
  a59=(a18*a9);
  a13=(a68*a6);
  a59=(a59+a13);
  a59=(a77*a59);
  a2=(a2-a59);
  a59=(a18*a33);
  a13=(a68*a12);
  a59=(a59+a13);
  a2=(a2-a59);
  if (res[0]!=0) res[0][39]=a2;
  a74=(a65*a74);
  a73=(a5*a73);
  a74=(a74+a73);
  a45=(a74*a45);
  a5=(a5*a69);
  a65=(a65*a64);
  a5=(a5-a65);
  a57=(a5*a57);
  a45=(a45-a57);
  a38=(a38*a45);
  a9=(a74*a9);
  a6=(a5*a6);
  a9=(a9+a6);
  a77=(a77*a9);
  a38=(a38-a77);
  a33=(a74*a33);
  a12=(a5*a12);
  a33=(a33+a12);
  a38=(a38-a33);
  if (res[0]!=0) res[0][40]=a38;
  a38=sin(a3);
  a33=cos(a34);
  a12=(a47*a33);
  a77=sin(a34);
  a9=(a43*a77);
  a6=(a24*a9);
  a12=(a12+a6);
  a12=(a26*a12);
  a33=(a48*a33);
  a9=(a27*a9);
  a33=(a33-a9);
  a33=(a29*a33);
  a12=(a12+a33);
  a77=(a49*a77);
  a77=(a30*a77);
  a12=(a12-a77);
  a77=(a38*a12);
  a33=(a22*a52);
  a9=(a7*a50);
  a33=(a33+a9);
  a9=cos(a34);
  a43=(a43*a9);
  a6=(a24*a43);
  a45=sin(a34);
  a47=(a47*a45);
  a6=(a6-a47);
  a6=(a26*a6);
  a48=(a48*a45);
  a43=(a27*a43);
  a48=(a48+a43);
  a48=(a29*a48);
  a6=(a6-a48);
  a49=(a49*a9);
  a49=(a30*a49);
  a6=(a6-a49);
  a49=(a33*a6);
  a77=(a77-a49);
  if (res[0]!=0) res[0][41]=a77;
  a77=(a66*a50);
  a49=(a20*a52);
  a77=(a77-a49);
  a49=(a77*a6);
  a16=(a16*a56);
  a9=(a16*a12);
  a49=(a49-a9);
  if (res[0]!=0) res[0][42]=a49;
  a50=(a41*a50);
  a52=(a85*a52);
  a50=(a50-a52);
  a6=(a50*a6);
  a10=(a10*a56);
  a12=(a10*a12);
  a6=(a6+a12);
  if (res[0]!=0) res[0][43]=a6;
  a3=sin(a3);
  a6=cos(a34);
  a12=(a79*a6);
  a56=sin(a34);
  a52=(a75*a56);
  a49=(a37*a52);
  a12=(a12+a49);
  a12=(a58*a12);
  a6=(a80*a6);
  a52=(a35*a52);
  a6=(a6-a52);
  a6=(a62*a6);
  a12=(a12+a6);
  a56=(a81*a56);
  a56=(a63*a56);
  a12=(a12-a56);
  a56=(a3*a12);
  a6=(a25*a84);
  a52=(a91*a82);
  a6=(a6+a52);
  a52=cos(a34);
  a75=(a75*a52);
  a49=(a37*a75);
  a34=sin(a34);
  a79=(a79*a34);
  a49=(a49-a79);
  a49=(a58*a49);
  a80=(a80*a34);
  a75=(a35*a75);
  a80=(a80+a75);
  a80=(a62*a80);
  a49=(a49-a80);
  a81=(a81*a52);
  a81=(a63*a81);
  a49=(a49-a81);
  a81=(a6*a49);
  a56=(a56-a81);
  if (res[0]!=0) res[0][44]=a56;
  a56=(a18*a82);
  a81=(a68*a84);
  a56=(a56-a81);
  a81=(a56*a49);
  a93=(a93*a88);
  a52=(a93*a12);
  a81=(a81-a52);
  if (res[0]!=0) res[0][45]=a81;
  a82=(a74*a82);
  a84=(a5*a84);
  a82=(a82-a84);
  a49=(a82*a49);
  a92=(a92*a88);
  a12=(a92*a12);
  a49=(a49+a12);
  if (res[0]!=0) res[0][46]=a49;
  a22=(a22*a28);
  a7=(a7*a39);
  a22=(a22-a7);
  a7=sin(a19);
  a49=(a27*a7);
  a49=(a29*a49);
  a7=(a24*a7);
  a7=(a26*a7);
  a49=(a49-a7);
  a7=cos(a19);
  a7=(a30*a7);
  a49=(a49+a7);
  a7=(a22*a49);
  a12=cos(a19);
  a88=(a42*a12);
  a84=(a24*a88);
  a84=(a26*a84);
  a88=(a27*a88);
  a88=(a29*a88);
  a84=(a84-a88);
  a88=sin(a19);
  a42=(a42*a88);
  a42=(a30*a42);
  a84=(a84+a42);
  a42=(a33*a84);
  a7=(a7-a42);
  a12=(a54*a12);
  a27=(a27*a12);
  a27=(a29*a27);
  a24=(a24*a12);
  a24=(a26*a24);
  a27=(a27-a24);
  a54=(a54*a88);
  a30=(a30*a54);
  a27=(a27-a30);
  a30=(a38*a27);
  a7=(a7+a30);
  if (res[0]!=0) res[0][47]=a7;
  a20=(a20*a28);
  a66=(a66*a39);
  a20=(a20+a66);
  a66=(a20*a49);
  a7=(a77*a84);
  a66=(a66+a7);
  a7=(a16*a27);
  a66=(a66-a7);
  if (res[0]!=0) res[0][48]=a66;
  a85=(a85*a28);
  a41=(a41*a39);
  a85=(a85+a41);
  a49=(a85*a49);
  a84=(a50*a84);
  a49=(a49+a84);
  a27=(a10*a27);
  a49=(a49+a27);
  if (res[0]!=0) res[0][49]=a49;
  a25=(a25*a61);
  a91=(a91*a72);
  a25=(a25-a91);
  a91=sin(a19);
  a49=(a35*a91);
  a49=(a62*a49);
  a91=(a37*a91);
  a91=(a58*a91);
  a49=(a49-a91);
  a91=cos(a19);
  a91=(a63*a91);
  a49=(a49+a91);
  a91=(a25*a49);
  a27=cos(a19);
  a84=(a67*a27);
  a41=(a37*a84);
  a41=(a58*a41);
  a84=(a35*a84);
  a84=(a62*a84);
  a41=(a41-a84);
  a19=sin(a19);
  a67=(a67*a19);
  a67=(a63*a67);
  a41=(a41+a67);
  a67=(a6*a41);
  a91=(a91-a67);
  a27=(a86*a27);
  a35=(a35*a27);
  a35=(a62*a35);
  a37=(a37*a27);
  a37=(a58*a37);
  a35=(a35-a37);
  a86=(a86*a19);
  a63=(a63*a86);
  a35=(a35-a63);
  a63=(a3*a35);
  a91=(a91+a63);
  if (res[0]!=0) res[0][50]=a91;
  a68=(a68*a61);
  a18=(a18*a72);
  a68=(a68+a18);
  a18=(a68*a49);
  a91=(a56*a41);
  a18=(a18+a91);
  a91=(a93*a35);
  a18=(a18-a91);
  if (res[0]!=0) res[0][51]=a18;
  a5=(a5*a61);
  a74=(a74*a72);
  a5=(a5+a74);
  a49=(a5*a49);
  a41=(a82*a41);
  a49=(a49+a41);
  a35=(a92*a35);
  a49=(a49+a35);
  if (res[0]!=0) res[0][52]=a49;
  a49=cos(a23);
  a35=(a51*a49);
  a41=sin(a23);
  a74=(a55*a41);
  a35=(a35+a74);
  a35=(a26*a35);
  a74=cos(a23);
  a55=(a55*a74);
  a72=sin(a23);
  a51=(a51*a72);
  a55=(a55-a51);
  a55=(a29*a55);
  a35=(a35+a55);
  a38=(a38*a35);
  a55=(a17*a41);
  a55=(a26*a55);
  a17=(a17*a74);
  a17=(a29*a17);
  a55=(a55+a17);
  a22=(a22*a55);
  a49=(a46*a49);
  a41=(a44*a41);
  a49=(a49-a41);
  a26=(a26*a49);
  a46=(a46*a72);
  a44=(a44*a74);
  a46=(a46+a44);
  a29=(a29*a46);
  a26=(a26-a29);
  a33=(a33*a26);
  a22=(a22+a33);
  a38=(a38-a22);
  if (res[0]!=0) res[0][53]=a38;
  a77=(a77*a26);
  a20=(a20*a55);
  a77=(a77-a20);
  a16=(a16*a35);
  a77=(a77-a16);
  if (res[0]!=0) res[0][54]=a77;
  a50=(a50*a26);
  a85=(a85*a55);
  a50=(a50-a85);
  a10=(a10*a35);
  a50=(a50+a10);
  if (res[0]!=0) res[0][55]=a50;
  a50=cos(a23);
  a10=(a83*a50);
  a35=sin(a23);
  a85=(a87*a35);
  a10=(a10+a85);
  a10=(a58*a10);
  a85=cos(a23);
  a87=(a87*a85);
  a23=sin(a23);
  a83=(a83*a23);
  a87=(a87-a83);
  a87=(a62*a87);
  a10=(a10+a87);
  a3=(a3*a10);
  a87=(a60*a35);
  a87=(a58*a87);
  a60=(a60*a85);
  a60=(a62*a60);
  a87=(a87+a60);
  a25=(a25*a87);
  a50=(a78*a50);
  a35=(a76*a35);
  a50=(a50-a35);
  a58=(a58*a50);
  a78=(a78*a23);
  a76=(a76*a85);
  a78=(a78+a76);
  a62=(a62*a78);
  a58=(a58-a62);
  a6=(a6*a58);
  a25=(a25+a6);
  a3=(a3-a25);
  if (res[0]!=0) res[0][56]=a3;
  a56=(a56*a58);
  a68=(a68*a87);
  a56=(a56-a68);
  a93=(a93*a10);
  a56=(a56-a93);
  if (res[0]!=0) res[0][57]=a56;
  a82=(a82*a58);
  a5=(a5*a87);
  a82=(a82-a5);
  a92=(a92*a10);
  a82=(a82+a92);
  if (res[0]!=0) res[0][58]=a82;
  return 0;
}

CASADI_SYMBOL_EXPORT int f_J(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int f_J_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int f_J_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void f_J_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int f_J_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void f_J_release(int mem) {
}

CASADI_SYMBOL_EXPORT void f_J_incref(void) {
}

CASADI_SYMBOL_EXPORT void f_J_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int f_J_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int f_J_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real f_J_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* f_J_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* f_J_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* f_J_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* f_J_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int f_J_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

#ifdef MATLAB_MEX_FILE
void mex_f_J(int resc, mxArray *resv[], int argc, const mxArray *argv[]) {
  casadi_int i;
  casadi_real w[175];
  casadi_int *iw = 0;
  const casadi_real* arg[4] = {0};
  casadi_real* res[1] = {0};
  if (argc>4) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"f_J\" failed. Too many input arguments (%d, max 4)", argc);
  if (resc>1) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"f_J\" failed. Too many output arguments (%d, max 1)", resc);
  if (--argc>=0) arg[0] = casadi_from_mex(argv[0], w, casadi_s0, w+78);
  if (--argc>=0) arg[1] = casadi_from_mex(argv[1], w+7, casadi_s1, w+78);
  if (--argc>=0) arg[2] = casadi_from_mex(argv[2], w+10, casadi_s1, w+78);
  if (--argc>=0) arg[3] = casadi_from_mex(argv[3], w+13, casadi_s2, w+78);
  --resc;
  res[0] = w+19;
  i = f_J(arg, res, iw, w+78, 0);
  if (i) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"f_J\" failed.");
  if (res[0]) resv[0] = casadi_to_mex(casadi_s3, res[0]);
}
#endif

casadi_int main_f_J(casadi_int argc, char* argv[]) {
  casadi_int j;
  casadi_real* a;
  const casadi_real* r;
  casadi_int flag;
  casadi_int *iw = 0;
  casadi_real w[175];
  const casadi_real* arg[4];
  casadi_real* res[1];
  arg[0] = w+0;
  arg[1] = w+7;
  arg[2] = w+10;
  arg[3] = w+13;
  res[0] = w+19;
  a = w;
  for (j=0; j<19; ++j) if (scanf("%lg", a++)<=0) return 2;
  flag = f_J(arg, res, iw, w+78, 0);
  if (flag) return flag;
  r = w+19;
  for (j=0; j<59; ++j) CASADI_PRINTF("%g ", *r++);
  CASADI_PRINTF("\n");
  return 0;
}


#ifdef MATLAB_MEX_FILE
void mexFunction(int resc, mxArray *resv[], int argc, const mxArray *argv[]) {
  char buf[4];
  int buf_ok = argc > 0 && !mxGetString(*argv, buf, sizeof(buf));
  if (!buf_ok) {
    mex_f_J(resc, resv, argc, argv);
    return;
  } else if (strcmp(buf, "f_J")==0) {
    mex_f_J(resc, resv, argc-1, argv+1);
    return;
  }
  mexErrMsgTxt("First input should be a command string. Possible values: 'f_J'");
}
#endif
int main(int argc, char* argv[]) {
  if (argc<2) {
    /* name error */
  } else if (strcmp(argv[1], "f_J")==0) {
    return main_f_J(argc-2, argv+2);
  }
  fprintf(stderr, "First input should be a command string. Possible values: 'f_J'\nNote: you may use function.generate_input to create a command string.\n");
  return 1;
}
#ifdef __cplusplus
} /* extern "C" */
#endif
