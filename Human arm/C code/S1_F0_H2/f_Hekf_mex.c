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
#define casadi_s4 CASADI_PREFIX(s4)
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
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[11] = {3, 2, 0, 3, 6, 0, 1, 2, 0, 1, 2};
static const casadi_int casadi_s4[58] = {9, 7, 0, 6, 15, 24, 30, 36, 42, 48, 1, 2, 4, 5, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 3, 4, 5, 6, 7, 8, 3, 4, 5, 6, 7, 8, 3, 4, 5, 6, 7, 8, 3, 4, 5, 6, 7, 8};

/* f_J:(i0[7],i1[3],i2[],i3[3x2])->(o0[9x7,48nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a9;
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
  a9=arg[0]? arg[0][5] : 0;
  a7=cos(a9);
  a6=arg[0]? arg[0][6] : 0;
  a12=cos(a6);
  a14=(a7*a12);
  a8=arg[3]? arg[3][0] : 0;
  a14=(a14*a8);
  a15=sin(a6);
  a16=(a7*a15);
  a17=arg[3]? arg[3][1] : 0;
  a16=(a16*a17);
  a14=(a14-a16);
  a16=sin(a9);
  a18=arg[3]? arg[3][2] : 0;
  a16=(a16*a18);
  a14=(a14+a16);
  a16=cos(a1);
  a19=sin(a3);
  a20=cos(a5);
  a21=(a19*a20);
  a22=(a16*a21);
  a23=sin(a1);
  a24=sin(a5);
  a25=(a23*a24);
  a22=(a22-a25);
  a22=(a14*a22);
  a25=arg[0]? arg[0][4] : 0;
  a26=sin(a25);
  a27=sin(a9);
  a28=(a26*a27);
  a29=(a28*a12);
  a30=cos(a25);
  a31=sin(a6);
  a32=(a30*a31);
  a29=(a29+a32);
  a29=(a29*a8);
  a32=cos(a6);
  a33=(a30*a32);
  a34=(a28*a15);
  a33=(a33-a34);
  a33=(a33*a17);
  a29=(a29+a33);
  a33=cos(a9);
  a34=(a26*a33);
  a34=(a34*a18);
  a29=(a29-a34);
  a34=arg[0]? arg[0][3] : 0;
  a35=cos(a34);
  a36=cos(a1);
  a24=(a36*a24);
  a37=sin(a1);
  a21=(a37*a21);
  a24=(a24+a21);
  a21=(a35*a24);
  a38=sin(a34);
  a39=cos(a3);
  a20=(a39*a20);
  a40=(a38*a20);
  a21=(a21+a40);
  a21=(a29*a21);
  a22=(a22-a21);
  a21=sin(a25);
  a40=(a21*a31);
  a41=cos(a25);
  a42=(a41*a27);
  a43=(a42*a12);
  a40=(a40-a43);
  a40=(a40*a8);
  a43=(a42*a15);
  a44=(a21*a32);
  a43=(a43+a44);
  a43=(a43*a17);
  a40=(a40+a43);
  a43=(a41*a33);
  a43=(a43*a18);
  a40=(a40+a43);
  a43=cos(a34);
  a44=(a43*a20);
  a45=sin(a34);
  a46=(a45*a24);
  a44=(a44-a46);
  a44=(a40*a44);
  a22=(a22-a44);
  a44=-2.5840000000000002e-01;
  a46=(a44*a35);
  a47=(a46*a24);
  a48=(a44*a38);
  a20=(a48*a20);
  a47=(a47+a20);
  a20=-4.7827999999999998e-01;
  a24=(a20*a24);
  a47=(a47+a24);
  a22=(a22-a47);
  if (res[0]!=0) res[0][2]=a22;
  a22=cos(a5);
  a47=(a23*a22);
  a24=sin(a5);
  a49=(a19*a24);
  a50=(a16*a49);
  a47=(a47+a50);
  a47=(a14*a47);
  a22=(a36*a22);
  a49=(a37*a49);
  a22=(a22-a49);
  a49=(a35*a22);
  a24=(a39*a24);
  a50=(a38*a24);
  a49=(a49-a50);
  a49=(a29*a49);
  a47=(a47+a49);
  a49=(a43*a24);
  a50=(a45*a22);
  a49=(a49+a50);
  a49=(a40*a49);
  a47=(a47-a49);
  a49=(a46*a22);
  a24=(a48*a24);
  a49=(a49-a24);
  a22=(a20*a22);
  a49=(a49+a22);
  a47=(a47+a49);
  if (res[0]!=0) res[0][3]=a47;
  a47=cos(a9);
  a49=cos(a6);
  a22=(a47*a49);
  a24=arg[3]? arg[3][3] : 0;
  a22=(a22*a24);
  a50=sin(a6);
  a51=(a47*a50);
  a52=arg[3]? arg[3][4] : 0;
  a51=(a51*a52);
  a22=(a22-a51);
  a51=sin(a9);
  a53=arg[3]? arg[3][5] : 0;
  a51=(a51*a53);
  a22=(a22+a51);
  a51=cos(a1);
  a54=sin(a3);
  a55=cos(a5);
  a56=(a54*a55);
  a57=(a51*a56);
  a58=sin(a1);
  a59=sin(a5);
  a60=(a58*a59);
  a57=(a57-a60);
  a57=(a22*a57);
  a60=sin(a25);
  a61=sin(a9);
  a62=(a60*a61);
  a63=(a62*a49);
  a64=cos(a25);
  a65=sin(a6);
  a66=(a64*a65);
  a63=(a63+a66);
  a63=(a63*a24);
  a66=cos(a6);
  a67=(a64*a66);
  a68=(a62*a50);
  a67=(a67-a68);
  a67=(a67*a52);
  a63=(a63+a67);
  a67=cos(a9);
  a68=(a60*a67);
  a68=(a68*a53);
  a63=(a63-a68);
  a68=cos(a34);
  a69=cos(a1);
  a59=(a69*a59);
  a70=sin(a1);
  a56=(a70*a56);
  a59=(a59+a56);
  a56=(a68*a59);
  a71=sin(a34);
  a72=cos(a3);
  a55=(a72*a55);
  a73=(a71*a55);
  a56=(a56+a73);
  a56=(a63*a56);
  a57=(a57-a56);
  a56=sin(a25);
  a73=(a56*a65);
  a74=cos(a25);
  a75=(a74*a61);
  a76=(a75*a49);
  a73=(a73-a76);
  a73=(a73*a24);
  a76=(a75*a50);
  a77=(a56*a66);
  a76=(a76+a77);
  a76=(a76*a52);
  a73=(a73+a76);
  a76=(a74*a67);
  a76=(a76*a53);
  a73=(a73+a76);
  a76=cos(a34);
  a77=(a76*a55);
  a78=sin(a34);
  a79=(a78*a59);
  a77=(a77-a79);
  a77=(a73*a77);
  a57=(a57-a77);
  a77=(a44*a68);
  a79=(a77*a59);
  a80=(a44*a71);
  a55=(a80*a55);
  a79=(a79+a55);
  a59=(a20*a59);
  a79=(a79+a59);
  a57=(a57-a79);
  if (res[0]!=0) res[0][4]=a57;
  a57=cos(a5);
  a79=(a58*a57);
  a59=sin(a5);
  a55=(a54*a59);
  a81=(a51*a55);
  a79=(a79+a81);
  a79=(a22*a79);
  a57=(a69*a57);
  a55=(a70*a55);
  a57=(a57-a55);
  a55=(a68*a57);
  a59=(a72*a59);
  a81=(a71*a59);
  a55=(a55-a81);
  a55=(a63*a55);
  a79=(a79+a55);
  a55=(a76*a59);
  a81=(a78*a57);
  a55=(a55+a81);
  a55=(a73*a55);
  a79=(a79-a55);
  a55=(a77*a57);
  a59=(a80*a59);
  a55=(a55-a59);
  a57=(a20*a57);
  a55=(a55+a57);
  a79=(a79+a55);
  if (res[0]!=0) res[0][5]=a79;
  a79=sin(a3);
  a55=(a13*a79);
  a55=(a11*a55);
  a79=(a2*a79);
  a79=(a0*a79);
  a55=(a55-a79);
  a79=cos(a3);
  a79=(a10*a79);
  a55=(a55+a79);
  if (res[0]!=0) res[0][6]=a55;
  a55=sin(a5);
  a79=cos(a3);
  a57=(a55*a79);
  a59=(a2*a57);
  a59=(a0*a59);
  a57=(a13*a57);
  a57=(a11*a57);
  a59=(a59-a57);
  a57=sin(a3);
  a81=(a55*a57);
  a81=(a10*a81);
  a59=(a59+a81);
  if (res[0]!=0) res[0][7]=a59;
  a59=cos(a5);
  a79=(a59*a79);
  a13=(a13*a79);
  a13=(a11*a13);
  a2=(a2*a79);
  a2=(a0*a2);
  a13=(a13-a2);
  a57=(a59*a57);
  a10=(a10*a57);
  a13=(a13-a10);
  if (res[0]!=0) res[0][8]=a13;
  a13=cos(a3);
  a10=(a38*a13);
  a57=sin(a3);
  a2=(a37*a57);
  a79=(a35*a2);
  a10=(a10+a79);
  a10=(a29*a10);
  a57=(a16*a57);
  a57=(a14*a57);
  a10=(a10-a57);
  a57=(a43*a13);
  a79=(a45*a2);
  a57=(a57-a79);
  a57=(a40*a57);
  a10=(a10+a57);
  a13=(a48*a13);
  a57=(a46*a2);
  a13=(a13+a57);
  a2=(a20*a2);
  a13=(a13+a2);
  a10=(a10+a13);
  if (res[0]!=0) res[0][9]=a10;
  a10=sin(a5);
  a13=cos(a3);
  a2=(a10*a13);
  a57=(a16*a2);
  a57=(a14*a57);
  a79=sin(a3);
  a81=(a10*a79);
  a82=(a38*a81);
  a2=(a37*a2);
  a83=(a35*a2);
  a82=(a82-a83);
  a82=(a29*a82);
  a57=(a57+a82);
  a82=(a45*a2);
  a83=(a43*a81);
  a82=(a82+a83);
  a82=(a40*a82);
  a57=(a57+a82);
  a81=(a48*a81);
  a82=(a46*a2);
  a81=(a81-a82);
  a2=(a20*a2);
  a81=(a81-a2);
  a57=(a57+a81);
  if (res[0]!=0) res[0][10]=a57;
  a57=cos(a5);
  a13=(a57*a13);
  a81=(a37*a13);
  a2=(a35*a81);
  a79=(a57*a79);
  a82=(a38*a79);
  a2=(a2-a82);
  a2=(a29*a2);
  a13=(a16*a13);
  a13=(a14*a13);
  a2=(a2-a13);
  a13=(a43*a79);
  a82=(a45*a81);
  a13=(a13+a82);
  a13=(a40*a13);
  a2=(a2-a13);
  a13=(a46*a81);
  a48=(a48*a79);
  a13=(a13-a48);
  a81=(a20*a81);
  a13=(a13+a81);
  a2=(a2+a13);
  if (res[0]!=0) res[0][11]=a2;
  a2=cos(a3);
  a13=(a71*a2);
  a81=sin(a3);
  a48=(a70*a81);
  a79=(a68*a48);
  a13=(a13+a79);
  a13=(a63*a13);
  a81=(a51*a81);
  a81=(a22*a81);
  a13=(a13-a81);
  a81=(a76*a2);
  a79=(a78*a48);
  a81=(a81-a79);
  a81=(a73*a81);
  a13=(a13+a81);
  a2=(a80*a2);
  a81=(a77*a48);
  a2=(a2+a81);
  a48=(a20*a48);
  a2=(a2+a48);
  a13=(a13+a2);
  if (res[0]!=0) res[0][12]=a13;
  a13=sin(a5);
  a2=cos(a3);
  a48=(a13*a2);
  a81=(a51*a48);
  a81=(a22*a81);
  a79=sin(a3);
  a82=(a13*a79);
  a83=(a71*a82);
  a48=(a70*a48);
  a84=(a68*a48);
  a83=(a83-a84);
  a83=(a63*a83);
  a81=(a81+a83);
  a83=(a78*a48);
  a84=(a76*a82);
  a83=(a83+a84);
  a83=(a73*a83);
  a81=(a81+a83);
  a82=(a80*a82);
  a83=(a77*a48);
  a82=(a82-a83);
  a48=(a20*a48);
  a82=(a82-a48);
  a81=(a81+a82);
  if (res[0]!=0) res[0][13]=a81;
  a81=cos(a5);
  a2=(a81*a2);
  a82=(a70*a2);
  a48=(a68*a82);
  a79=(a81*a79);
  a83=(a71*a79);
  a48=(a48-a83);
  a48=(a63*a48);
  a2=(a51*a2);
  a2=(a22*a2);
  a48=(a48-a2);
  a2=(a76*a79);
  a83=(a78*a82);
  a2=(a2+a83);
  a2=(a73*a2);
  a48=(a48-a2);
  a2=(a77*a82);
  a80=(a80*a79);
  a2=(a2-a80);
  a82=(a20*a82);
  a2=(a2+a82);
  a48=(a48+a2);
  if (res[0]!=0) res[0][14]=a48;
  a48=cos(a3);
  a2=sin(a1);
  a82=(a48*a2);
  a82=(a0*a82);
  a80=cos(a1);
  a48=(a48*a80);
  a48=(a11*a48);
  a82=(a82+a48);
  a82=(-a82);
  if (res[0]!=0) res[0][15]=a82;
  a82=cos(a5);
  a48=cos(a1);
  a79=(a82*a48);
  a55=(a55*a4);
  a83=(a55*a2);
  a79=(a79-a83);
  a79=(a0*a79);
  a83=sin(a1);
  a82=(a82*a83);
  a55=(a55*a80);
  a82=(a82+a55);
  a82=(a11*a82);
  a79=(a79-a82);
  if (res[0]!=0) res[0][16]=a79;
  a79=sin(a5);
  a48=(a79*a48);
  a59=(a59*a4);
  a2=(a59*a2);
  a48=(a48+a2);
  a0=(a0*a48);
  a59=(a59*a80);
  a79=(a79*a83);
  a59=(a59-a79);
  a11=(a11*a59);
  a0=(a0+a11);
  if (res[0]!=0) res[0][17]=a0;
  a0=cos(a3);
  a11=cos(a1);
  a59=(a0*a11);
  a79=(a45*a59);
  a79=(a40*a79);
  a83=sin(a1);
  a80=(a0*a83);
  a80=(a14*a80);
  a48=(a35*a59);
  a48=(a29*a48);
  a80=(a80+a48);
  a79=(a79-a80);
  a80=(a46*a59);
  a59=(a20*a59);
  a80=(a80+a59);
  a79=(a79-a80);
  if (res[0]!=0) res[0][18]=a79;
  a79=cos(a5);
  a80=cos(a1);
  a59=(a79*a80);
  a48=(a10*a19);
  a2=(a48*a83);
  a59=(a59-a2);
  a59=(a14*a59);
  a2=sin(a1);
  a4=(a79*a2);
  a82=(a48*a11);
  a4=(a4+a82);
  a82=(a35*a4);
  a82=(a29*a82);
  a59=(a59-a82);
  a82=(a45*a4);
  a82=(a40*a82);
  a59=(a59+a82);
  a82=(a46*a4);
  a4=(a20*a4);
  a82=(a82+a4);
  a59=(a59-a82);
  if (res[0]!=0) res[0][19]=a59;
  a59=sin(a5);
  a80=(a59*a80);
  a19=(a57*a19);
  a83=(a19*a83);
  a80=(a80+a83);
  a14=(a14*a80);
  a11=(a19*a11);
  a2=(a59*a2);
  a11=(a11-a2);
  a2=(a35*a11);
  a2=(a29*a2);
  a14=(a14+a2);
  a2=(a45*a11);
  a2=(a40*a2);
  a14=(a14-a2);
  a46=(a46*a11);
  a11=(a20*a11);
  a46=(a46+a11);
  a14=(a14+a46);
  if (res[0]!=0) res[0][20]=a14;
  a14=cos(a3);
  a46=cos(a1);
  a11=(a14*a46);
  a2=(a78*a11);
  a2=(a73*a2);
  a80=sin(a1);
  a83=(a14*a80);
  a83=(a22*a83);
  a82=(a68*a11);
  a82=(a63*a82);
  a83=(a83+a82);
  a2=(a2-a83);
  a83=(a77*a11);
  a11=(a20*a11);
  a83=(a83+a11);
  a2=(a2-a83);
  if (res[0]!=0) res[0][21]=a2;
  a2=cos(a5);
  a83=cos(a1);
  a11=(a2*a83);
  a82=(a13*a54);
  a4=(a82*a80);
  a11=(a11-a4);
  a11=(a22*a11);
  a1=sin(a1);
  a4=(a2*a1);
  a55=(a82*a46);
  a4=(a4+a55);
  a55=(a68*a4);
  a55=(a63*a55);
  a11=(a11-a55);
  a55=(a78*a4);
  a55=(a73*a55);
  a11=(a11+a55);
  a55=(a77*a4);
  a4=(a20*a4);
  a55=(a55+a4);
  a11=(a11-a55);
  if (res[0]!=0) res[0][22]=a11;
  a5=sin(a5);
  a83=(a5*a83);
  a54=(a81*a54);
  a80=(a54*a80);
  a83=(a83+a80);
  a22=(a22*a83);
  a46=(a54*a46);
  a1=(a5*a1);
  a46=(a46-a1);
  a1=(a68*a46);
  a1=(a63*a1);
  a22=(a22+a1);
  a1=(a78*a46);
  a1=(a73*a1);
  a22=(a22-a1);
  a77=(a77*a46);
  a20=(a20*a46);
  a77=(a77+a20);
  a22=(a22+a77);
  if (res[0]!=0) res[0][23]=a22;
  a22=sin(a3);
  a77=cos(a34);
  a20=(a22*a77);
  a46=(a0*a37);
  a1=sin(a34);
  a83=(a46*a1);
  a20=(a20+a83);
  a20=(a29*a20);
  a83=cos(a34);
  a80=(a46*a83);
  a11=sin(a34);
  a55=(a22*a11);
  a80=(a80-a55);
  a80=(a40*a80);
  a20=(a20+a80);
  a80=(a44*a77);
  a55=(a22*a80);
  a4=(a44*a1);
  a84=(a46*a4);
  a55=(a55+a84);
  a20=(a20+a55);
  if (res[0]!=0) res[0][24]=a20;
  a20=(a79*a36);
  a55=(a48*a37);
  a20=(a20-a55);
  a55=(a20*a1);
  a10=(a10*a39);
  a84=(a10*a77);
  a55=(a55+a84);
  a55=(a29*a55);
  a84=(a20*a83);
  a85=(a10*a11);
  a84=(a84-a85);
  a84=(a40*a84);
  a55=(a55+a84);
  a84=(a20*a4);
  a85=(a10*a80);
  a84=(a84+a85);
  a55=(a55+a84);
  a55=(-a55);
  if (res[0]!=0) res[0][25]=a55;
  a57=(a57*a39);
  a77=(a57*a77);
  a37=(a19*a37);
  a36=(a59*a36);
  a37=(a37+a36);
  a1=(a37*a1);
  a77=(a77-a1);
  a29=(a29*a77);
  a11=(a57*a11);
  a83=(a37*a83);
  a11=(a11+a83);
  a40=(a40*a11);
  a29=(a29-a40);
  a80=(a57*a80);
  a4=(a37*a4);
  a80=(a80-a4);
  a29=(a29+a80);
  if (res[0]!=0) res[0][26]=a29;
  a3=sin(a3);
  a29=cos(a34);
  a80=(a3*a29);
  a4=(a14*a70);
  a40=sin(a34);
  a11=(a4*a40);
  a80=(a80+a11);
  a80=(a63*a80);
  a11=cos(a34);
  a83=(a4*a11);
  a34=sin(a34);
  a77=(a3*a34);
  a83=(a83-a77);
  a83=(a73*a83);
  a80=(a80+a83);
  a83=(a44*a29);
  a77=(a3*a83);
  a44=(a44*a40);
  a1=(a4*a44);
  a77=(a77+a1);
  a80=(a80+a77);
  if (res[0]!=0) res[0][27]=a80;
  a80=(a2*a69);
  a77=(a82*a70);
  a80=(a80-a77);
  a77=(a80*a40);
  a13=(a13*a72);
  a1=(a13*a29);
  a77=(a77+a1);
  a77=(a63*a77);
  a1=(a80*a11);
  a36=(a13*a34);
  a1=(a1-a36);
  a1=(a73*a1);
  a77=(a77+a1);
  a1=(a80*a44);
  a36=(a13*a83);
  a1=(a1+a36);
  a77=(a77+a1);
  a77=(-a77);
  if (res[0]!=0) res[0][28]=a77;
  a81=(a81*a72);
  a29=(a81*a29);
  a70=(a54*a70);
  a69=(a5*a69);
  a70=(a70+a69);
  a40=(a70*a40);
  a29=(a29-a40);
  a63=(a63*a29);
  a34=(a81*a34);
  a11=(a70*a11);
  a34=(a34+a11);
  a73=(a73*a34);
  a63=(a63-a73);
  a83=(a81*a83);
  a44=(a70*a44);
  a83=(a83-a44);
  a63=(a63+a83);
  if (res[0]!=0) res[0][29]=a63;
  a63=(a22*a38);
  a83=(a46*a35);
  a63=(a63-a83);
  a83=cos(a25);
  a44=(a27*a83);
  a73=(a12*a44);
  a34=sin(a25);
  a11=(a31*a34);
  a73=(a73-a11);
  a73=(a8*a73);
  a34=(a32*a34);
  a44=(a15*a44);
  a34=(a34+a44);
  a34=(a17*a34);
  a73=(a73-a34);
  a83=(a33*a83);
  a83=(a18*a83);
  a73=(a73-a83);
  a83=(a63*a73);
  a46=(a46*a45);
  a22=(a22*a43);
  a46=(a46+a22);
  a22=cos(a25);
  a31=(a31*a22);
  a34=sin(a25);
  a27=(a27*a34);
  a44=(a12*a27);
  a31=(a31+a44);
  a31=(a8*a31);
  a32=(a32*a22);
  a27=(a15*a27);
  a32=(a32-a27);
  a32=(a17*a32);
  a31=(a31+a32);
  a33=(a33*a34);
  a33=(a18*a33);
  a31=(a31-a33);
  a33=(a46*a31);
  a83=(a83+a33);
  if (res[0]!=0) res[0][30]=a83;
  a83=(a20*a35);
  a33=(a10*a38);
  a83=(a83-a33);
  a33=(a83*a73);
  a20=(a20*a45);
  a10=(a10*a43);
  a20=(a20+a10);
  a10=(a20*a31);
  a33=(a33-a10);
  if (res[0]!=0) res[0][31]=a33;
  a35=(a37*a35);
  a38=(a57*a38);
  a35=(a35+a38);
  a73=(a35*a73);
  a57=(a57*a43);
  a37=(a37*a45);
  a57=(a57-a37);
  a31=(a57*a31);
  a73=(a73+a31);
  if (res[0]!=0) res[0][32]=a73;
  a73=(a3*a71);
  a31=(a4*a68);
  a73=(a73-a31);
  a31=cos(a25);
  a37=(a61*a31);
  a45=(a49*a37);
  a43=sin(a25);
  a38=(a65*a43);
  a45=(a45-a38);
  a45=(a24*a45);
  a43=(a66*a43);
  a37=(a50*a37);
  a43=(a43+a37);
  a43=(a52*a43);
  a45=(a45-a43);
  a31=(a67*a31);
  a31=(a53*a31);
  a45=(a45-a31);
  a31=(a73*a45);
  a4=(a4*a78);
  a3=(a3*a76);
  a4=(a4+a3);
  a3=cos(a25);
  a65=(a65*a3);
  a25=sin(a25);
  a61=(a61*a25);
  a43=(a49*a61);
  a65=(a65+a43);
  a65=(a24*a65);
  a66=(a66*a3);
  a61=(a50*a61);
  a66=(a66-a61);
  a66=(a52*a66);
  a65=(a65+a66);
  a67=(a67*a25);
  a67=(a53*a67);
  a65=(a65-a67);
  a67=(a4*a65);
  a31=(a31+a67);
  if (res[0]!=0) res[0][33]=a31;
  a31=(a80*a68);
  a67=(a13*a71);
  a31=(a31-a67);
  a67=(a31*a45);
  a80=(a80*a78);
  a13=(a13*a76);
  a80=(a80+a13);
  a13=(a80*a65);
  a67=(a67-a13);
  if (res[0]!=0) res[0][34]=a67;
  a68=(a70*a68);
  a71=(a81*a71);
  a68=(a68+a71);
  a45=(a68*a45);
  a81=(a81*a76);
  a70=(a70*a78);
  a81=(a81-a70);
  a65=(a81*a65);
  a45=(a45+a65);
  if (res[0]!=0) res[0][35]=a45;
  a0=(a0*a16);
  a45=sin(a9);
  a65=(a15*a45);
  a65=(a17*a65);
  a45=(a12*a45);
  a45=(a8*a45);
  a65=(a65-a45);
  a45=cos(a9);
  a45=(a18*a45);
  a65=(a65+a45);
  a45=(a0*a65);
  a70=cos(a9);
  a78=(a26*a70);
  a76=(a12*a78);
  a76=(a8*a76);
  a78=(a15*a78);
  a78=(a17*a78);
  a76=(a76-a78);
  a78=sin(a9);
  a26=(a26*a78);
  a26=(a18*a26);
  a76=(a76+a26);
  a26=(a63*a76);
  a45=(a45+a26);
  a70=(a41*a70);
  a15=(a15*a70);
  a15=(a17*a15);
  a12=(a12*a70);
  a12=(a8*a12);
  a15=(a15-a12);
  a41=(a41*a78);
  a18=(a18*a41);
  a15=(a15-a18);
  a18=(a46*a15);
  a45=(a45+a18);
  if (res[0]!=0) res[0][36]=a45;
  a48=(a48*a16);
  a79=(a79*a23);
  a48=(a48+a79);
  a79=(a48*a65);
  a45=(a83*a76);
  a79=(a79+a45);
  a45=(a20*a15);
  a79=(a79-a45);
  if (res[0]!=0) res[0][37]=a79;
  a59=(a59*a23);
  a19=(a19*a16);
  a59=(a59-a19);
  a65=(a59*a65);
  a76=(a35*a76);
  a65=(a65+a76);
  a15=(a57*a15);
  a65=(a65+a15);
  if (res[0]!=0) res[0][38]=a65;
  a14=(a14*a51);
  a65=sin(a9);
  a15=(a50*a65);
  a15=(a52*a15);
  a65=(a49*a65);
  a65=(a24*a65);
  a15=(a15-a65);
  a65=cos(a9);
  a65=(a53*a65);
  a15=(a15+a65);
  a65=(a14*a15);
  a76=cos(a9);
  a19=(a60*a76);
  a16=(a49*a19);
  a16=(a24*a16);
  a19=(a50*a19);
  a19=(a52*a19);
  a16=(a16-a19);
  a9=sin(a9);
  a60=(a60*a9);
  a60=(a53*a60);
  a16=(a16+a60);
  a60=(a73*a16);
  a65=(a65+a60);
  a76=(a74*a76);
  a50=(a50*a76);
  a50=(a52*a50);
  a49=(a49*a76);
  a49=(a24*a49);
  a50=(a50-a49);
  a74=(a74*a9);
  a53=(a53*a74);
  a50=(a50-a53);
  a53=(a4*a50);
  a65=(a65+a53);
  if (res[0]!=0) res[0][39]=a65;
  a82=(a82*a51);
  a2=(a2*a58);
  a82=(a82+a2);
  a2=(a82*a15);
  a65=(a31*a16);
  a2=(a2+a65);
  a65=(a80*a50);
  a2=(a2-a65);
  if (res[0]!=0) res[0][40]=a2;
  a5=(a5*a58);
  a54=(a54*a51);
  a5=(a5-a54);
  a15=(a5*a15);
  a16=(a68*a16);
  a15=(a15+a16);
  a50=(a81*a50);
  a15=(a15+a50);
  if (res[0]!=0) res[0][41]=a15;
  a15=cos(a6);
  a50=(a30*a15);
  a16=sin(a6);
  a54=(a28*a16);
  a50=(a50-a54);
  a50=(a8*a50);
  a54=sin(a6);
  a30=(a30*a54);
  a51=cos(a6);
  a28=(a28*a51);
  a30=(a30+a28);
  a30=(a17*a30);
  a50=(a50-a30);
  a63=(a63*a50);
  a30=(a7*a16);
  a30=(a8*a30);
  a7=(a7*a51);
  a7=(a17*a7);
  a30=(a30+a7);
  a0=(a0*a30);
  a63=(a63-a0);
  a15=(a21*a15);
  a16=(a42*a16);
  a15=(a15+a16);
  a8=(a8*a15);
  a42=(a42*a51);
  a21=(a21*a54);
  a42=(a42-a21);
  a17=(a17*a42);
  a8=(a8+a17);
  a46=(a46*a8);
  a63=(a63+a46);
  if (res[0]!=0) res[0][42]=a63;
  a83=(a83*a50);
  a48=(a48*a30);
  a83=(a83-a48);
  a20=(a20*a8);
  a83=(a83-a20);
  if (res[0]!=0) res[0][43]=a83;
  a35=(a35*a50);
  a59=(a59*a30);
  a35=(a35-a59);
  a57=(a57*a8);
  a35=(a35+a57);
  if (res[0]!=0) res[0][44]=a35;
  a35=cos(a6);
  a57=(a64*a35);
  a8=sin(a6);
  a59=(a62*a8);
  a57=(a57-a59);
  a57=(a24*a57);
  a59=sin(a6);
  a64=(a64*a59);
  a6=cos(a6);
  a62=(a62*a6);
  a64=(a64+a62);
  a64=(a52*a64);
  a57=(a57-a64);
  a73=(a73*a57);
  a64=(a47*a8);
  a64=(a24*a64);
  a47=(a47*a6);
  a47=(a52*a47);
  a64=(a64+a47);
  a14=(a14*a64);
  a73=(a73-a14);
  a35=(a56*a35);
  a8=(a75*a8);
  a35=(a35+a8);
  a24=(a24*a35);
  a75=(a75*a6);
  a56=(a56*a59);
  a75=(a75-a56);
  a52=(a52*a75);
  a24=(a24+a52);
  a4=(a4*a24);
  a73=(a73+a4);
  if (res[0]!=0) res[0][45]=a73;
  a31=(a31*a57);
  a82=(a82*a64);
  a31=(a31-a82);
  a80=(a80*a24);
  a31=(a31-a80);
  if (res[0]!=0) res[0][46]=a31;
  a68=(a68*a57);
  a5=(a5*a64);
  a68=(a68-a5);
  a81=(a81*a24);
  a68=(a68+a81);
  if (res[0]!=0) res[0][47]=a68;
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
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* f_J_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
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
  casadi_real w[150];
  casadi_int *iw = 0;
  const casadi_real* arg[4] = {0};
  casadi_real* res[1] = {0};
  if (argc>4) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"f_J\" failed. Too many input arguments (%d, max 4)", argc);
  if (resc>1) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"f_J\" failed. Too many output arguments (%d, max 1)", resc);
  if (--argc>=0) arg[0] = casadi_from_mex(argv[0], w, casadi_s0, w+64);
  if (--argc>=0) arg[1] = casadi_from_mex(argv[1], w+7, casadi_s1, w+64);
  if (--argc>=0) arg[2] = casadi_from_mex(argv[2], w+10, casadi_s2, w+64);
  if (--argc>=0) arg[3] = casadi_from_mex(argv[3], w+10, casadi_s3, w+64);
  --resc;
  res[0] = w+16;
  i = f_J(arg, res, iw, w+64, 0);
  if (i) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"f_J\" failed.");
  if (res[0]) resv[0] = casadi_to_mex(casadi_s4, res[0]);
}
#endif

casadi_int main_f_J(casadi_int argc, char* argv[]) {
  casadi_int j;
  casadi_real* a;
  const casadi_real* r;
  casadi_int flag;
  casadi_int *iw = 0;
  casadi_real w[150];
  const casadi_real* arg[4];
  casadi_real* res[1];
  arg[0] = w+0;
  arg[1] = w+7;
  arg[2] = w+10;
  arg[3] = w+10;
  res[0] = w+16;
  a = w;
  for (j=0; j<16; ++j) if (scanf("%lg", a++)<=0) return 2;
  flag = f_J(arg, res, iw, w+64, 0);
  if (flag) return flag;
  r = w+16;
  for (j=0; j<48; ++j) CASADI_PRINTF("%g ", *r++);
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