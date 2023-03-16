//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: driver.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 16-Jan-2023 18:23:03
//

// Include Files
#include "driver.h"
#include "PresolveWorkingSet.h"
#include "RemoveDependentEq_.h"
#include "RemoveDependentIneq_.h"
#include "computeFirstOrderOpt.h"
#include "computeFval.h"
#include "feasibleX0ForWorkingSet.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "removeConstr.h"
#include "../rt_nonfinite.h"
#include "setProblemType.h"
#include "solveLSQ_internal_types.h"
#include "../coder_array.h"

// Type Definitions
struct g_struct_T {
  char SolverName[6];
};

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &H
//                const ::coder::array<double, 1U> &f
//                struct_T *solution
//                e_struct_T *memspace
//                f_struct_T *workingset
//                c_struct_T *qrmanager
//                d_struct_T *cholmanager
//                b_struct_T *objective
//                int runTimeOptions_MaxIterations
//                double runTimeOptions_ProbRelTolFactor
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void driver(const ::coder::array<double, 2U> &H,
            const ::coder::array<double, 1U> &f, struct_T *solution,
            e_struct_T *memspace, f_struct_T *workingset, c_struct_T *qrmanager,
            d_struct_T *cholmanager, b_struct_T *objective,
            int runTimeOptions_MaxIterations,
            double runTimeOptions_ProbRelTolFactor)
{
  static const char cv[128]{
      '\x00', '\x01', '\x02', '\x03', '\x04', '\x05', '\x06', '\x07', '\x08',
      '\x09', '\x0a', '\x0b', '\x0c', '\x0d', '\x0e', '\x0f', '\x10', '\x11',
      '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18', '\x19', '\x1a',
      '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ',    '!',    '\"',   '#',
      '$',    '%',    '&',    '\'',   '(',    ')',    '*',    '+',    ',',
      '-',    '.',    '/',    '0',    '1',    '2',    '3',    '4',    '5',
      '6',    '7',    '8',    '9',    ':',    ';',    '<',    '=',    '>',
      '?',    '@',    'a',    'b',    'c',    'd',    'e',    'f',    'g',
      'h',    'i',    'j',    'k',    'l',    'm',    'n',    'o',    'p',
      'q',    'r',    's',    't',    'u',    'v',    'w',    'x',    'y',
      'z',    '[',    '\\',   ']',    '^',    '_',    '`',    'a',    'b',
      'c',    'd',    'e',    'f',    'g',    'h',    'i',    'j',    'k',
      'l',    'm',    'n',    'o',    'p',    'q',    'r',    's',    't',
      'u',    'v',    'w',    'x',    'y',    'z',    '{',    '|',    '}',
      '~',    '\x7f'};
  static const char cv1[6]{'l', 's', 'q', 'l', 'i', 'n'};
  static const char t0_SolverName[6]{'l', 's', 'q', 'l', 'i', 'n'};
  g_struct_T options;
  double constrViolation;
  int idxEndIneq;
  int idxStartIneq;
  int nVar;
  bool guard1{false};
  bool okWorkingSet;
  solution->iterations = 0;
  nVar = workingset->nVar - 1;
  idxStartIneq = workingset->sizes[0];
  for (idxEndIneq = 0; idxEndIneq < idxStartIneq; idxEndIneq++) {
    solution->xstar[workingset->indexFixed[idxEndIneq] - 1] = 1.0;
  }
  idxStartIneq = workingset->sizes[3];
  for (idxEndIneq = 0; idxEndIneq < idxStartIneq; idxEndIneq++) {
    if (workingset
            ->isActiveConstr[(workingset->isActiveIdx[3] + idxEndIneq) - 1]) {
      solution->xstar[workingset->indexLB[idxEndIneq] - 1] =
          -workingset->lb[workingset->indexLB[idxEndIneq] - 1];
    }
  }
  idxStartIneq = workingset->sizes[4];
  for (idxEndIneq = 0; idxEndIneq < idxStartIneq; idxEndIneq++) {
    if (workingset
            ->isActiveConstr[(workingset->isActiveIdx[4] + idxEndIneq) - 1]) {
      solution->xstar[workingset->indexUB[idxEndIneq] - 1] = 1.0;
    }
  }
  solution->state = 82;
  idxStartIneq =
      initialize::RemoveDependentEq_(memspace, workingset, qrmanager);
  if ((idxStartIneq != -1) && (workingset->nActiveConstr <= qrmanager->ldq)) {
    initialize::RemoveDependentIneq_(workingset, qrmanager, memspace);
    okWorkingSet = initialize::feasibleX0ForWorkingSet(
        memspace->workspace_double, solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      initialize::b_RemoveDependentIneq_(workingset, qrmanager, memspace);
      okWorkingSet = initialize::feasibleX0ForWorkingSet(
          memspace->workspace_double, solution->xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      constrViolation =
          WorkingSet::maxConstraintViolation(workingset, solution->xstar);
      if (constrViolation > 1.0E-8) {
        solution->state = -2;
      }
    }
  } else {
    solution->state = -3;
    idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
    idxEndIneq = workingset->nActiveConstr;
    for (int idx_global{idxStartIneq}; idx_global <= idxEndIneq; idx_global++) {
      workingset->isActiveConstr
          [(workingset->isActiveIdx[workingset->Wid[idx_global - 1] - 1] +
            workingset->Wlocalidx[idx_global - 1]) -
           2] = false;
    }
    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr =
        workingset->nWConstr[0] + workingset->nWConstr[1];
  }
  for (idxStartIneq = 0; idxStartIneq < 6; idxStartIneq++) {
    options.SolverName[idxStartIneq] = t0_SolverName[idxStartIneq];
  }
  if (solution->state >= 0) {
    int b_nVar;
    int nVarP1;
    bool exitg1;
    solution->iterations = 0;
    solution->maxConstr =
        WorkingSet::maxConstraintViolation(workingset, solution->xstar);
    guard1 = false;
    if (solution->maxConstr > 1.0E-8) {
      b_nVar = workingset->nVar;
      nVarP1 = workingset->nVar;
      solution->xstar[workingset->nVar] = solution->maxConstr + 1.0;
      WorkingSet::setProblemType(workingset, 1);
      idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
      idxEndIneq = workingset->nActiveConstr;
      for (int idx_global{idxStartIneq}; idx_global <= idxEndIneq;
           idx_global++) {
        workingset->isActiveConstr
            [(workingset->isActiveIdx[workingset->Wid[idx_global - 1] - 1] +
              workingset->Wlocalidx[idx_global - 1]) -
             2] = false;
      }
      workingset->nWConstr[2] = 0;
      workingset->nWConstr[3] = 0;
      workingset->nWConstr[4] = 0;
      workingset->nActiveConstr =
          workingset->nWConstr[0] + workingset->nWConstr[1];
      objective->prev_objtype = objective->objtype;
      objective->prev_nvar = objective->nvar;
      objective->prev_hasLinear = true;
      objective->objtype = 5;
      objective->nvar = nVarP1 + 1;
      objective->gammaScalar = 1.0;
      objective->hasLinear = true;
      solution->fstar = Objective::computeFval(
          objective, memspace->workspace_double, H, f, solution->xstar);
      solution->state = 5;
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, 1.0E-8, 1.4901161193847657E-10,
              runTimeOptions_MaxIterations, runTimeOptions_ProbRelTolFactor,
              true);
      if (workingset->isActiveConstr
              [(workingset->isActiveIdx[3] + workingset->sizes[3]) - 2]) {
        idxEndIneq = workingset->sizes[0];
        exitg1 = false;
        while ((!exitg1) && (idxEndIneq + 1 <= workingset->nActiveConstr)) {
          if ((workingset->Wid[idxEndIneq] == 4) &&
              (workingset->Wlocalidx[idxEndIneq] == workingset->sizes[3])) {
            WorkingSet::removeConstr(workingset, idxEndIneq + 1);
            exitg1 = true;
          } else {
            idxEndIneq++;
          }
        }
      }
      idxStartIneq = workingset->nActiveConstr;
      idxEndIneq = workingset->sizes[0];
      while ((idxStartIneq > idxEndIneq) && (idxStartIneq > b_nVar)) {
        WorkingSet::removeConstr(workingset, idxStartIneq);
        idxStartIneq--;
      }
      solution->maxConstr = solution->xstar[nVarP1];
      WorkingSet::setProblemType(workingset, 3);
      objective->objtype = objective->prev_objtype;
      objective->nvar = objective->prev_nvar;
      objective->hasLinear = true;
      if (solution->state != 0) {
        solution->maxConstr =
            WorkingSet::maxConstraintViolation(workingset, solution->xstar);
        if (solution->maxConstr > 1.0E-8) {
          idxStartIneq = workingset->mConstrMax;
          for (idxEndIneq = 0; idxEndIneq < idxStartIneq; idxEndIneq++) {
            solution->lambda[idxEndIneq] = 0.0;
          }
          solution->fstar = Objective::computeFval(
              objective, memspace->workspace_double, H, f, solution->xstar);
          solution->state = -2;
        } else {
          if (solution->maxConstr > 0.0) {
            for (idxEndIneq = 0; idxEndIneq <= nVar; idxEndIneq++) {
              solution->searchDir[idxEndIneq] = solution->xstar[idxEndIneq];
            }
            initialize::PresolveWorkingSet(solution, memspace, workingset,
                                           qrmanager);
            constrViolation =
                WorkingSet::maxConstraintViolation(workingset, solution->xstar);
            if (constrViolation >= solution->maxConstr) {
              solution->maxConstr = constrViolation;
              for (idxEndIneq = 0; idxEndIneq <= nVar; idxEndIneq++) {
                solution->xstar[idxEndIneq] = solution->searchDir[idxEndIneq];
              }
            }
          }
          guard1 = true;
        }
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, -1.0E+20, 1.0E-8, runTimeOptions_MaxIterations,
              runTimeOptions_ProbRelTolFactor, true);
      okWorkingSet = false;
      idxStartIneq = 0;
      int exitg2;
      do {
        exitg2 = 0;
        if (idxStartIneq < 6) {
          if (cv[static_cast<int>(options.SolverName[idxStartIneq])] !=
              cv[static_cast<int>(cv1[idxStartIneq])]) {
            exitg2 = 1;
          } else {
            idxStartIneq++;
          }
        } else {
          okWorkingSet = true;
          exitg2 = 1;
        }
      } while (exitg2 == 0);
      if (okWorkingSet && (solution->state != -6)) {
        solution->maxConstr =
            WorkingSet::maxConstraintViolation(workingset, solution->xstar);
        parseoutput::computeFirstOrderOpt(solution, objective, workingset->nVar,
                                          workingset->ldA, workingset->ATwset,
                                          workingset->nActiveConstr,
                                          memspace->workspace_double);
        while ((solution->iterations < runTimeOptions_MaxIterations) &&
               ((solution->state == -7) ||
                ((solution->state == 1) &&
                 ((solution->maxConstr > 1.0E-8) ||
                  (solution->firstorderopt >
                   1.0E-8 * runTimeOptions_ProbRelTolFactor))))) {
          initialize::feasibleX0ForWorkingSet(memspace->workspace_double,
                                              solution->xstar, workingset,
                                              qrmanager);
          initialize::PresolveWorkingSet(solution, memspace, workingset,
                                         qrmanager);
          b_nVar = workingset->probType;
          nVar = workingset->nVar;
          nVarP1 = workingset->nVar;
          solution->xstar[workingset->nVar] = solution->maxConstr + 1.0;
          if (workingset->probType == 3) {
            idxStartIneq = 1;
          } else {
            idxStartIneq = 4;
          }
          WorkingSet::setProblemType(workingset, idxStartIneq);
          idxStartIneq =
              (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          idxEndIneq = workingset->nActiveConstr;
          for (int idx_global{idxStartIneq}; idx_global <= idxEndIneq;
               idx_global++) {
            workingset->isActiveConstr
                [(workingset->isActiveIdx[workingset->Wid[idx_global - 1] - 1] +
                  workingset->Wlocalidx[idx_global - 1]) -
                 2] = false;
          }
          workingset->nWConstr[2] = 0;
          workingset->nWConstr[3] = 0;
          workingset->nWConstr[4] = 0;
          workingset->nActiveConstr =
              workingset->nWConstr[0] + workingset->nWConstr[1];
          objective->prev_objtype = objective->objtype;
          objective->prev_nvar = objective->nvar;
          objective->prev_hasLinear = true;
          objective->objtype = 5;
          objective->nvar = nVarP1 + 1;
          objective->gammaScalar = 1.0;
          objective->hasLinear = true;
          solution->fstar = Objective::computeFval(
              objective, memspace->workspace_double, H, f, solution->xstar);
          solution->state = 5;
          iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                  objective, 1.0E-8, 1.4901161193847657E-10,
                  runTimeOptions_MaxIterations, runTimeOptions_ProbRelTolFactor,
                  false);
          if (workingset->isActiveConstr
                  [(workingset->isActiveIdx[3] + workingset->sizes[3]) - 2]) {
            idxEndIneq = workingset->sizes[0];
            exitg1 = false;
            while ((!exitg1) && (idxEndIneq + 1 <= workingset->nActiveConstr)) {
              if ((workingset->Wid[idxEndIneq] == 4) &&
                  (workingset->Wlocalidx[idxEndIneq] == workingset->sizes[3])) {
                WorkingSet::removeConstr(workingset, idxEndIneq + 1);
                exitg1 = true;
              } else {
                idxEndIneq++;
              }
            }
          }
          idxStartIneq = workingset->nActiveConstr;
          idxEndIneq = workingset->sizes[0];
          while ((idxStartIneq > idxEndIneq) && (idxStartIneq > nVar)) {
            WorkingSet::removeConstr(workingset, idxStartIneq);
            idxStartIneq--;
          }
          solution->maxConstr = solution->xstar[nVarP1];
          WorkingSet::setProblemType(workingset, b_nVar);
          objective->objtype = objective->prev_objtype;
          objective->nvar = objective->prev_nvar;
          objective->hasLinear = true;
          iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                  objective, -1.0E+20, 1.0E-8, runTimeOptions_MaxIterations,
                  runTimeOptions_ProbRelTolFactor, false);
          solution->maxConstr =
              WorkingSet::maxConstraintViolation(workingset, solution->xstar);
          parseoutput::computeFirstOrderOpt(
              solution, objective, workingset->nVar, workingset->ldA,
              workingset->ATwset, workingset->nActiveConstr,
              memspace->workspace_double);
        }
      }
    }
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for driver.cpp
//
// [EOF]
//
